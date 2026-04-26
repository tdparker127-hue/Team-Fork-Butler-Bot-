"""
robot_controller.py
Jetson Nano — main control loop

Receives Xbox One controller input over Bluetooth (via pygame) and sends
command strings over USB serial to two ESP32s:

  /dev/Drive  →  "fl:X;bl:X;fr:X;br:X;\n"   (wheel velocity setpoints, rad/s)
  /dev/Arm    →  "lift:X;grip:Y;\n"           (arm rate commands, rad/s)

Receives IMU telemetry back from both ESPs (lines starting with "IMU:").
All higher-level kinematics live here; the ESPs are pure actuator nodes.

Xbox One BT axis/button layout (pygame on Linux):
  Axes:    0=LX  1=LY  2=RX  3=RY  4=LT(-1→+1)  5=RT(-1→+1)
  Buttons: 0=A   1=B   2=X   3=Y   4=LB  5=RB  6=Back  7=Start  8=Xbox  9=LS  10=RS

Drive mapping (holonomic vector control):
  - Left stick  → direction vector the robot moves in (LX=strafe, LY=forward)
  - Right stick X → yaw rotation (right = turn right, left = turn left)
  - Mecanum mixing identical to updateSetpoints(FrLft, BkLft, FrRgt, BkRgt)

Arm mapping:
  - LB (button 4) held  → lift moves down (fixed rate)
  - RB (button 5) held  → lift moves up   (fixed rate)
  - LT (axis 4)  held  → gripper closes  (rate proportional to trigger depth)
  - RT (axis 5)  held  → gripper opens   (rate proportional to trigger depth)

  NOTE: If the wrong button is triggering lift, check BTN_LB/BTN_RB below.
  Run `python3 -c "import pygame; pygame.init(); pygame.joystick.init();
  j=pygame.joystick.Joystick(0); j.init();
  [print(i, j.get_button(i)) for _ in iter(pygame.event.pump,None)]"` to
  identify your actual button indices.
"""

import pygame
import serial
import threading
import time

# ── Serial ports ─────────────────────────────────────────────────────────────
DRIVE_PORT = "/dev/Drive"
ARM_PORT   = "/dev/Arm"
BAUD_RATE  = 115200

# ── Kinematic constants ───────────────────────────────────────────────────────
# MAX_FORWARD / MAX_TURN scaling lives on the ESP32 (robot_drive.h).
# The Jetson only applies a deadband and sends normalized [-1, 1] values.
DEADBAND    = 0.1   # joystick dead-zone

# ── Arm setpoint limits (radians) ────────────────────────────────────────────
# These must match MIN/MAX_LIFT_RAD and MIN/MAX_GRIP_RAD in arm_drive.h.
# TODO: measure physical end-stops and update both files.
MIN_LIFT_RAD = -3.0
MAX_LIFT_RAD =  3.0
MIN_GRIP_RAD = -2.0
MAX_GRIP_RAD =  2.0

# ── Arm max speeds ────────────────────────────────────────────────────────────
# At 50 Hz loop: step_per_loop = speed / 50
MAX_LIFT_SPEED = 1.0   # rad/s — fixed rate when bumper held
MAX_GRIP_SPEED = 1.5   # rad/s — at full trigger press (proportional to depth)

# ── Xbox One BT axis / button indices (pygame on Linux) ─────────────────────
# Verified against Xbox One controller connected via Bluetooth.
# If your controller enumerates differently, run pygame's joystick example
# to print live axis/button indices.
AXIS_LX = 0   # Left stick X  → strafe
AXIS_LY = 1   # Left stick Y  → forward (up = negative, we invert below)
AXIS_RX = 2   # Right stick X → yaw rotation
AXIS_RY = 3   # Right stick Y → unused
AXIS_LT = 4   # Left trigger  → grip close (rest=-1, full=+1)
AXIS_RT = 5   # Right trigger → grip open  (rest=-1, full=+1)
# NOTE: On some Xbox One BT configurations, LT=axis 2 and RX=axis 3.
# If drive yaw is wrong, try swapping AXIS_RX=3 and AXIS_LT=2.
BTN_LB  = 6   # Left bumper   → lift down
BTN_RB  = 7   # Right bumper  → lift up
# NOTE: If the Y face button (standard index 3) is triggering lift instead
# of the bumper, try BTN_LB=3 / BTN_RB=4.


def _trigger_depth(raw: float) -> float:
    """Convert trigger axis (rest=-1, full=+1) to depth in [0, 1]."""
    return max(0.0, (raw + 1.0) / 2.0)


def step_arm_setpoints(
    lift_sp: float, grip_sp: float,
    lt_raw: float, rt_raw: float,
    lb_held: bool, rb_held: bool,
    dt: float,
) -> tuple:
    """
    Increment arm position setpoints by one loop step and clamp to limits.

    Lift  — bumpers (digital, fixed speed):
      RB held → lift rises at MAX_LIFT_SPEED rad/s
      LB held → lift lowers at MAX_LIFT_SPEED rad/s

    Grip  — triggers (analog, proportional to depth):
      RT depth → gripper opens  at up to MAX_GRIP_SPEED rad/s
      LT depth → gripper closes at up to MAX_GRIP_SPEED rad/s

    Returns (new_lift_sp, new_grip_sp).
    """
    # Lift: fixed rate, direction from which bumper is held
    lift_delta = (int(rb_held) - int(lb_held)) * MAX_LIFT_SPEED * dt
    lift_sp = max(MIN_LIFT_RAD, min(MAX_LIFT_RAD, lift_sp + lift_delta))

    # Grip: proportional — net depth drives the rate
    grip_delta = (_trigger_depth(rt_raw) - _trigger_depth(lt_raw)) * MAX_GRIP_SPEED * dt
    grip_sp = max(MIN_GRIP_RAD, min(MAX_GRIP_RAD, grip_sp + grip_delta))

    return lift_sp, grip_sp


def compute_drive_command(lx: float, ly: float, rx: float) -> str:
    """
    Send normalized joystick intent to the drive ESP32.
    The ESP32 owns all mecanum kinematic mixing and velocity scaling.
      lx  = strafe  component, normalized [-1, 1] (right = +1)
      ly  = forward component, normalized [-1, 1] (forward = +1, up-stick inverted here)
      yaw = rotation intent,   normalized [-1, 1] (right = +1)
    Deadband is applied here so the ESP receives clean zeros.
    """
    lx_out  =  _scale(lx)
    ly_out  = -_scale(ly)   # invert: up stick (negative axis) → positive forward
    yaw_out =  _scale(rx)
    return f"lx:{lx_out:.3f};ly:{ly_out:.3f};yaw:{yaw_out:.3f};\n"

def _scale(value: float, deadband: float = DEADBAND) -> float:
    """Apply deadband and return the raw value unchanged otherwise."""
    return 0.0 if abs(value) < deadband else value


# ── IMU state shared between reader threads and main loop ────────────────────
_imu_lock = threading.Lock()
_imu_data = {
    "drive": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
              "rollRate": 0.0, "pitchRate": 0.0, "yawRate": 0.0},
    "arm":   {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
              "rollRate": 0.0, "pitchRate": 0.0, "yawRate": 0.0},
}


def _parse_imu_line(line: str) -> dict | None:
    """
    Parse "IMU:roll:X;pitch:X;yaw:X;rollRate:X;pitchRate:X;yawRate:X;"
    Returns a dict of floats or None on parse failure.
    """
    if not line.startswith("IMU:"):
        return None
    result = {}
    try:
        payload = line[4:]  # strip "IMU:"
        for token in payload.split(";"):
            if not token:
                continue
            key, _, val = token.partition(":")
            result[key] = float(val)
    except (ValueError, AttributeError):
        return None
    return result if result else None


def _serial_reader(ser: serial.Serial, label: str) -> None:
    """Background thread: continuously reads lines from one ESP serial port."""
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            imu = _parse_imu_line(line)
            if imu:
                with _imu_lock:
                    _imu_data[label].update(imu)
            elif line.startswith("DBG:"):
                pass  # ignore debug telemetry silently
            else:
                print(f"[{label}] {line}")
        except serial.SerialException:
            print(f"[{label}] Serial read error — reconnect and restart.")
            break
        except Exception as e:
            print(f"[{label}] Reader error: {e}")


def get_imu(label: str) -> dict:
    with _imu_lock:
        return dict(_imu_data[label])


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Init pygame joystick
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Connect Xbox controller and retry.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller: {joystick.get_name()}")

    # Open serial ports
    try:
        drive_ser = serial.Serial(DRIVE_PORT, BAUD_RATE, timeout=1)
        print(f"Drive serial open: {DRIVE_PORT}")
    except serial.SerialException as e:
        print(f"Could not open drive port {DRIVE_PORT}: {e}")
        return

    try:
        arm_ser = serial.Serial(ARM_PORT, BAUD_RATE, timeout=1)
        print(f"Arm serial open:   {ARM_PORT}")
    except serial.SerialException as e:
        print(f"Could not open arm port {ARM_PORT}: {e}")
        drive_ser.close()
        return

    # Allow ESPs to reboot after DTR assert
    time.sleep(2)
    drive_ser.reset_input_buffer()
    arm_ser.reset_input_buffer()

    # Start background reader threads
    threading.Thread(target=_serial_reader, args=(drive_ser, "drive"),
                     daemon=True).start()
    threading.Thread(target=_serial_reader, args=(arm_ser, "arm"),
                     daemon=True).start()

    print("Running at 50 Hz. Ctrl-C to stop.")

    loop_period = 1.0 / 50.0  # 50 Hz

    # Arm setpoint state — Jetson owns the incremental integration
    lift_sp = 0.0
    grip_sp = 0.0

    try:
        while True:
            t0 = time.monotonic()

            # Pump pygame events (required for joystick state to update)
            pygame.event.pump()

            # Read axes — safe even if controller disconnects mid-run
            lx  = joystick.get_axis(AXIS_LX)
            ly  = joystick.get_axis(AXIS_LY)
            rx  = joystick.get_axis(AXIS_RX)
            lt  = joystick.get_axis(AXIS_LT) if joystick.get_numaxes() > AXIS_LT else -1.0
            rt  = joystick.get_axis(AXIS_RT) if joystick.get_numaxes() > AXIS_RT else -1.0
            lb  = bool(joystick.get_button(BTN_LB)) if joystick.get_numbuttons() > BTN_LB else False
            rb  = bool(joystick.get_button(BTN_RB)) if joystick.get_numbuttons() > BTN_RB else False

            drive_cmd = compute_drive_command(lx, ly, rx)

            # Step arm setpoints by one dt increment then build command string
            lift_sp, grip_sp = step_arm_setpoints(
                lift_sp, grip_sp, lt, rt, lb, rb, loop_period
            )
            arm_cmd = f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n"

            drive_ser.write(drive_cmd.encode())
            arm_ser.write(arm_cmd.encode())

            # Throttle debug prints to 5 Hz to avoid flooding the console
            if int(t0 * 5) % 5 == 0:
                imu_d = get_imu("drive")
                imu_a = get_imu("arm")
                # Parse lx/ly/yaw back out of the command string for display
                parts = {t.split(":")[0]: float(t.split(":")[1])
                         for t in drive_cmd.strip().rstrip(";").split(";") if ":" in t}
                print(
                    f"lx={parts.get('lx', 0):+.2f}  "
                    f"ly={parts.get('ly', 0):+.2f}  "
                    f"yaw={parts.get('yaw', 0):+.2f}  "
                    f"| lift={lift_sp:+.3f}  grip={grip_sp:+.3f}  "
                    f"| DriveIMU yaw={imu_d['yaw']:.2f}  ArmIMU yaw={imu_a['yaw']:.2f}",
                    end="\r"
                )

            elapsed = time.monotonic() - t0
            sleep_for = loop_period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        # Zero out drive motors on exit; arm holds its last setpoint (safest for a belt-driven joint)
        drive_ser.write(b"lx:0.000;ly:0.000;yaw:0.000;\n")
        arm_ser.write(f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n".encode())
        arm_ser.write(b"lift:0.000;grip:0.000;\n")
        time.sleep(0.1)
        drive_ser.close()
        arm_ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
