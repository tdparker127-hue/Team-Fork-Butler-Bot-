"""
robot_controller.py
Jetson Nano — main control loop

Receives Xbox controller input over Bluetooth (via pygame) and sends
command strings over USB serial to two ESP32s:

  /dev/Drive  →  "fl:X;bl:X;fr:X;br:X;\n"   (wheel velocity setpoints, rad/s)
  /dev/Arm    →  "lift:X;grip:Y;\n"           (arm rate commands, rad/s)

Receives IMU telemetry back from both ESPs (lines starting with "IMU:").
All higher-level kinematics live here; the ESPs are pure actuator nodes.

Kinematic model mirrors robot_motion_control.cpp / followTrajectory():
  - Left stick Y  → forward  (±MAX_FORWARD rad/s)
  - Left stick X  → turn     (±MAX_TURN rad/s)
  - Right stick X → strafe   (±MAX_FORWARD rad/s)
  - Mecanum wheel mixing identical to updateSetpoints() in robot_drive.cpp

Arm mapping:
  - Left trigger  (axis 2, 0→1) → positive lift rate
  - Right trigger (axis 5, 0→1) → negative lift rate
  - Right bumper  (button 5)    → positive grip rate (open)
  - Left bumper   (button 4)    → negative grip rate (close)
"""

import pygame
import serial
import threading
import time

# ── Serial ports ─────────────────────────────────────────────────────────────
DRIVE_PORT = "/dev/Drive"
ARM_PORT   = "/dev/Arm"
BAUD_RATE  = 115200

# ── Kinematic constants (match robot_drive.h) ─────────────────────────────────
MAX_FORWARD = 8.0   # rad/s
MAX_TURN    = 5.0   # rad/s
DEADBAND    = 0.1   # joystick dead-zone (matches abs() < 0.1 in firmware)

# ── Arm rate constants ────────────────────────────────────────────────────────
MAX_LIFT_RATE = 1.5  # rad/s — adjust to taste
MAX_GRIP_RATE = 1.0  # rad/s

# ── Xbox axis / button indices (pygame, standard mapping) ────────────────────
AXIS_LX        = 0   # left stick X  (strafe will use this... wait, it's turn)
AXIS_LY        = 1   # left stick Y  (forward)
AXIS_RX        = 2   # right stick X (strafe — xbox mapping: RX is axis 2? check below)
AXIS_RY        = 3   # right stick Y (unused for drive)
AXIS_LT        = 4   # left trigger  (lift up)
AXIS_RT        = 5   # right trigger (lift down)
BTN_LB         = 4   # left bumper   (grip close)
BTN_RB         = 5   # right bumper  (grip open)

# Note: pygame maps Xbox One BT axes as:
#   0=LX  1=LY  2=RX  3=RY  4=LT(-1→1)  5=RT(-1→1)
# Triggers report -1 at rest, +1 at full press → scale to [0,1] below.


def _scale(value: float, deadband: float = DEADBAND) -> float:
    """Apply deadband and return the raw value unchanged otherwise."""
    return 0.0 if abs(value) < deadband else value


def _trigger_to_rate(raw: float, max_rate: float) -> float:
    """Convert trigger axis (-1 rest, +1 full) to a 0→max_rate value."""
    return max(0.0, (raw + 1.0) / 2.0) * max_rate


def compute_drive_command(lx: float, ly: float, rx: float) -> str:
    """
    Mecanum kinematic mixing (mirrors followTrajectory / updateSetpoints).
    lx = left stick X (turn), ly = left stick Y (forward), rx = right stick X (strafe)
    Returns a formatted drive command string.
    """
    forward = _scale(ly) * (-MAX_FORWARD)   # invert Y so up = positive forward
    turn    = _scale(lx) * MAX_TURN
    strafe  = _scale(rx) * MAX_FORWARD

    # Motor sign convention matches updateSetpoints(FrLft, BkLft, FrRgt, BkRgt)
    fl =  turn - forward + strafe
    bl =  forward + turn + strafe
    fr =  forward - strafe + turn
    br =  turn - strafe - forward

    return f"fl:{fl:.3f};bl:{bl:.3f};fr:{fr:.3f};br:{br:.3f};\n"


def compute_arm_command(lt_raw: float, rt_raw: float,
                         lb_held: bool, rb_held: bool) -> str:
    """
    Build arm rate command.
    lift_rate > 0 → arm rises, lift_rate < 0 → arm lowers.
    grip_rate > 0 → gripper opens, grip_rate < 0 → gripper closes.
    """
    lift_rate = _trigger_to_rate(lt_raw, MAX_LIFT_RATE) \
              - _trigger_to_rate(rt_raw, MAX_LIFT_RATE)

    grip_rate = 0.0
    if rb_held:
        grip_rate =  MAX_GRIP_RATE
    elif lb_held:
        grip_rate = -MAX_GRIP_RATE

    return f"lift:{lift_rate:.3f};grip:{grip_rate:.3f};\n"


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
            arm_cmd   = compute_arm_command(lt, rt, lb, rb)

            drive_ser.write(drive_cmd.encode())
            arm_ser.write(arm_cmd.encode())

            # Throttle debug prints to ~5 Hz to avoid flooding the console
            if int(t0 * 5) % 5 == 0:
                imu_d = get_imu("drive")
                imu_a = get_imu("arm")
                print(f"Drive: {drive_cmd.strip()}", end="   ")
                print(f"Arm: {arm_cmd.strip()}", end="   ")
                print(f"DriveIMU yaw={imu_d['yaw']:.2f}  ArmIMU yaw={imu_a['yaw']:.2f}",
                      end="\r")

            elapsed = time.monotonic() - t0
            sleep_for = loop_period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        # Zero out motors on exit
        drive_ser.write(b"fl:0.000;bl:0.000;fr:0.000;br:0.000;\n")
        arm_ser.write(b"lift:0.000;grip:0.000;\n")
        time.sleep(0.1)
        drive_ser.close()
        arm_ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
