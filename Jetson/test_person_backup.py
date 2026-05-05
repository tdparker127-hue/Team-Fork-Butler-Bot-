"""
test_person_backup.py — Drive straight back to maintain a 2-foot stand-off
from a detected person.

Run on the Jetson with the drive ESP32 connected and a camera attached:
    python3 -m Jetson.test_person_backup

Press Q in the OpenCV window (or Ctrl-C in terminal) to stop.

How it works end-to-end:
  1. Each camera frame is fed to PersonDetector (YOLOv8) which returns the
     distance to every visible person in metres.
  2. A proportional controller converts the distance error into a backward
     drive command (negative ly in the robot's serial protocol).
  3. The command is written to the drive ESP32 over USB serial each loop.

Depth camera note:
  Pass a RealSense depth frame to detector.detect() for accurate metric
  distances (proportional control).  Without depth (webcam only) dist_m is
  None and the script falls back to a coarser threat-level approach.
"""

import sys
import time

import cv2
import serial

from config import COLOR_CAMERA_DEVICE
from vision.person_detection import PersonDetector, PersonDetection, PersonThreat

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

DRIVE_PORT  = "/dev/ttyACM0"   # USB serial port for the drive ESP32
BAUD_RATE   = 115200

# Distance the robot tries to maintain from the person (2 feet in metres)
TARGET_DIST_M    = 0.6096

# Dead-zone: ignore errors smaller than this to avoid jitter at the target
DIST_TOLERANCE_M = 0.05

# Proportional gain for the distance controller.
# 1 metre of error produces KP_BACKUP worth of normalised backward speed.
# Example: person 0.3 m closer than target → error=0.3 → ly = -(0.6 * 0.3) = -0.18
KP_BACKUP = 0.6

# Hard cap on backward speed (normalised, sent straight to the ESP32).
# The ESP32 scales this by MAX_Back (currently 4 rad/s), so 0.35 ≈ 1.4 rad/s.
MAX_BACKUP_SPEED = 0.35

# Fallback speeds used when no depth camera is available and dist_m is None.
# The controller drops to a rule-based mode using PersonThreat levels instead.
SLOW_BACKUP_SPEED = 0.20   # person in SLOW zone → gentle retreat
STOP_BACKUP_SPEED = 0.35   # person in STOP zone → faster retreat


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def open_drive_serial() -> serial.Serial:
    """
    Open USB serial to the drive ESP32 and wait for it to finish rebooting.

    The ESP32 resets when the serial port opens (DTR assert), so a 2-second
    sleep is needed before the first command is sent.  Without this delay
    the ESP32 may not yet be running its main loop and will miss the command.
    """
    ser = serial.Serial(DRIVE_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    return ser


def send_drive(ser: serial.Serial, lx: float, ly: float, yaw: float) -> None:
    """
    Send one drive command to the ESP32.

    The ESP32 expects the ASCII format:
        "lx:X;ly:X;yaw:X;\n"   where each value is in [-1.0, 1.0]

    The ESP32 then scales each axis:
        ly  * MAX_FORWARD  → forward/backward wheel setpoints (rad/s)
        lx  * MAX_FORWARD  → strafe wheel setpoints
        yaw * MAX_TURN     → rotation wheel setpoints

    This script only drives straight back so lx=0 and yaw=0 are always passed.
    A negative ly means backward: the ESP32's mecanum mixing produces equal
    backward speed on all four wheels, keeping the robot tracking straight.
    """
    ser.write(f"lx:{lx:.3f};ly:{ly:.3f};yaw:{yaw:.3f};\n".encode())


def stop_motors(ser: serial.Serial) -> None:
    """
    Zero all drive setpoints.

    Called in the finally block so the motors always halt on exit, even if
    the script crashes or is interrupted mid-command.
    """
    send_drive(ser, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Distance-to-speed control law
# ---------------------------------------------------------------------------

def compute_backup_speed(dist_m: float | None, threat: PersonThreat) -> float:
    """
    Return the ly drive command needed to maintain TARGET_DIST_M from a person.

    Return value is in [-MAX_BACKUP_SPEED, 0]:
        0.0  → hold still  (person is far enough, or nobody visible)
        -0.x → drive backward at that fraction of max speed

    --- When depth is available (dist_m is a float) ---
    Uses a proportional controller:
        error = TARGET_DIST_M - dist_m
        ly    = -clamp(KP_BACKUP * error,  0,  MAX_BACKUP_SPEED)

    A positive error means the person is closer than desired, so ly is
    driven negative (backward).  The DIST_TOLERANCE_M dead-zone prevents
    the robot from continuously nudging when it is already close enough.

    --- When depth is unavailable (dist_m is None) ---
    Falls back to a stepped rule using PersonThreat:
        STOP  → retreat at STOP_BACKUP_SPEED  (person very close)
        SLOW  → retreat at SLOW_BACKUP_SPEED  (person in slow zone)
        CLEAR → stop                           (no detected threat)
    This mode will not maintain an exact 2-foot distance but keeps the robot
    from closing on the person.
    """
    if dist_m is not None:
        error = TARGET_DIST_M - dist_m          # positive when person is too close
        if error <= DIST_TOLERANCE_M:
            return 0.0                          # already at target distance
        speed = min(KP_BACKUP * error, MAX_BACKUP_SPEED)
        return -speed                           # negative = backward

    # No depth reading — use threat level as a coarse proxy for distance
    if threat == PersonThreat.STOP:
        return -STOP_BACKUP_SPEED
    if threat == PersonThreat.SLOW:
        return -SLOW_BACKUP_SPEED
    return 0.0


def pick_closest_person(detections: list[PersonDetection]) -> PersonDetection | None:
    """
    Return the person who is nearest to the robot from the detection list.

    Prefers detections that have a real depth reading (dist_m is not None)
    because those have metric accuracy.  If no depth readings exist, falls
    back to the highest-threat detection as a best-effort proxy for "closest."
    Returns None if detections is empty.
    """
    with_depth = [d for d in detections if d.dist_m is not None]
    if with_depth:
        return min(with_depth, key=lambda d: d.dist_m)
    if detections:
        return max(detections, key=lambda d: d.threat)
    return None


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    # --- Open drive serial ---------------------------------------------------
    try:
        drive_ser = open_drive_serial()
        print(f"Drive serial open on {DRIVE_PORT}")
    except serial.SerialException as e:
        print(f"ERROR: could not open drive port {DRIVE_PORT}: {e}")
        sys.exit(1)

    # --- Build detector ------------------------------------------------------
    # YOLOv8n weights are downloaded automatically on first use (~6 MB).
    # The constructor also runs a warm-up inference so the first real frame
    # is not artificially slow.
    detector = PersonDetector()
    print("PersonDetector ready.")

    # --- Open camera ---------------------------------------------------------
    # COLOR_CAMERA_DEVICE is defined in config.py (currently "/dev/video0").
    # Replace cap.read() with a RealSense pipeline to also get depth frames.
    cap = cv2.VideoCapture(COLOR_CAMERA_DEVICE)
    if not cap.isOpened():
        print(f"ERROR: could not open camera {COLOR_CAMERA_DEVICE}")
        drive_ser.close()
        sys.exit(1)

    print(f"Running — target distance {TARGET_DIST_M:.2f} m ({TARGET_DIST_M / 0.3048:.1f} ft)")
    print("Press Q to quit.")

    try:
        while True:
            ret, color_bgr = cap.read()
            if not ret:
                print("Camera read failed — exiting.")
                break

            # Run YOLO on the color frame.
            # depth_mm=None forces the bbox-area fallback; swap in a real
            # aligned depth frame from RealSense for metric distances.
            threat, detections, annotated = detector.detect(color_bgr, depth_mm=None)

            # Choose which detected person to react to
            person = pick_closest_person(detections)

            if person is not None:
                ly = compute_backup_speed(person.dist_m, person.threat)
                dist_str = f"{person.dist_m:.2f} m" if person.dist_m is not None else "no depth"
                print(f"Person | dist={dist_str:<10} threat={person.threat.name:<5} ly={ly:+.3f}", end="\r")
            else:
                # No person in frame — stop and hold position
                ly = 0.0
                print("No person detected — holding.                    ", end="\r")

            # Transmit drive command: straight back only (lx=0, yaw=0)
            send_drive(drive_ser, lx=0.0, ly=ly, yaw=0.0)

            # Show the annotated frame (bounding boxes + threat banner)
        #    cv2.imshow("Person Backup Test", annotated)
         #   if cv2.waitKey(1) & 0xFF == ord("q"):
          #      break

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Zero motors before closing — prevents the robot running away on exit
        stop_motors(drive_ser)
        cap.release()
        drive_ser.close()
       # cv2.destroyAllWindows()
        print("Motors stopped. Done.")


if __name__ == "__main__":
    main()
