import sys
from pathlib import Path
import cv2
import numpy as np

_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.config import (
    CALIB_CHESSBOARD_SIZE, CALIB_SQUARE_SIZE_M, DEPTH_CAMERA_DEVICE,
)

def main():
    # Parameters imported from Jetson/config.py
    chessboard_size = CALIB_CHESSBOARD_SIZE
    square_size     = CALIB_SQUARE_SIZE_M
    
    # Define criteria for corner sub-pixel refinement
    # (Stop after max 30 iterations or when moving by less than 0.001)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # --- Data Structures for Calibration ---
    # 3D points in real-world space for each image
    objpoints = []
    # 2D points in image plane for each image
    imgpoints = []
    
    # Prepare a single "view" of object points:
    # e.g. (0,0,0), (1,0,0), (2,0,0) ... for the entire chessboard grid.
    # This will get appended multiple times—once per collected frame.
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Open the default camera (device from config)
    # cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap = cv2.VideoCapture(DEPTH_CAMERA_DEVICE) # For MacOS or Linux, you may need to remove the cv2.CAP_DSHOW flag
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )
    
    if not cap.isOpened():
        print("Could not open webcam.")
        return

    print("Press 's' to save corners when a chessboard is detected.")
    print("Press 'c' to calibrate using all saved frames.")
    print("Press 'q' to quit without calibrating (or quit after calibration).")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            break
        
        # Convert to grayscale for corner detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Attempt to find corners
        found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        # If found, refine and draw the corners
        if found:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, chessboard_size, corners, found)

        # Show the live feed
        cv2.imshow('Calibration - Live Feed', frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            # Quit the program
            print("Exiting without further calibration.")
            break
        
        elif key == ord('s'):
            # If corners were found, store them
            if found:
                print("Chessboard corners found! Saving this frame.")
                objpoints.append(objp)
                imgpoints.append(corners)
            else:
                print("Chessboard not found in this frame. No corners saved.")
        
        elif key == ord('c'):
            # Perform calibration if we have enough frames
            if len(objpoints) < 3:
                print("Not enough frames collected. Need at least 3. Keep collecting.")
                continue
            
            print("Calibrating... please wait.")
            # Perform camera calibration
            ret_calib, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints,
                imgpoints,
                gray.shape[::-1],  # (width, height)
                None,
                None
            )
            
            if ret_calib:
                print("\n--- Calibration Successful ---")
                print("Camera Matrix:\n", camera_matrix)
                print("Distortion Coeffs:\n", dist_coeffs.ravel())
                print("--------------------------------\n")

                # Optionally, save to a file for later use
                np.savez(
                    "camera_calibration_live.npz",
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    rvecs=rvecs,
                    tvecs=tvecs
                )
                print("Calibration saved as camera_calibration_live.npz.\n")
            else:
                print("Calibration was not successful. Try collecting more data.")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
