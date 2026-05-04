import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables
depth_frame_data = None
depth_scale = None
click_point = None
click_depth = None
view_mode = "depth"  # "depth" or "rgb"


def mouse_callback(event, x, y, flags, param):
    """Handle mouse click events - always measures depth regardless of view mode."""
    global click_point, click_depth, depth_frame_data

    if event == cv2.EVENT_LBUTTONDOWN:
        if depth_frame_data is not None:
            click_point = (x, y)
            # Clamp to depth frame bounds
            dy = min(y, depth_frame_data.shape[0] - 1)
            dx = min(x, depth_frame_data.shape[1] - 1)
            depth_value = depth_frame_data[dy, dx]
            click_depth = depth_value * depth_scale
            print(
                f"[{view_mode.upper()}] Pixel ({x}, {y}) - Depth: {click_depth:.4f}m ({click_depth * 1000:.1f}mm)"
            )


def draw_overlay(image, frame_label):
    """Draw click info and HUD overlay on the image."""
    h, w = image.shape[:2]

    # Draw click point and depth info
    if click_point is not None and click_depth is not None:
        cx, cy = click_point

        # Only draw if click point is within current image bounds
        if 0 <= cx < w and 0 <= cy < h:
            # Draw crosshair
            cv2.drawMarker(image, click_point, (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.circle(image, click_point, 5, (0, 255, 0), -1)

            # Depth text
            if click_depth > 0:
                text = f"Depth: {click_depth:.3f}m ({click_depth * 1000:.1f}mm)"
            else:
                text = "Depth: No data (0)"

            # Position text
            text_y = cy - 15 if cy > 30 else cy + 30
            text_x = max(10, min(cx - 80, w - 280))

            # Text background
            (text_w, text_h), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            cv2.rectangle(
                image,
                (text_x - 5, text_y - text_h - 5),
                (text_x + text_w + 5, text_y + 5),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                image,
                text,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            # Pixel coordinates
            coord_text = f"Pixel: ({cx}, {cy})"
            cv2.putText(
                image,
                coord_text,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

    # View mode indicator (top right)
    mode_text = f"View: {frame_label}"
    (tw, th), _ = cv2.getTextSize(mode_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(image, (w - tw - 15, 5), (w - 5, th + 15), (0, 0, 0), -1)
    cv2.putText(
        image,
        mode_text,
        (w - tw - 10, th + 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    # Instructions (bottom)
    instructions = "Click: measure depth | T: toggle view | Q/ESC: quit"
    cv2.putText(
        image,
        instructions,
        (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (200, 200, 200),
        1,
    )

    return image


def main():
    global depth_frame_data, depth_scale, view_mode, click_point, click_depth

    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable both depth and color streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the pipeline
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"Error: Could not start RealSense pipeline: {e}")
        print("Make sure a RealSense camera is connected.")
        return

    # Get depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale: {depth_scale} meters per unit")
    print("-" * 50)
    print("Controls:")
    print("  Click     - Measure depth at pixel")
    print("  T         - Toggle between Depth and RGB view")
    print("  Q / ESC   - Quit")
    print("-" * 50)

    # Align depth to color so clicks match between views
    align = rs.align(rs.stream.color)

    # Create window
    window_name = "RealSense Viewer"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, mouse_callback)

    try:
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()

            # Align depth to color
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert to numpy arrays
            depth_frame_data = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Choose display based on view mode
            if view_mode == "depth":
                depth_display = cv2.convertScaleAbs(depth_frame_data, alpha=0.03)
                display_image = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
                display_image = draw_overlay(display_image, "DEPTH")
            else:
                display_image = color_image.copy()
                display_image = draw_overlay(display_image, "RGB")

            # Show image
            cv2.imshow(window_name, display_image)

            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:  # Quit
                break
            elif key == ord("t") or key == ord("T"):  # Toggle view
                view_mode = "rgb" if view_mode == "depth" else "depth"
                click_point = None  # Reset click on toggle
                click_depth = None
                print(f"Switched to {view_mode.upper()} view")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Pipeline stopped. Goodbye!")


if __name__ == "__main__":
    main()
