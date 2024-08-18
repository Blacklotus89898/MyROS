import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize the RealSense pipeline and configuration
pipe = rs.pipeline()
cfg = rs.config()

# List available devices
devices = rs.context().query_devices()
if not devices:
    raise RuntimeError("No RealSense devices detected")

# Print available devices for debugging
for device in devices:
    print(f"Found device: {device.get_info(rs.camera_info.name)}")

# Configure the depth stream with lower resolution
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Lower resolution and frame rate for efficiency

# Start streaming
try:
    pipe.start(cfg)
except RuntimeError as e:
    print(f"Failed to start pipeline: {e}")
    exit(1)

try:
    while True:
        # Wait for a frame
        frames = pipe.wait_for_frames(timeout_ms=10000)
        depth_frame = frames.get_depth_frame()

        # Check if the depth frame is available
        if not depth_frame:
            print("Depth frame not available")
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert depth image to a color map with scaling
        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Display the depth image
        cv2.imshow('Depth Image', depth_cm)

        # Check for 'q' key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming and clean up resources
    pipe.stop()
    cv2.destroyAllWindows()
