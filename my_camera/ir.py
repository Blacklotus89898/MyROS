
import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Initialize the RealSense pipeline and configuration
pipe = rs.pipeline()
cfg = rs.config()

# Configure the depth stream
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipe.start(cfg)

try:
    while True:
        # Wait for a frame
        frames = pipe.wait_for_frames(timeout_ms=3000)
        depth_frame = frames.get_depth_frame()
        
        # Check if the depth frame is available
        if not depth_frame:
            print("Depth frame not available")
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert depth image to a color map
        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        # Display the depth image
        cv2.imshow('Depth Image', depth_cm)
        
        # Check for 'q' key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # Optional: Adjust sleep time as needed
        # time.sleep(0.1)

finally:
    # Stop streaming
    pipe.stop()
    # Close any OpenCV windows
    cv2.destroyAllWindows()
