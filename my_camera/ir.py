import cv2
import numpy as np


def list_available_cameras():
    available_cameras = []
    for i in range(10):  # Arbitrarily checking the first 10 device indexes
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera found at index {i}")
            available_cameras.append(i)
            cap.release()
        else:
            print(f"No camera at index {i}")
    return available_cameras


def main():
    # # Open the video capture (0 for default camera, or provide a specific device index)
    # cap = cv2.VideoCapture(4)  # Change the index or path as needed

    # if not cap.isOpened():
    #     print("Error: Could not open video capture.")
    #     return

    # # Create a window
    # cv2.namedWindow("IR Camera Feed")

    # List available cameras
    available_cameras = list_available_cameras()

    if not available_cameras:
        print("No available cameras found.")
        return

    # Open the first available camera (or change index as needed)
    cap = cv2.VideoCapture(available_cameras[2])  # Use the first available camera
    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    # Create a window
    cv2.namedWindow("IR Camera Feed")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Process the frame if needed
        # For IR cameras, frames might be in grayscale
        # Convert to a grayscale image (if not already)
        # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Optionally, apply some processing specific to IR images
        # For example, increase contrast
        # contrast_frame = cv2.convertScaleAbs(gray_frame, alpha=1.5, beta=0)

        # Display the resulting frame
        cv2.imshow("IR Camera Feed", frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
