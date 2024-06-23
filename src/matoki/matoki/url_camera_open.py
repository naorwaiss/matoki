import cv2

def main():
    # URL of the MJPG stream
    stream_url = 'http://192.168.1.101:5050/?action=stream'

    # Open the video stream
    cap = cv2.VideoCapture(stream_url)

    # Check if the stream is opened successfully
    if not cap.isOpened():
        print("Error: Could not open stream.")
        return

    # Create a named window
    cv2.namedWindow("MJPG Stream", cv2.WINDOW_NORMAL)

    # Set the window to fullscreen
    cv2.setWindowProperty("MJPG Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Loop to continuously capture and display frames from the stream
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame is empty
        if not ret:
            print("Error: Unable to capture frame.")
            break

        # Display the frame
        cv2.imshow('MJPG Stream', frame)

        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to quit
            break

    # Release the video stream and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
