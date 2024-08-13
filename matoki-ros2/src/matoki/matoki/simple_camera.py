import av
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def main():
    # Open video capture
    container = av.open('/dev/video2')  # Replace with your video device

    # Get the first stream (video stream)
    video_stream = container.streams.video[0]

    # Create a window for display
    plt.ion()
    fig, ax = plt.subplots()
    img = ax.imshow(np.zeros((video_stream.height, video_stream.width, 3), dtype=np.uint8))

    # Loop to read frames and display
    for frame in container.decode(video_stream):
        # Convert frame to numpy array
        img_array = frame.to_ndarray(format='bgr24')
        
        # Update the display
        img.set_data(img_array)
        ax.set_xlim(0, frame.width)
        ax.set_ylim(frame.height, 0)  # Invert y-axis
        plt.draw()
        
        # Pause for a short time to keep the plot interactive
        plt.pause(0.001)  # Adjust this value if needed

        # Check for user input to exit
        if plt.waitforbuttonpress(timeout=0.001):  # Very short timeout for detecting key press
            break

    plt.close()

if __name__ == "__main__":
    main()

