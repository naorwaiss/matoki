import sys
import cv2
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import time


#this code is run and open 2 camera together
#resulotion -- mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 1280x720 -f 30" -o "output_http.so -p 8080 -w /usr/local/share/mjpg-streamer/www"

class JoySubscriber(Node):
    def __init__(self, switch_callback):
        super().__init__('camera_switch_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.switch_callback = switch_callback

    def joy_callback(self, msg):
        if len(msg.axes) > 6:
            print(f"Joystick input detected: {msg.axes[6]}")
            if msg.axes[6] == 1 or msg.axes[6] == -1:
                self.switch_callback(msg.axes[6])
        else:
            print("Error: Joystick message does not contain enough axes.")


class StreamViewer(QWidget):
    def __init__(self, stream_urls):
        super().__init__()
        self.stream_urls = stream_urls
        self.caps = [cv2.VideoCapture(url) for url in stream_urls]
        self.current_stream_index = 0
        self.lock = threading.Lock()
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        for i, cap in enumerate(self.caps):
            if not cap.isOpened():
                print(f"Error: Could not open stream {stream_urls[i]}")
                sys.exit()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        with self.lock:
            cap = self.caps[self.current_stream_index]
            if not cap.isOpened():
                print("Error: Stream is not opened.")
                return

            ret, frame = cap.read()
            if ret:
                height, width, channel = frame.shape
                bytes_per_line = 3 * width
                qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                self.label.setPixmap(QPixmap.fromImage(qimg))
            else:
                print("Error: Unable to capture frame.")

    def switch_stream(self, direction):
        with self.lock:
            if direction == 1:
                self.current_stream_index = 0  # Switch to the first stream
            elif direction == -1:
                self.current_stream_index = 1  # Switch to the second stream

            print(f"Switching to stream: {self.stream_urls[self.current_stream_index]}")

    def close(self):
        with self.lock:
            for cap in self.caps:
                if cap.isOpened():
                    cap.release()


class MainWindow(QMainWindow):
    def __init__(self, stream_urls):
        super().__init__()
        self.viewer = StreamViewer(stream_urls)
        self.setCentralWidget(self.viewer)
        self.setWindowTitle("MJPG Stream Viewer")
        self.showMaximized()

    def closeEvent(self, event):
        self.viewer.close()
        event.accept()


def main():
    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Stream URLs
    stream_urls = ['http://192.168.1.151:5050/?action=stream', 'http://192.168.1.151:8080/?action=stream']
    for url in stream_urls:
        print(f"Checking stream URL: {url}")
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            print(f"Error: Could not open stream {url}")
            sys.exit()
        cap.release()

    # Initialize Qt application
    app = QApplication(sys.argv)
    main_window = MainWindow(stream_urls)

    # Create Joy Subscriber
    joy_subscriber = JoySubscriber(main_window.viewer.switch_stream)

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(joy_subscriber,))
    ros_thread.start()

    # Start Qt main loop
    sys.exit(app.exec_())

    # Cleanup
    rclpy.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
