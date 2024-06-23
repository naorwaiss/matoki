import sys
import av
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
import screeninfo
from mavros_msgs.msg import RCIn
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class JoySubscriber(Node):
    def __init__(self, switch_callback):
        super().__init__('camera_switch_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.switch_callback = switch_callback
        self.current_stream = 0

    def joy_callback(self, msg):
        if msg.axes[6] == -1 and self.current_stream == 0:
            self.switch_callback(msg.axes[6])
            self.current_stream = 1
        elif msg.axes[6] == 1 and self.current_stream == 1:
            self.switch_callback(msg.axes[6])
            self.current_stream = 0


class RCInSubscriber(Node):
    def __init__(self, plot_widget):
        super().__init__('rc_in_subscriber')
        self.subscription = self.create_subscription(RCIn, '/mavros/rc/override', self.rc_in_callback, 10)
        self.plot_widget = plot_widget

    def rc_in_callback(self, msg):
        channels = msg.channels
        if len(channels) >= 4:
            print(f"Received RC channels: {channels[:4]}")  # Debug statement
            self.plot_widget.update_plot(channels[0], channels[1], channels[2], channels[3])


class StreamViewer(QWidget):
    def __init__(self, stream_urls):
        super().__init__()
        screen = screeninfo.get_monitors()[0]
        self.screen_size = (screen.width, screen.height)
        self.setFixedSize(*self.screen_size)
        print(self.screen_size)
        self.stream_urls = stream_urls
        self.current_stream_index = 0
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        layout = QVBoxLayout()
        layout.addWidget(self.label, alignment=Qt.AlignTop | Qt.AlignLeft)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self.input_container = av.open(self.stream_urls[self.current_stream_index])
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_frame(self):
        try:
            for frame in self.input_container.decode(video=0):
                image = frame.to_image()
                image = image.convert("RGB")
                data = image.tobytes("raw", "RGB")
                qimg = QImage(data, image.width, image.height, QImage.Format_RGB888)
                self.label.setPixmap(
                    QPixmap.fromImage(qimg).scaled(self.screen_size[0], self.screen_size[1], Qt.KeepAspectRatio,
                                                   Qt.SmoothTransformation))
                break
        except Exception as e:
            print(f"Error: {e}")

    def switch_stream(self, direction):
        self.current_stream_index = (self.current_stream_index + 1) % len(self.stream_urls)
        print(f"Switching to stream: {self.stream_urls[self.current_stream_index]}")
        self.input_container = av.open(self.stream_urls[self.current_stream_index])

    def close(self):
        if self.input_container:
            self.input_container.close()


class MainWindow(QMainWindow):
    def __init__(self, stream_urls):
        super().__init__()
        self.viewer = StreamViewer(stream_urls)
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.viewer)
        layout.setContentsMargins(0, 0, 0, 0)  # Remove margins to avoid any white outline
        self.setCentralWidget(central_widget)
        self.setWindowTitle("MJPG Stream Viewer")
        self.showFullScreen()

    def closeEvent(self, event):
        self.viewer.close()
        event.accept()


def main():
    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Stream URLs
    stream_urls = ['http://127.0.0.1:5050/?action=stream', 'http://127.0.0.1:8080/?action=stream']
    for url in stream_urls:
        print(f"Checking stream URL: {url}")
        try:
            input_container = av.open(url)
            input_container.close()
        except av.AVError as e:
            print(f"Error: Could not open stream {url}. {e}")
            sys.exit()

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
