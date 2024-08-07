import sys
import av
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
import screeninfo
from mavros_msgs.msg import OverrideRCIn
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot
from rclpy.executors import MultiThreadedExecutor
import math
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen


class JoystickWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.x = 0
        self.y = 0
        self.initUI()

    def initUI(self):
        self.setMinimumSize(150, 150)

    @pyqtSlot(int, int)
    def update_position(self, x, y):
        max_movement = 75  # 100 (base radius) - 25 (handle radius)
        distance = math.sqrt(x ** 2 + y ** 2)
        if distance > max_movement:
            scaling_factor = max_movement / distance
            self.x = int(x * scaling_factor)
            self.y = int(y * scaling_factor)
        else:
            self.x = int(x)
            self.y = int(y)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        self.draw_joystick_base(painter)
        self.draw_joystick_handle(painter)

    def draw_joystick_base(self, painter):
        base_color = QColor(50, 50, 50)
        painter.setBrush(QBrush(base_color))
        radius = 100
        center_x = self.width() // 2
        center_y = self.height() // 2
        painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)

    def draw_joystick_handle(self, painter):
        handle_color = QColor(255, 0, 0)
        painter.setBrush(QBrush(handle_color))
        painter.setPen(QPen(Qt.black, 2))
        radius = 25
        center_x = self.width() // 2 + self.x
        center_y = self.height() // 2 + self.y
        painter.drawEllipse(int(center_x - radius), int(center_y - radius), int(radius * 2), int(radius * 2))


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
    def __init__(self, joystick_1, joystick_2):
        super().__init__('rc_in_subscriber')
        self.subscription = self.create_subscription(OverrideRCIn, '/mavros/rc/override', self.rc_in_callback, 10)
        self.joystick_1 = joystick_1
        self.joystick_2 = joystick_2

    def rc_in_callback(self, msg):
        channels = msg.channels
        if len(channels) >= 4:
            x1 = channels[0] - 1500
            y1 = -(channels[1] - 1500)
            x2 = channels[3] - 1500
            y2 = -(channels[2] - 1500)
            threading.Thread(target=self.joystick_1.update_position, args=(x1, y1)).start()
            threading.Thread(target=self.joystick_2.update_position, args=(x2, y2)).start()


class StreamViewer(QWidget):
    def __init__(self, stream_urls):
        super().__init__()
        screen = screeninfo.get_monitors()[0]
        self.screen_size = (screen.width, screen.height)
        self.setFixedSize(*self.screen_size)
        self.stream_urls = stream_urls
        self.current_stream_index = 0
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.joystick_1 = JoystickWidget()
        self.joystick_2 = JoystickWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.label, alignment=Qt.AlignTop | Qt.AlignLeft)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self.joystick_1.setParent(self)
        self.joystick_2.setParent(self)
        self.joystick_1.move((self.screen_size[0] - self.joystick_1.width()) // 2 - 100, self.screen_size[1] - 150)  # Position joystick 1 centered horizontally
        self.joystick_2.move((self.screen_size[0] - self.joystick_2.width()) // 2 + 100, self.screen_size[1] - 150)  # Position joystick 2 centered horizontally

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

    # Create Joy Subscriber and RCIn Subscriber
    joy_subscriber = JoySubscriber(main_window.viewer.switch_stream)
    rc_in_subscriber = RCInSubscriber(main_window.viewer.joystick_1, main_window.viewer.joystick_2)

    # Create a MultiThreadedExecutor and add both nodes to it
    executor = MultiThreadedExecutor()
    executor.add_node(joy_subscriber)
    executor.add_node(rc_in_subscriber)

    # Start ROS2 spinning in a separate thread
    ros_thread = threading.Thread(target=executor.spin)
    ros_thread.start()

    # Start Qt main loop
    sys.exit(app.exec_())

    # Cleanup
    rclpy.shutdown()
    executor.shutdown()
    ros_thread.join()


if __name__ == "__main__":
    main()
