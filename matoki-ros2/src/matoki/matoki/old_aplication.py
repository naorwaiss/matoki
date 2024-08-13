import sys
import av
import rclpy
import math
import threading
from rclpy.node import Node
from sensor_msgs.msg import Joy
import screeninfo
from mavros_msgs.msg import OverrideRCIn
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen




#mission at this code:
#this code run without dilay...
#add somthing to use fullscrin
#add the widgert


#
# class JoystickWidget(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.x = 0
#         self.y = 0
#         self.initUI()
#
#     def initUI(self):
#         self.setMinimumSize(150, 150)
#
#     @pyqtSlot(int, int)
#     def update_position(self, x, y):
#         max_movement = 75  # 100 (base radius) - 25 (handle radius)
#         distance = math.sqrt(x ** 2 + y ** 2)
#         if distance > max_movement:
#             scaling_factor = max_movement / distance
#             self.x = int(x * scaling_factor)
#             self.y = int(y * scaling_factor)
#         else:
#             self.x = int(x)
#             self.y = int(y)
#         self.update()
#
#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.Antialiasing)
#         self.draw_joystick_base(painter)
#         self.draw_joystick_handle(painter)
#
#     def draw_joystick_base(self, painter):
#         base_color = QColor(50, 50, 50)
#         painter.setBrush(QBrush(base_color))
#         radius = 100
#         center_x = self.width() // 2
#         center_y = self.height() // 2
#         painter.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)
#
#     def draw_joystick_handle(self, painter):
#         handle_color = QColor(255, 0, 0)
#         painter.setBrush(QBrush(handle_color))
#         painter.setPen(QPen(Qt.black, 2))
#         radius = 25
#         center_x = self.width() // 2 + self.x
#         center_y = self.height() // 2 + self.y
#         painter.drawEllipse(int(center_x - radius), int(center_y - radius), int(radius * 2), int(radius * 2))
#
#



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
        self.subscription = self.create_subscription(OverrideRCIn, '/mavros/rc/override', self.rc_in_callback, 10)
        self.channels = [0, 0, 0, 0]  # Initialize the channels list with zeros

    def rc_in_callback(self, msg):
        channels = msg.channels
        if len(channels) >= 4:
            self.channels[0] = msg.channels[0]
            self.channels[1] = msg.channels[1]
            self.channels[2] = msg.channels[2]
            self.channels[3] = msg.channels[3]


class StreamViewer(QWidget):
    def __init__(self, stream_urls):
        super().__init__()
        screen = screeninfo.get_monitors()[0]
        self.screen_size = (screen.width, screen.height)
        self.setFixedSize(*self.screen_size)
        self.stream_urls = stream_urls
        self.current_stream_index = 0
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label, alignment=Qt.AlignCenter)
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


def spin_node(node):
    rclpy.spin(node)
    node.destroy_node()


def main():
    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Stream URLs
    stream_urls = ['rtsp://admin:password1@192.168.1.64:554/Streaming/Channels/101/', 'http://127.0.0.1:5050/?action=stream']
    for url in stream_urls:
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
    rc_in_subscriber = RCInSubscriber(main_window.viewer)

    # Create and start threads for each subscriber
    joy_thread = threading.Thread(target=spin_node, args=(joy_subscriber,))
    rc_in_thread = threading.Thread(target=spin_node, args=(rc_in_subscriber,))
    joy_thread.start()
    rc_in_thread.start()

    # Start Qt main loop
    sys.exit(app.exec_())

    # Cleanup
    rclpy.shutdown()
    joy_thread.join()
    rc_in_thread.join()


if __name__ == "__main__":
    main()