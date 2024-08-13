import sys
import av
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

class RTSPViewer(QWidget):
    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url
        self.initUI()
        self.initStream()

    def initUI(self):
        self.setWindowTitle("RTSP Stream Viewer")
        self.image_label = QLabel(self)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)

    def initStream(self):
        self.container = av.open(self.rtsp_url)
        self.stream = self.container.streams.video[0]
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30 ms interval for approximately 33 FPS

    def update_frame(self):
        for packet in self.container.demux(self.stream):
            for frame in packet.decode():
                img = frame.to_image()
                qt_img = self.convert_image(img)
                self.image_label.setPixmap(qt_img)
                return

    def convert_image(self, img):
        img_data = img.tobytes()
        w, h = img.size
        q_img = QImage(img_data, w, h, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        return pixmap

if __name__ == "__main__":
    rtsp_url = r"rtsp://admin:password1@192.168.1.64:554/Streaming/Channels/101/"
    app = QApplication(sys.argv)
    viewer = RTSPViewer(rtsp_url)
    viewer.show()
    sys.exit(app.exec_())
