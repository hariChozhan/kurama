# import sys
# import cv2
# from PyQt5.QtCore import Qt, QTimer
# from PyQt5.QtGui import QImage, QPixmap
# from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget


# class VideoStreamApp(QMainWindow):
#     def __init__(self):
#         super().__init__()

#         # 0 corresponds to the default camera
#         self.video_capture = cv2.VideoCapture(0)

#         self.central_widget = QWidget(self)
#         self.setCentralWidget(self.central_widget)

#         self.video_label = QLabel(self)
#         self.video_label.setAlignment(Qt.AlignCenter)

#         self.layout = QVBoxLayout(self.central_widget)
#         self.layout.addWidget(self.video_label)

#         self.timer = QTimer(self)
#         self.timer.timeout.connect(self.update_frame)
#         self.timer.start(30)  # Set the timer interval in milliseconds

#     def update_frame(self):
#         ret, frame = self.video_capture.read()
#         if ret:
#             frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             h, w, ch = frame.shape
#             bytes_per_line = ch * w
#             q_image = QImage(frame.data, w, h, bytes_per_line,
#                              QImage.Format_RGB888)
#             pixmap = QPixmap.fromImage(q_image)
#             self.video_label.setPixmap(pixmap)

#     def closeEvent(self, event):
#         self.video_capture.release()
#         event.accept()


# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     window = VideoStreamApp()
#     window.setGeometry(100, 100, 800, 600)
#     window.setWindowTitle('Video Stream App')
#     window.show()
#     sys.exit(app.exec_())


import sys
import cv2
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QGraphicsScene, QGraphicsView


class VideoStreamApp(QMainWindow):
    def __init__(self):
        super().__init__()

        # 0 corresponds to the default camera
        self.video_capture = cv2.VideoCapture(0)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.scene = QGraphicsScene(self)
        self.video_view = QGraphicsView(self.scene)
        self.video_view.setAlignment(Qt.AlignCenter)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addWidget(self.video_view)

        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)

        self.layout.addWidget(self.video_label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Set the timer interval in milliseconds

        self.bounding_box_start = None
        self.bounding_box_end = None
        self.drawing_box = False

    def update_frame(self):
        ret, frame = self.video_capture.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_image = QImage(frame.data, w, h, bytes_per_line,
                             QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap)

            if self.drawing_box:
                self.draw_temporary_box()

    def draw_temporary_box(self):
        if self.bounding_box_start and self.bounding_box_end:
            painter = QPainter(self.video_label.pixmap())
            painter.setPen(QPen(Qt.red, 2, Qt.SolidLine))
            rect = QRectF(self.bounding_box_start, self.bounding_box_end)
            painter.drawRect(rect)
            painter.end()
            self.video_label.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.bounding_box_start = event.pos()
            self.bounding_box_end = event.pos()
            self.drawing_box = True

    def mouseMoveEvent(self, event):
        if self.drawing_box:
            self.bounding_box_end = event.pos()
            self.draw_temporary_box()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing_box:
            self.bounding_box_end = event.pos()
            self.drawing_box = False

            # Convert coordinates to video frame coordinates
            bounding_box_start = self.video_label.mapToParent(
                self.bounding_box_start)
            bounding_box_end = self.video_label.mapToParent(
                self.bounding_box_end)

            # Do something with the coordinates (e.g., print them)
            print(
                f"Bounding Box Coordinates: {bounding_box_start}, {bounding_box_end}")

            # Reset bounding box points
            self.bounding_box_start = None
            self.bounding_box_end = None


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = VideoStreamApp()
    window.setGeometry(100, 100, 800, 600)
    window.setWindowTitle('Video Stream App')
    window.show()
    sys.exit(app.exec_())
