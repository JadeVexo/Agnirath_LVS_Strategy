# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QLabel
# from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap, QFont, QTransform
# from PyQt5.QtCore import Qt, QRectF, QTimer, QSize


# class SteeringDisplay(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Steering Display")
#         self.setGeometry(100, 100, 800, 480)

#         self.speed = 0
#         self.max_speed = 150
#         self.brake_pressed = False

#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_speed)
#         self.timer.start(100)  # Update speed every 100 milliseconds

#         self.speed_label = QLabel(self)
#         self.speed_label.setGeometry(350, 250, 100, 50)
#         self.speed_label.setAlignment(Qt.AlignCenter)
#         self.speed_label.setStyleSheet("font-family: 'Good Times'; font-size: 40px; font-weight: 400; "
#                                         "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
#                                         "background-color: #1E1E1E")

#         self.image_above = QLabel(self)
#         self.image_above.setGeometry(300, 100, 200, 100)
#         self.image_above.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/ellipse_2.png").scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio))

#         self.image_below = QLabel(self)
#         self.image_below.setGeometry(300, 300, 200, 100)
#         self.image_below.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/ellipse_3.png").scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio))

#         transform = QTransform().rotate(180)
#         self.image_below.setPixmap(self.image_below.pixmap().transformed(transform))


#     def paintEvent(self, event):
#         painter = QPainter(self)
#         painter.setRenderHint(QPainter.Antialiasing)

#         # Set the background color
#         self.set_background_color(painter)

#         # Draw speedometer arc
#         self.draw_speedometer(painter)

#     def set_background_color(self, painter):
#         painter.fillRect(self.rect(), QColor("#1E1E1E"))

#     def draw_speedometer(self, painter):
#         arc_radius = min(self.width(), self.height()) * 0.4
#         arc_center = self.rect().center()
#         arc_rect = QRectF(arc_center.x() - arc_radius, arc_center.y() - arc_radius, 2 * arc_radius, 2 * arc_radius)

#         # Draw the base arc
#         base_color = QColor(200, 200, 200)
#         painter.setPen(QPen(base_color, 30))
#         painter.drawArc(arc_rect, -130 * 16, -280 * 16)  #-50 * 16, 280 * 16

#         # Draw the dynamic arc
#         if self.brake_pressed:
#             arc_color = QColor(255, 0, 0)
#         else:
#             arc_color = QColor(0, 255, 0)

#         painter.setPen(QPen(arc_color, 30))
#         start_angle = -130 * 16
#         span_angle = int(-280 * (self.speed / self.max_speed))

#         painter.drawArc(arc_rect, start_angle, span_angle * 16)

#     def update_speed(self):
#         if self.speed == 150:
#             self.speed = 0
#         else:
#             self.speed += 1

#         self.speed_label.setText(str(self.speed))

#         self.update()

#     def keyPressEvent(self, event):
#         if event.key() == Qt.Key_S:
#             self.brake_pressed = True

#     def keyReleaseEvent(self, event):
#         if event.key() == Qt.Key_S:
#             self.brake_pressed = False


# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     steering_display = SteeringDisplay()
#     steering_display.show()
#     sys.exit(app.exec_())
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap, QFont, QTransform, QImage, QPolygon
from PyQt5.QtCore import Qt, QRectF, QTimer, QSize, QThread, pyqtSignal, QPoint
import socket

class SpeedThread(QThread):
    speed_received = pyqtSignal(int)

    def run(self):
        host = 'localhost'  # IP address of the C++ server
        port = 8888  # port number used by the C++ server

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((host, port))
                while True:
                    speed_data = client_socket.recv(1024)
                    if not speed_data:
                        break
                    speed = int(speed_data.decode().strip())
                    self.speed_received.emit(speed)

        except Exception as e:
            print("Error occurred:", str(e))


class SteeringDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Steering Display")
        self.setGeometry(100, 100, 800, 480)

        self.speed = 0
        self.max_speed = 150
        self.brake_pressed = False

        self.rpm_value = 0
        self.rpm_max = 1700
        self.rpm_angle = 0
        self.rpm_needle_length = 75

        self.regen_value = 0
        self.regen_max = 100
        self.regen_angle = 0
        self.regen_needle_length = 75

        self.speed_label = QLabel(self)
        self.speed_label.setGeometry(350, 250, 100, 50)
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.speed_label.setStyleSheet("font-family: 'Good Times'; font-size: 40px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FF4D00; "
                                        "background-color: #1E1E1E")
        
        self.dist_label = QLabel(self)
        self.dist_label.setGeometry(40, 65, 100, 50)
        self.dist_label.setAlignment(Qt.AlignCenter)
        self.dist_label.setStyleSheet("font-family: 'Good Times'; font-size: 15px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        "background-color: #1E1E1E")
        self.dist_label.setText("DST.RMN\n3000")

        self.regen_label = QLabel(self)
        self.regen_label.setGeometry(640, 400, 100, 50)
        self.regen_label.setAlignment(Qt.AlignCenter)
        self.regen_label.setStyleSheet("font-family: 'Good Times'; font-size: 12px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        )
        self.regen_label.setText("100%")

        self.rpm_label = QLabel(self)
        self.rpm_label.setGeometry(65, 400, 100, 50)
        self.rpm_label.setAlignment(Qt.AlignCenter)
        self.rpm_label.setStyleSheet("font-family: 'Good Times'; font-size: 12px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        )
        self.rpm_label.setText("1000")

        self.image_above = QLabel(self)
        self.image_above.setGeometry(300, 100, 200, 100)
        self.image_above.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/ellipse_2.png").scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio))

        self.image_below = QLabel(self)
        self.image_below.setGeometry(300, 230, 200, 100)
        self.image_below.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/ellipse_3.png").scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio))

        self.disellipse = QLabel(self)
        self.disellipse.setGeometry(40, 40, 200, 100)
        self.disellipse.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/ellipse_7.png").scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio))

        self.rpmellipse = QLabel(self)
        self.rpmellipse.setGeometry(30, 290, 350, 175)
        self.rpmellipse.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/rpm.png").scaled(350, 175, Qt.AspectRatioMode.KeepAspectRatio))

        self.regenellipse = QLabel(self)
        self.regenellipse.setGeometry(600, 290, 350, 175)
        self.regenellipse.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/Regen.png").scaled(350, 175, Qt.AspectRatioMode.KeepAspectRatio))

        self.battery = QLabel(self)
        self.battery.setGeometry(630, 50, 250, 125)
        self.battery.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery.png").scaled(250, 125, Qt.AspectRatioMode.KeepAspectRatio))

        self.rectangle = QLabel(self)
        self.rectangle.setGeometry(310, 370, 200, 100)
        self.rectangle.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/Rectangle.png").scaled(175, 175, Qt.AspectRatioMode.KeepAspectRatio))

        transform = QTransform().rotate(180)
        self.image_below.setPixmap(self.image_below.pixmap().transformed(transform))

        self.speed_thread = SpeedThread()
        self.speed_thread.speed_received.connect(self.update_speed)
        self.speed_thread.start()

        self.left_signal_active = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/left_active.png")
        self.left_signal_inactive = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/left_inactive.png")
        self.right_signal_active = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/union1_state_property_1_Variant2.png")
        self.right_signal_inactive = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/union1_state_property_1_Default.png")
        self.horn_active = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/Horn_active.png")
        self.horn_inactive = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/Horn_Inactive.png")
        self.radio_active = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/radio_active.png")
        self.radio_inactive = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/radio_inactive.png")
        self.cruise_active = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/cruise_active.png")
        self.cruise_inactive = QImage("/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/cruise_inactive.png")

        self.left_signal_on = False
        self.right_signal_on = False
        self.horn_on = False
        self.radio_on = False
        self.cruise_on = False
        
        self.key_press_mapping = {
            Qt.Key_1: self.toggle_left_signal,
            Qt.Key_2: self.toggle_right_signal,
            Qt.Key_3: self.toggle_horn,
            Qt.Key_4: self.toggle_radio,
            Qt.Key_5: self.toggle_cruise
        }

    def toggle_left_signal(self):
        self.left_signal_on = not self.left_signal_on
        self.update()

    def toggle_right_signal(self):
        self.right_signal_on = not self.right_signal_on
        self.update()

    def toggle_horn(self):
        self.horn_on = not self.horn_on
        self.update()

    def toggle_radio(self):
        self.radio_on = not self.radio_on
        self.update()

    def toggle_cruise(self):
        self.cruise_on = not self.cruise_on
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Set the background color
        self.set_background_color(painter)

        # Draw speedometer arc
        self.draw_speedometer(painter)

        # Draw indicators
        self.draw_indicators(painter)

        self.draw_needlerpm(painter)

        self.draw_needleregen(painter)

    def set_background_color(self, painter):
        painter.fillRect(self.rect(), QColor("#1E1E1E"))

    def draw_speedometer(self, painter):
        arc_radius = min(self.width(), self.height()) * 0.39
        arc_center = self.rect().center()
        arc_rect = QRectF(arc_center.x() - arc_radius, arc_center.y() - arc_radius, 2 * arc_radius, 2 * arc_radius)

        # Draw the base arc
        base_color = QColor(200, 200, 200)
        painter.setPen(QPen(base_color, 30))
        painter.drawArc(arc_rect, -130 * 16, -280 * 16)  #-50 * 16, 280 * 16

        # Draw the dynamic arc
        if self.brake_pressed:
            arc_color = QColor(255, 0, 0)
        else:
            arc_color = QColor(0, 255, 0)

        painter.setPen(QPen(arc_color, 30))
        start_angle = -130 * 16
        span_angle = int(-280 * (self.speed / self.max_speed))

        painter.drawArc(arc_rect, start_angle, span_angle * 16)

    def update_speed(self, speed):
        self.speed = speed
        self.speed_label.setText(str(self.speed))
        self.update()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_S:
            self.brake_pressed = True

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_S:
            self.brake_pressed = False

    def draw_indicators(self, painter):
        indicator_width = 40
        indicator_height = 40
        indicator_spacing = 10
        indicator_margin = 20

        # Draw left turn signal
        left_signal_rect = QRectF(200, indicator_margin, indicator_width, indicator_height)
        if self.left_signal_on:
            painter.drawImage(left_signal_rect, self.left_signal_active)
        else:
            painter.drawImage(left_signal_rect, self.left_signal_inactive)

        # Draw right turn signal
        right_signal_rect = QRectF(500 + indicator_width + indicator_spacing, indicator_margin,
                                   indicator_width, indicator_height)
        if self.right_signal_on:
            painter.drawImage(right_signal_rect, self.right_signal_active)
        else:
            painter.drawImage(right_signal_rect, self.right_signal_inactive)

        # Draw horn status
        horn_rect = QRectF(310, 325,
                           indicator_width, indicator_height)
        if self.horn_on:
            painter.drawImage(horn_rect, self.horn_active)
        else:
            painter.drawImage(horn_rect, self.horn_inactive)

        # Draw radio communication status
        radio_rect = QRectF(450, 325,
                            indicator_width, indicator_height)
        if self.radio_on:
            painter.drawImage(radio_rect, self.radio_active)
        else:
            painter.drawImage(radio_rect, self.radio_inactive)

        # Draw cruise control status
        cruise_rect = QRectF(375, 160,
                             indicator_width, indicator_height)
        if self.cruise_on:
            painter.drawImage(cruise_rect, self.cruise_active)
        else:
            painter.drawImage(cruise_rect, self.cruise_inactive)

    def draw_needlerpm(self, painter):
        needle_length = self.rpm_needle_length
        needle_center = QPoint(self.width() - 682, self.height()-100)

        # Calculate the angle based on the RPM value
        rpm_angle = (self.rpm_value / self.rpm_max) * 300

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45)
        painter.translate(-needle_center)

        # Define the needle shape as a polygon
        needle_points = QPolygon([
            QPoint(needle_center.x() - 3, needle_center.y() - 10),
            QPoint(needle_center.x() + 3, needle_center.y() - 10),
            QPoint(needle_center.x(), needle_center.y() + needle_length)
        ])

        # Set the needle color and draw it
        needle_color = QColor(255, 255, 255)  
        painter.setPen(QPen(needle_color, 2))
        painter.setBrush(needle_color)
        painter.drawPolygon(needle_points)

        # Reset the painter transform
        painter.resetTransform()

    def draw_needleregen(self, painter):
        needle_length = self.rpm_needle_length
        needle_center = QPoint(self.width() - 113, self.height()-100)

        # Calculate the angle based on the RPM value
        rpm_angle = (self.rpm_value / self.rpm_max) * 300

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45)
        painter.translate(-needle_center)

        # Define the needle shape as a polygon
        needle_points = QPolygon([
            QPoint(needle_center.x() - 3, needle_center.y() - 10),
            QPoint(needle_center.x() + 3, needle_center.y() - 10),
            QPoint(needle_center.x(), needle_center.y() + needle_length)
        ])

        # Set the needle color and draw it
        needle_color = QColor(255, 255, 255)  
        painter.setPen(QPen(needle_color, 2))
        painter.setBrush(needle_color)
        painter.drawPolygon(needle_points)

        # Reset the painter transform
        painter.resetTransform()
    
    def keyPressEvent(self, event):
        key = event.key()
        if key in self.key_press_mapping:
            self.key_press_mapping[key]()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    steering_display = SteeringDisplay()
    steering_display.show()
    sys.exit(app.exec_())
