import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap, QFont, QTransform, QImage, QPolygon, QFontMetrics
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal, QPoint, QTime
import socket

class VariableThread(QThread):
    variables_received = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.variables = ["speed", "rpm", "mode", "regen", "battery", "disrem","brake", "horn", "radio", "cruise", "Lindicator", "Rindicator"]
        self.socket = None

    def run(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(('localhost', 8888))

        while True:
            variable_data = self.socket.recv(1024)
            if not variable_data:
                break
            variable_values = variable_data.decode().strip().split()

            values = []
            for i, value in enumerate(variable_values):
                variable_name = self.variables[i]
                variable_value = int(value)
                values.append((variable_name, variable_value))

            self.variables_received.emit(values)

        self.socket.close()

class SteeringDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowFlag(Qt.FramelessWindowHint)
        self.setWindowTitle("Steering Display")
        # self.setGeometry(500, 100, 800, 480)
        self.setFixedSize(800, 480)

        self.speed = 0
        self.max_speed = 150
        
        self.rpm = 0
        self.rpm_max = 1700
        self.rpm_angle = 0
        self.rpm_needle_length = 75

        self.regen = 0
        self.regen_max = 100
        self.regen_angle = 0
        self.regen_needle_length = 75

        self.battery = 0
        self.battery_max = 100

        self.mode = 0 
        self.horn_value = 0
        self.radio_value = 0
        self.cruise_value = 0
        self.l_indicator_value = 0
        self.r_indicator_value = 0 
        self.brake_value = 0

        self.variable_thread = VariableThread()
        self.variable_thread.variables_received.connect(self.update_values)
        self.variable_thread.start()
        
        self.speed_label = QLabel(self)
        self.speed_label.setGeometry(350, 200, 100, 50)
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
        self.regen_label.setText("0%")

        self.rpm_label = QLabel(self)
        self.rpm_label.setGeometry(65, 400, 100, 50)
        self.rpm_label.setAlignment(Qt.AlignCenter)
        self.rpm_label.setStyleSheet("font-family: 'Good Times'; font-size: 12px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        )
        self.rpm_label.setText("0")

        self.dmode_label = QLabel(self)
        self.dmode_label.setGeometry(350, 265, 100, 50)
        self.dmode_label.setAlignment(Qt.AlignCenter)
        self.dmode_label.setStyleSheet("font-family: 'Good Times'; font-size: 35px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        )
        self.dmode_label.setText("N")

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

        transform = QTransform().rotate(180)
        self.image_below.setPixmap(self.image_below.pixmap().transformed(transform))

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

        self.current_time = ""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)  

    def update_time(self):
        current = QTime.currentTime()
        self.current_time = current.toString("hh:mm")  
        self.update()
        
    def update_values(self, values):
        for variable_name, variable_value in values:
            if variable_name == "speed":
                self.speed_value = variable_value
                self.update_speed()
            elif variable_name == "rpm":
                self.rpm_value = variable_value
                self.update_rpm()
            elif variable_name == "mode":
                self.mode_value = variable_value
                self.update_mode()
            elif variable_name == "regen":
                self.regen_value = variable_value
                self.update_regen()
            elif variable_name == "battery":
                self.battery_value = variable_value
                self.update_battery()
            elif variable_name == "disrem":
                self.disrem_value = variable_value
                self.update_disrem()
            elif variable_name == "brake":
                self.brake_value = variable_value
            elif variable_name == "horn":
                self.horn_value = variable_value
            elif variable_name == "radio":
                self.radio_value = variable_value
            elif variable_name == "cruise":
                self.cruise_value = variable_value
            elif variable_name == "Lindicator":
                self.l_indicator_value = variable_value
            elif variable_name == "Rindicator":
                self.r_indicator_value = variable_value

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

        self.batterystatus(painter)

        self.time(painter)

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
        if self.brake_value:
            arc_color = QColor(255, 0, 0)
        else:
            arc_color = QColor(0, 255, 0)

        painter.setPen(QPen(arc_color, 30))
        start_angle = -130 * 16
        span_angle = int(-280 * (self.speed / self.max_speed))

        painter.drawArc(arc_rect, start_angle, span_angle * 16)

    def update_speed(self):
        self.speed = self.speed_value
        self.speed_label.setText(str(self.speed))
        self.update()

    def update_mode(self):
        self.mode = self.mode_value
        if self.mode == 0:
            self.dmode_label.setText("N")
        elif self.mode == 1:
            self.dmode_label.setText("D1")
        elif self.mode == 2:
            self.dmode_label.setText("D2")
        else:
            self.dmode_label.setText("R")
        self.update()

    def update_disrem(self):
        self.dist_label.setText("DST.RMN\n" + str(self.disrem_value))
        self.update()

    def draw_indicators(self, painter):
        # Draw left turn signal
        left_signal_rect = QRectF(205, 20, 40, 40)
        if self.l_indicator_value:
            painter.drawImage(left_signal_rect, self.left_signal_active)
        else:
            painter.drawImage(left_signal_rect, self.left_signal_inactive)

        # Draw right turn signal
        right_signal_rect = QRectF(555,20,40,40)
        if self.r_indicator_value:
            painter.drawImage(right_signal_rect, self.right_signal_active)
        else:
            painter.drawImage(right_signal_rect, self.right_signal_inactive)

        # Draw horn status
        horn_rect = QRectF(310, 325, 35, 35)
        if self.horn_value:
            painter.drawImage(horn_rect, self.horn_active)
        else:
            painter.drawImage(horn_rect, self.horn_inactive)

        # Draw radio communication status
        radio_rect = QRectF(450, 325, 35, 35)
        if self.radio_value:
            painter.drawImage(radio_rect, self.radio_active)
        else:
            painter.drawImage(radio_rect, self.radio_inactive)

        # Draw cruise control status
        cruise_rect = QRectF(375, 125,40,40)
        if self.cruise_value:
            painter.drawImage(cruise_rect, self.cruise_active)
        else:
            painter.drawImage(cruise_rect, self.cruise_inactive)

    def update_battery(self):
        self.battery = self.battery_value
        self.update()
   
    def batterystatus(self, painter):
        if self.battery == 0:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery 0.png')
            painter.drawImage(QPoint(630,50), image)
        elif self.battery >= 0 and self.battery <= 20:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery 20.png')
            painter.drawImage(QPoint(630,50), image)
        elif self.battery > 20 and self.battery <= 40:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery 40.png')
            painter.drawImage(QPoint(630,50), image)
        elif self.battery > 40 and self.battery <= 60:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery 60.png')
            painter.drawImage(QPoint(630,50), image)
        elif self.battery > 60 and self.battery <= 80:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery 80.png')
            painter.drawImage(QPoint(630,50), image)
        else:
            image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/battery.png')
            painter.drawImage(QPoint(630,50), image)

        pen = QPen(Qt.white)
        pen.setWidth(5)
        painter.setPen(pen)        

        font = QFont()
        font.setFamily('Good Times')
        font.setBold(True)
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(673, 112, str(self.battery)+"%")
        painter.end()

    def update_rpm(self):
        self.rpm = self.rpm_value
        self.rpm_label.setText(str(self.rpm))
        self.update()

    def draw_needlerpm(self, painter):
        needle_length = self.rpm_needle_length
        needle_center = QPoint(self.width() - 682, self.height()-100)

        # Calculate the angle based on the RPM value
        rpm_angle = (self.rpm / self.rpm_max) * 270

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45 + rpm_angle)
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

    def update_regen(self):
        self.regen = self.regen_value
        self.regen_label.setText(str(self.regen))
        self.update()

    def draw_needleregen(self, painter):
        needle_length = self.regen_needle_length
        needle_center = QPoint(self.width() - 113, self.height()-100)

        # Calculate the angle based on the RPM value
        regen_angle = (self.regen / self.regen_max) * 270

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45 + regen_angle)
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
    
    def time(self, painter):
        painter.begin(self)

        image  = QImage('/home/veadesh/Agnirath/GUI/Steering/SteeringV2/Speedometer final/assets/Rectangle.png')
        painter.drawImage(QPoint(325,390), image)

        pen = QPen(Qt.white)
        pen.setWidth(5)
        painter.setPen(pen)        

        font = QFont()
        font.setFamily('Good Times')
        font.setBold(True)
        font.setPointSize(29)
        painter.setFont(font)
        fm = QFontMetrics(font)
        time = self.current_time
        text_width = fm.width(time)
        text_height = fm.height()
        text_x = 323 + (image.width() - text_width) // 2
        text_y = 430 + (image.height() - text_height) // 2
        painter.drawText(text_x, text_y, time)

        painter.end()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    steering_display = SteeringDisplay()
    steering_display.show()
    sys.exit(app.exec_())
