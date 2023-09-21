import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import (
    QPainter,
    QColor,
    QPen,
    QPixmap,
    QFont,
    QTransform,
    QImage,
    QPolygon,
    QFontMetrics,
)
from PyQt5.QtCore import Qt, QRectF, QTimer, QThread, pyqtSignal, QPoint, QTime
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import smbus
import can
from time import sleep
import struct

class Button:
    def __init__(self, PIN):
        self.PIN = PIN  # give pin no. as a string like P9_13
        GPIO.setup(self.PIN, GPIO.IN, initial=GPIO.LOW)

    def isPressed(self):
        if GPIO.input(self.PIN):
            return 1

        else:
            return 0


class AnalogIn:
    def __init__(self, aPin):
        self.aPin = aPin
        ADC.setup()

    def readVal(self):
        self.val = ADC.read(self.aPin)
        return self.val


class MPU_6050:
    def __init__(self, MPU_BUS_NO):
        # MPU6050 registers addresses
        self.MPU6050_ADDR = 0x68
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT = 0x3B
        self.ACCEL_YOUT = 0x3D
        self.ACCEL_ZOUT = 0x3F
        self.GYRO_XOUT = 0x43
        self.GYRO_YOUT = 0x45
        self.GYRO_ZOUT = 0x47

        self.MPU_BUS_NO = MPU_BUS_NO
        self.MPU_BUS = smbus.SMBus(self.MPU_BUS_NO)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0x00)
        # Configure the accelerometer and gyroscope
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.SMPLRT_DIV, 0x07)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.CONFIG, 0x06)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.GYRO_CONFIG, 0x18)
        self.MPU_BUS.write_byte_data(self.MPU6050_ADDR, self.ACCEL_CONFIG, 0x01)

    def read_raw_data(self, addr):
        self.high = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr)
        self.low = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr + 1)
        self.value = (self.high << 8) + self.low

        if self.value > 32767:
            self.value = self.value - 65536
        return self.value

    def get_data(self):
        #       def read_raw_data(self,addr):
        #
        #           self.high = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr)
        #           self.low = self.MPU_BUS.read_byte_data(self.MPU6050_ADDR, addr + 1)
        #           self.value = (self.high << 8) + self.lowlow
        #
        #           if self.value > 32767:
        #               self.value = self.value - 65536
        #           return self.value

        self.accel_x = self.read_raw_data(self.ACCEL_XOUT)
        self.accel_y = self.read_raw_data(self.ACCEL_YOUT)
        self.accel_z = self.read_raw_data(self.ACCEL_ZOUT)

        # Read gyroscope data
        self.gyro_x = self.read_raw_data(self.GYRO_XOUT)
        self.gyro_y = self.read_raw_data(self.GYRO_YOUT)
        self.gyro_z = self.read_raw_data(self.GYRO_ZOUT)

        return (
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
        )


class Indicator:
    def __init__(self, IND_X_PIN, IND_SW_PIN):
        self.IND_X_PIN = IND_X_PIN
        self.IND_SW_PIN = IND_SW_PIN

        # Set thresholds for right and left indication

        self.rt_th = 0.75
        self.lt_th = 0.25

        # make instances of the Classes you are using to get inputs from the joystick

        self.xIn = AnalogIn(self.IND_X_PIN)
        self.swIn = Button(self.IND_SW_PIN)

    def indicate(self):
        self.xVal = self.xIn.readVal()
        # self.yVal = self.yIn.readVal()
        self.swVal = self.swIn.isPressed()

        if self.xVal >= self.rt_th:
            print("R On")
            return 1

        if self.xVal <= self.lt_th:
            print("L On")
            return 2

        if self.swVal:
            print("Indicator Off")
            return 0


class DriveSelect:
    def __init__(self, X_PIN, Y_PIN, SW_PIN):
        self.X_PIN = X_PIN
        self.Y_PIN = Y_PIN
        self.SW_PIN = SW_PIN

        self.rt_th = 0.75
        self.lt_th = 0.25
        self.up_th = 0.75
        self.dn_th = 0.25

        self.xIn = AnalogIn(self.X_PIN)
        self.yIn = AnalogIn(self.Y_PIN)
        self.swIn = Button(self.SW_PIN)
        self.mode = 0

    def mode_select(self):
        self.xVal = self.xIn.readVal()
        self.yVal = self.yIn.readVal()
        self.swVal = self.swIn.isPressed()

        if self.xVal >= self.rt_th:
            print("D1")
            self.mode = 2
            return self.mode

        elif self.xVal <= self.lt_th:
            print("D2")
            self.mode = 1
            return self.mode

        elif self.yVal >= self.up_th:
            print("N")
            self.mode = 0
            return self.mode

        elif self.yVal <= self.dn_th:
            print("R")
            self.mode = 3
            return self.mode

        elif self.swVal:
            print("Cruise off")

        else:
            return self.mode


# MPU PINS AND BUS
MPU_I2C_BUS = 2

# DRIVE SELECT PINS
CLUTCH_PIN = "P9_23"  # change this
DS_X_PIN = "P9_40"
DS_Y_PIN = "P9_39"
DS_SW_PIN = "P8_14"

# Indicator Pins
IND_X_PIN = "P9_38"
IND_SW_PIN = "P9_41"

# OTHER BUTTON PINS
HORN_PIN = "P9_11"
B3_PIN = "P9_27"
B4_PIN = "P8_26"
B5_PIN = "P8_19"

# test_stick = Joystick(xpin,ypin,swpin)

# imu1 = MPU_6050(MPU_I2C_BUS)
clutch = Button(CLUTCH_PIN)
dr_sel = DriveSelect(DS_X_PIN, DS_Y_PIN, DS_SW_PIN)
horn = Button(HORN_PIN)
button3 = Button(B3_PIN)
button4 = Button(B4_PIN)
button5 = Button(B5_PIN)
indicator = Indicator(IND_X_PIN, IND_SW_PIN)


class VariableThread(QThread):
    variables_received = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        self.Rindicator = 0
        self.Lindicator = 0
        self.disrem = 3000
        self.cruiseval = 0
        self.old_cruise = 0
        self.hazardval = 0
        self.hornval = 0
        self.radio = 0
        self.mode = 0
        self.variables = [
            "speed",
            "rpm",
            "mode",
            "regen",
            "battery",
            "disrem",
            "brake",
            "horn",
            "radio",
            "cruise",
            "Lindicator",
            "Rindicator",
        ]
    
    def right_indicator(self,state):
        bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        try: 
            if state == 1:
                message = can.Message(arbitration_id = 200, data = 1)
                bus.send(message)
            elif state == 0:
                message = can.Message(arbitration_id = 200, data = 0)
                bus.send(message)
        except can.CanError:
            print("CAN error")
        finally:
            bus.shutdown()

    def left_indicator(self,state):
        bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        try:
            if state == 1:
                message = can.Message(arbitration_id = 201, data = 1)
                bus.send(message)
            elif state == 0:
                message = can.Message(arbitration_id = 201, data = 0)
                bus.send(message)
        except can.CanError:
            print("CAN error")
        finally:
            bus.shutdown()

    def horn(self,state):
        bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        try:
            if state == 1:
                message = can.Message(arbitration_id = 202, data = 1)
                bus.send(message)
            elif state == 0:
                message = can.Message(arbitration_id = 202, data = 0)
                bus.send(message)
        except can.CanError:
            print("CAN error")
        finally:
            bus.shutdown()

    def hazard(self,state):
        bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        try:
            if state == 1:
                message = can.Message(arbitration_id = 203, data = 1)
                bus.send(message)
            elif state == 0:
                message = can.Message(arbitration_id = 203, data = 0)
                bus.send(message)
        except can.CanError:
            print("CAN error")
        finally:
            bus.shutdown()

    def cruise(self,speed):
        bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
        try:
            message = can.Message(arbitration_id = 204, data = speed)
            bus.send(message)
        except can.CanError:
            print("CAN error")
        finally:
            bus.shutdown()
            
            
    def run(self):
        
        try:
            while True:
                self.disrem -= 10
                if self.disrem < 0:
                    self.disrem == 3000
                self.mode += 1
                if self.mode > 3:
                    self.mode = 0

                response = self.bus.recv(timeout=2)
                data = response.data
                arbitration_id = response.arbitration_id
                if arbitration_id == 1026:
                    motor_current = struct('<f',data[4:8])
                if arbitration_id == 1027:
                    motor_velocity = struct('<f',data[0:4])
                    vehicle_velocity = struct('<f',data[4:8])
                if arbitration_id == 1781:
                    battery_SOC = struct('<f',data[4:8])
                if arbitration_id == 0: #Change to what the main PCB transmits for state
                    state = int(data)

                if button3.isPressed():
                    self.hazard(1)
                    self.hazardval = 1
                else:
                    self.hazardval = 0

                if horn.isPressed():
                    self.horn(1)
                    self.hornval = 1
                else:
                    self.hornval = 0

                if button4.isPressed():
                    self.radio = 1
                else:
                    self.radio = 0

                if button5.isPressed():
                    if self.old_cruise == 1:
                        self.cruiseval = 0
                        self.old_cruise = self.cruiseval
                    else:
                        self.cruiseval = 1
                        self.cruise(vehicle_velocity)
                        self.old_cruise = self.cruiseval
                else:
                    self.cruiseval = self.old_cruise
                
                if indicator.indicate() == 1:
                    self.Rindicator = 1
                    self.right_indicator(1)
                    self.Lindicator = 0
                    self.left_indicator(0)
                elif indicator.indicate() == 2:
                    self.Lindicator = 1
                    self.left_indicator(1)
                    self.Rindicator = 0
                    self.right_indicator(0)
                else:
                    self.Rindicator = 0
                    self.right_indicator(0)
                    self.Lindicator = 0
                    self.left_indicator(0)
                
                values = [motor_current,motor_velocity,vehicle_velocity,battery_SOC,state,self.mode,self.hazardval,self.hornval,self.radio,self.cruiseval,self.Lindicator,self.Rindicator,self.disrem ]
                self.variables_received.emit(values)


        except can.CanError:
            print("CAN error")  
        finally:
            self.bus.shutdown()        


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
        self.hazard_value = 0

        self.variable_thread = VariableThread()
        self.variable_thread.variables_received.connect(self.update_values)
        self.variable_thread.start()

        self.speed_label = QLabel(self)
        self.speed_label.setGeometry(350, 200, 100, 50)
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.speed_label.setStyleSheet(
            "font-family: 'Good Times'; font-size: 40px; font-weight: 400; "
            "line-height: 48px; letter-spacing: 0em; color: #FF4D00; "
            "background-color: #1E1E1E"
        )

        self.dist_label = QLabel(self)
        self.dist_label.setGeometry(40, 65, 100, 50)
        self.dist_label.setAlignment(Qt.AlignCenter)
        self.dist_label.setStyleSheet(
            "font-family: 'Good Times'; font-size: 15px; font-weight: 400; "
            "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
            "background-color: #1E1E1E"
        )
        self.dist_label.setText("DST.RMN\n3000")

        self.regen_label = QLabel(self)
        self.regen_label.setGeometry(640, 400, 100, 50)
        self.regen_label.setAlignment(Qt.AlignCenter)
        self.regen_label.setStyleSheet(
            "font-family: 'Good Times'; font-size: 12px; font-weight: 400; "
            "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
        )
        self.regen_label.setText("0%")

        self.rpm_label = QLabel(self)
        self.rpm_label.setGeometry(65, 400, 100, 50)
        self.rpm_label.setAlignment(Qt.AlignCenter)
        self.rpm_label.setStyleSheet(
            "font-family: 'Good Times'; font-size: 12px; font-weight: 400; "
            "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
        )
        self.rpm_label.setText("0")

        self.dmode_label = QLabel(self)
        self.dmode_label.setGeometry(350, 265, 100, 50)
        self.dmode_label.setAlignment(Qt.AlignCenter)
        self.dmode_label.setStyleSheet(
            "font-family: 'Good Times'; font-size: 35px; font-weight: 400; "
            "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
        )
        self.dmode_label.setText("N")

        self.image_above = QLabel(self)
        self.image_above.setGeometry(300, 100, 200, 100)
        self.image_above.setPixmap(
            QPixmap(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/ellipse_2.png"
            ).scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio)
        )

        self.image_below = QLabel(self)
        self.image_below.setGeometry(300, 230, 200, 100)
        self.image_below.setPixmap(
            QPixmap(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/ellipse_3.png"
            ).scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio)
        )

        self.disellipse = QLabel(self)
        self.disellipse.setGeometry(40, 40, 200, 100)
        self.disellipse.setPixmap(
            QPixmap(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/ellipse_7.png"
            ).scaled(200, 100, Qt.AspectRatioMode.KeepAspectRatio)
        )

        self.rpmellipse = QLabel(self)
        self.rpmellipse.setGeometry(30, 290, 350, 175)
        self.rpmellipse.setPixmap(
            QPixmap(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/rpm.png"
            ).scaled(350, 175, Qt.AspectRatioMode.KeepAspectRatio)
        )

        self.regenellipse = QLabel(self)
        self.regenellipse.setGeometry(600, 290, 350, 175)
        self.regenellipse.setPixmap(
            QPixmap(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/Regen.png"
            ).scaled(350, 175, Qt.AspectRatioMode.KeepAspectRatio)
        )

        transform = QTransform().rotate(180)
        self.image_below.setPixmap(self.image_below.pixmap().transformed(transform))

        self.left_signal_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/left_active.png"
        )
        self.left_signal_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/left_inactive.png"
        )
        self.right_signal_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/union1_state_property_1_Variant2.png"
        )
        self.right_signal_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/union1_state_property_1_Default.png"
        )
        self.horn_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/Horn_active.png"
        )
        self.horn_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/Horn_Inactive.png"
        )
        self.radio_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/radio_active.png"
        )
        self.radio_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/radio_inactive.png"
        )
        self.cruise_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/cruise_active.png"
        )
        self.cruise_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/cruise_inactive.png"
        )

        self.hazard_active = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/hazard_active.png"
        )

        self.hazard_inactive = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/hazard_active.png"
        )

        self.current_time = ""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)

    def update_time(self):
        current = QTime.currentTime()
        self.current_time = current.toString("hh:mm")
        self.update()

    def update_values(self, values):
        self.speed_value = values[2]
        self.update_speed()
        self.rpm_value = values[1]
        self.update_rpm()
        self.mode_value = values[5]
        self.update_mode()
        self.battery_value = values[3]
        self.update_battery()
        self.disrem_value = values[12]
        self.update_disrem()
        self.hazard_value = values[6]
        self.horn_value = values[7]
        self.radio_value = values[8]
        self.cruise_value = values[9]
        self.l_indicator_value = values[10]
        self.r_indicator_value = values[11]
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
        arc_rect = QRectF(
            arc_center.x() - arc_radius,
            arc_center.y() - arc_radius,
            2 * arc_radius,
            2 * arc_radius,
        )

        # Draw the base arc
        base_color = QColor(200, 200, 200)
        painter.setPen(QPen(base_color, 30))
        painter.drawArc(arc_rect, -130 * 16, -280 * 16)  # -50 * 16, 280 * 16

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
        # self.update()

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
        # self.update()

    def update_disrem(self):
        self.dist_label.setText("DST.RMN\n" + str(self.disrem_value))
        # self.update()

    def draw_indicators(self, painter):
        # Draw left turn signal
        left_signal_rect = QRectF(205, 20, 40, 40)
        if self.l_indicator_value:
            painter.drawImage(left_signal_rect, self.left_signal_active)
        else:
            painter.drawImage(left_signal_rect, self.left_signal_inactive)

        # Draw right turn signal
        right_signal_rect = QRectF(555, 20, 40, 40)
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
        cruise_rect = QRectF(375, 125, 40, 40)
        if self.cruise_value:
            painter.drawImage(cruise_rect, self.cruise_active)
        else:
            painter.drawImage(cruise_rect, self.cruise_inactive)

        # Draw hazard light status
        hazard_rect = (380, 325, 40, 40)
        if self.hazard_value:
            painter.drawImage(hazard_rect, self.hazard_active)
        else:
            painter.drawImage(hazard_rect, self.hazard_inactive)

    def update_battery(self):
        self.battery = self.battery_value
        # self.update()

    def batterystatus(self, painter):
        if self.battery == 0:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery 0.png"
            )
            painter.drawImage(QPoint(630, 50), image)
        elif self.battery >= 0 and self.battery <= 20:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery 20.png"
            )
            painter.drawImage(QPoint(630, 50), image)
        elif self.battery > 20 and self.battery <= 40:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery 40.png"
            )
            painter.drawImage(QPoint(630, 50), image)
        elif self.battery > 40 and self.battery <= 60:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery 60.png"
            )
            painter.drawImage(QPoint(630, 50), image)
        elif self.battery > 60 and self.battery <= 80:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery 80.png"
            )
            painter.drawImage(QPoint(630, 50), image)
        else:
            image = QImage(
                "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/battery.png"
            )
            painter.drawImage(QPoint(630, 50), image)

        pen = QPen(Qt.white)
        pen.setWidth(5)
        painter.setPen(pen)

        font = QFont()
        font.setFamily("Good Times")
        font.setBold(True)
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(673, 112, str(self.battery) + "%")
        painter.end()

    def update_rpm(self):
        self.rpm = self.rpm_value
        self.rpm_label.setText(str(self.rpm))
        # self.update()

    def draw_needlerpm(self, painter):
        needle_length = self.rpm_needle_length
        needle_center = QPoint(self.width() - 682, self.height() - 100)

        # Calculate the angle based on the RPM value
        rpm_angle = (self.rpm / self.rpm_max) * 270

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45 + rpm_angle)
        painter.translate(-needle_center)

        # Define the needle shape as a polygon
        needle_points = QPolygon(
            [
                QPoint(needle_center.x() - 3, needle_center.y() - 10),
                QPoint(needle_center.x() + 3, needle_center.y() - 10),
                QPoint(needle_center.x(), needle_center.y() + needle_length),
            ]
        )

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
        # self.update()

    def draw_needleregen(self, painter):
        needle_length = self.regen_needle_length
        needle_center = QPoint(self.width() - 113, self.height() - 100)

        # Calculate the angle based on the RPM value
        regen_angle = (self.regen / self.regen_max) * 270

        # Rotate the painter to the desired angle
        painter.translate(needle_center)
        painter.rotate(45 + regen_angle)
        painter.translate(-needle_center)

        # Define the needle shape as a polygon
        needle_points = QPolygon(
            [
                QPoint(needle_center.x() - 3, needle_center.y() - 10),
                QPoint(needle_center.x() + 3, needle_center.y() - 10),
                QPoint(needle_center.x(), needle_center.y() + needle_length),
            ]
        )

        # Set the needle color and draw it
        needle_color = QColor(255, 255, 255)
        painter.setPen(QPen(needle_color, 2))
        painter.setBrush(needle_color)
        painter.drawPolygon(needle_points)

        # Reset the painter transform
        painter.resetTransform()

    def time(self, painter):
        painter.begin(self)

        image = QImage(
            "/home/debian/Agnirath_LVS_Strategy/Steering/Speedometer final/assets/Rectangle.png"
        )
        painter.drawImage(QPoint(325, 390), image)

        pen = QPen(Qt.white)
        pen.setWidth(5)
        painter.setPen(pen)

        font = QFont()
        font.setFamily("Good Times")
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    steering_display = SteeringDisplay()
    steering_display.show()
    sys.exit(app.exec_())
