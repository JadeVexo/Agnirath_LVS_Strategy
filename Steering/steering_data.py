import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import smbus
import time
import socket


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

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

server_address = ("", 8888)
server_socket.bind(server_address)
server_socket.listen(1)

client_socket, client_address = server_socket.accept()
server_socket.close()

speed = 0
rpm = 0
regen = 0
battery = 0
disrem = 3000
mode = 0
omode = 0
brake = 0
horn_1 = 0
radio = 0
cruise = 0
old_cruise = 0
Lindicator = 0
old_L = 0
Rindicator = 0
old_R = 0
var  = " "

while True:
    # print(imu1.get_data())

    if indicator.indicate() == 1:
        Rindicator = 1
        Lindicator = 0
    elif indicator.indicate() == 2:
        Lindicator = 1
        Rindicator = 0
    else:
        Rindicator = 0
        Lindicator = 0

    speed += 1
    if speed > 150:
        speed = 0

    if clutch.isPressed():
        mode = dr_sel.mode_select()
        omode = mode
    else:
        mode = omode

    # mode = dr_sel.mode_return()
    # mode1 = dr_sel.mode_select()
    # mode = 0

    rpm += 10
    if rpm > 1700:
        rpm = 0

    regen += 1
    if regen > 100:
        regen = 0

    battery += 1
    if battery > 100:
        battery = 0

    disrem -= 10
    if disrem < 0:
        disrem == 3000

    brake = button3.isPressed()
    horn_1 = horn.isPressed()
    radio = button4.isPressed()

    if button5.isPressed():
        if old_cruise == 1:
            cruise = 0
            old_cruise = cruise
        else:
            cruise = 1
            old_cruise = cruise
    else:
        cruise = old_cruise

    data = [
        speed,
        rpm,
        regen,
        battery,
        disrem,
        brake,
        horn_1,
        radio,
        cruise,
        Lindicator,
        Rindicator,
    ]
    variable_data = f"{speed} {rpm} {mode} {regen} {battery} {disrem} {brake} {horn_1} {radio} {cruise} {Lindicator} {Rindicator}\n"
    try:
        client_socket.sendall(variable_data.encode())
        var = variable_data  
    except IndexError:
        
        client_socket.sendall(var.encode())
    
    print(variable_data)
    # client_socket.sendall(variable_data.encode())
    time.sleep(0.1)

client_socket.close()
