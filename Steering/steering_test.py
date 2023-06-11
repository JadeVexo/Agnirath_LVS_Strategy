import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import smbus
import time
import socket
import random

'''
Additional parameters for the GPIO.setup function as per docum$
    ->channel(str) pin number
    ->direction(int) GPIO.IN / GPIO.OUT
    ->pull_up_down(int) GPIO.PUD_UP / GPIO.PUD_DOWN
    ->initial(int) initial/default value of the channel (GPIO.$
    ->delay(int) time to wait for exporting the GPIO
'''

# class DigOut:

    # def __init__(outPin):
    #     self.outPin = outPin
    #     GPIO.setup(self.outPin,GPIO.OUT,initial=GPIO.LOW)

    # def writeHigh(self):
    #     GPIO.output(self.outPin,GPIO.HIGH)

    # def writeLow(self):
    #     GPIO.output(self.outPin,GPIO.LOW)

class Button:

    def __init__(self,PIN):
        self.PIN = PIN #give pin no. as a string like P9_13
        GPIO.setup(self.PIN,GPIO.IN,initial=GPIO.LOW)

    def isPressed(self):

        

        if GPIO.input(self.PIN):
            return True
     
        else:
            return False
        
    

class AnalogIn:

    def __init__(self,aPin):
        self.aPin = aPin
        ADC.setup()

    def readVal(self):
            self.val = ADC.read(self.aPin)
            return(self.val)

# class Joystick:

#     def __init__(self, X_PIN, Y_PIN, SW_PIN):

#         self.X_PIN = X_PIN
#         self.Y_PIN = Y_PIN
#         self.SW_PIN = SW_PIN
#         self.xIn = AnalogIn(self.X_PIN)
#         self.yIn = AnalogIn(self.Y_PIN)
#         self.swIn = Button(self.SW_PIN)

#     def read_pins(self):
#         self.xVal = self.xIn.readVal()
#         self.yVal = self.yIn.readVal()
#         self.swVal = self.swIn.isPressed()
#         return(self.xVal,self.yVal,self.swVal)

'''
class MPU_6050:

    def __init__(self,MPU_BUS_NO):
        
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

    def read_raw_data(self,addr):
            
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

        return (self.accel_x,self.accel_y,self.accel_z,self.gyro_x,self.gyro_y,self.gyro_z)
   ''' 


class Indicator:

    def __init__(self,IND_X_PIN,IND_SW_PIN):

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
            return 1
            #TURN ON INDICATOR
            print("RIGHT INDICATOR ON")
        

        if self.xVal <= self.lt_th:
            return 1
            #TURN ON INDICATOR
            print("LEFT INDICATOR ON")
        

        if self.swVal :
            return 0
            #SWITCH OFF INDICATOR
            print("INDICATOR SWITCHED OFF")

    

        
class Horn : 

    def __init__(self , HN_BTN_PIN):

        self.HN_BTN_PIN = HN_BTN_PIN

        self.input = Button(self.HN_BTN_PIN)

    def bajaao(self):

        horn_signal = self.input.isPressed()

        if horn_signal :

            # BLOW HORN
            return 1
            print("!!!!!BEEEP BEEP BEEP!!!!!!")

        else: 
            return 0



class DriveSelect:

    def __init__(self,X_PIN,Y_PIN,SW_PIN):
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

    def mode_select(self):

        self.xVal = self.xIn.readVal()
        self.yVal = self.yIn.readVal()
        self.swVal = self.swIn.isPressed()

        if self.xVal >= self.rt_th :
            return 2
            #CRUISE CONTROL
            print("CAR'S IN D2")

        if self.xVal <= self.lt_th :
            return 1
            #CRUISE CONTROL
            print("CAR'S IN D1")

        if self.yVal >= self.up_th :
            return 0
            #DRIVE CONTROL
            print("CAR'S IN NEUTRAL")

        if self.yVal <= self.dn_th :
            return 3
            #REVERSE CONTROL
            print("CAR'S IN REVERSE")

        if self.swVal :

            #NEUTRAL 
            print("CAR'S IN NEUTRAL")



        
# def temp_fn(): #function to be called when button is pressed

#         print("BUTTON PRESSED")

# swpin = "P9_41" #pin from button connected to BBB
# xpin = "P9_33"
# ypin = "P9_35"

#MPU PINS AND BUS
MPU_I2C_BUS = 2

#DRIVE SELECT PINS
CLUTCH_PIN = "P9_23" #change this
DS_X_PIN = "P9_40"
DS_Y_PIN = "P9_39"
DS_SW_PIN  = "P8_14"

#Indicator Pins
IND_X_PIN = "P9_38"
IND_SW_PIN = "P9_41"

#OTHER BUTTON PINS
HORN_PIN = "P9_11"
B3_PIN = "P9_27"
B4_PIN = "P8_26"
B5_PIN = "P8_19"

# test_stick = Joystick(xpin,ypin,swpin)

#imu1 = MPU_6050(MPU_I2C_BUS)
clutch = Button(CLUTCH_PIN)
dr_sel = DriveSelect(DS_X_PIN,DS_Y_PIN,DS_SW_PIN)
horn = Horn(HORN_PIN)
button3 = Button(B3_PIN)
button4 = Button(B4_PIN)
button5 = Button(B5_PIN)
indicator = Indicator(IND_X_PIN,IND_SW_PIN)

def send_data():
    print("1")
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_address = ('', 8888)
    server_socket.bind(server_address)
    server_socket.listen(1)
    print("2")
    client_socket, client_address = server_socket.accept()
    server_socket.close()
    print("3")
    speed = 0
    rpm = 0
    regen = 0
    battery = 0
    disrem = 3000
    mode = 0
    brake = 0
    horn1 = 0
    radio = 0
    cruise = 0
    Lindicator = 0
    Rindicator = 0
    print("hi")
    while True:
        # print(test_stick.read_pins())
        # print(imu1.get_data())

   
        #if btn1.isPressed():
            #    temp_fn()
        speed += 1
        if speed > 150:
            speed = 0

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
            disrem = 3000

        Lindicator = indicator.indicate()
        Rindicator = indicator.indicate()

        if clutch.isPressed() :
        
            mode = DriveSelect.mode_select()
   

        horn1 = horn.bajaao()
    
        if button3.isPressed() == True:
            radio = 1
            print("Button 3")
        else:
            radio = 0
        
  
        if button4.isPressed() == True:
            cruise = 1
            print("Button 4 Pressed")
        else:
            cruise = 0

        if button5.isPressed() == True:
            brake = 1
            print("Button 5 Pressed")
        else:
            brake = 0

        variable_data = f"{speed} {rpm} {mode} {regen} {battery} {disrem} {brake} {horn1} {radio} {cruise} {Lindicator} {Rindicator}\n"
        print(variable_data)
        client_socket.sendall(variable_data.encode())
        time.sleep(1)

    client_socket.close()
print("helo")
send_data()
        
    
