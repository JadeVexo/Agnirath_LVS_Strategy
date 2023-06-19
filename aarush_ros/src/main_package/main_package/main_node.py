import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

'''
KEY FOR DATA LISTS:

Final Data list:[

    time_utc
    latitude
    lat_direction
    longitude
    lon_direction
    fix_quality
    num_satellites
    hdop
    altitude
    altitude_unit
    geoid_separation
    geoid_separation_unit
    age_of_dgps
    dgps_reference_id

    -------------

    X Acceleration
    Y Acceleration
    Z Acceleration
    X Angular Acceleration
    Y Angular Acceleration
    Z Angular Acceleration

    -----------------

    Cabin Temperature
    Cabin Altitude
    Cabin Pressure

    ----------------

    Cabin 02 Level

    -----------------

    Cabin C02 Level
    Cabin TVOC

    ------------------

    Cabin Temperature
    Cabin Humidity

    -----------------

    BMU device ID
    BMU Serial Number
    CMU1 Serial Number
    CMU1 PCB Temperature
    CMU1 Cell Temperature
    CMU1 Cell 0 Voltage
    CMU1 Cell 1 Voltage
    CMU1 Cell 2 Voltage
    CMU1 Cell 3 Voltage
    CMU1 Cell 4 Voltage
    CMU1 Cell 5 Voltage
    CMU1 Cell 6 Voltage
    CMU1 Cell 7 Voltage

    CMU2 Serial Number
    CMU2 PCB Temperature
    CMU2 Cell Temperature
    CMU2 Cell 0 Voltage
    CMU2 Cell 1 Voltage
    CMU2 Cell 2 Voltage
    CMU2 Cell 3 Voltage
    CMU2 Cell 4 Voltage
    CMU2 Cell 5 Voltage
    CMU2 Cell 6 Voltage
    CMU2 Cell 7 Voltage

    CMU3 Serial Number
    CMU3 PCB Temperature
    CMU3 Cell Temperature
    CMU3 Cell 0 Voltage
    CMU3 Cell 1 Voltage
    CMU3 Cell 2 Voltage
    CMU3 Cell 3 Voltage
    CMU3 Cell 4 Voltage
    CMU3 Cell 5 Voltage
    CMU3 Cell 6 Voltage
    CMU3 Cell 7 Voltage

    CMU4 Serial Number
    CMU4 PCB Temperature
    CMU4 Cell Temperature
    CMU4 Cell 0 Voltage
    CMU4 Cell 1 Voltage
    CMU4 Cell 2 Voltage
    CMU4 Cell 3 Voltage
    CMU4 Cell 4 Voltage
    CMU4 Cell 5 Voltage
    CMU4 Cell 6 Voltage
    CMU4 Cell 7 Voltage
    
    CMU5 Serial Number
    CMU5 PCB Temperature
    CMU5 Cell Temperature
    CMU5 Cell 0 Voltage
    CMU5 Cell 1 Voltage
    CMU5 Cell 2 Voltage
    CMU5 Cell 3 Voltage
    CMU5 Cell 4 Voltage
    CMU5 Cell 5 Voltage
    CMU5 Cell 6 Voltage
    CMU5 Cell 7 Voltage

    SOC(Ah) Used
    SOC percentage

    Balance SOC(Ah) Used
    Balance SOC percentage

    Charging Cell Voltage Error
    Cell Temperature Margin
    Dischaging Cell Voltage Error
    Total Pack Capacity

    Precharge contactor driver status
    Precharge state
    12v Contactor supply voltage
    Precharge timer activity
    Precharge Timer Counter

    Minimum cell voltage
    Maximum cell voltage
    CMU number that has the minimum cell voltage
    Cell number in CMU that is the minimum cell voltage
    CMU number that has the maximum cell voltage
    Cell number in CMU that is the maximum cell voltage

    Minimum cell temperature
    Maximum cell temperature
    CMU number that has the minimum cell temperature
    CMU number that has the maximum cell temperature

    Battery Voltage
    Battery Current

    Balance voltage threshold
    Balance voltage threshold
    Status Flags
    BMS CMU count
    BMS BMU Firmware Build Number

    Fan speed 0
    Fan speed 1
    12V current consumption of fans + contactors
    12V current consumption of CMUs

    Status Flags
    BMU Hardware version
    BMU Model ID

    EVDC Switch Position

    -------------------

    MPPT1 Input Voltage
    MPPT1 Input Current
    MPPT1 Output Voltage
    MPPT1 Output Current
    MPPT1 Mosfet Temperature
    MPPT1 Controller Temperature
    MPPT1 12V Aux Supply
    MPPT1 3V Aux Supply
    MPPT1 Max Output Voltage
    MPPT1 Max Input Voltage
    MPPT1 CAN RX error counter 
    MPPT1 CAN TX error counter
    MPPT1 CAN TX overfow counter
    MPPT1 error flag
    MPPT1 limit flags
    MPPT1 Mode
    MPPT1 Reserved
    MPPT1 Test Counter
    MPPT1 Output Voltage

    MPPT2 Input Voltage
    MPPT2 Input Current
    MPPT2 Output Voltage
    MPPT2 Output Current
    MPPT2 Mosfet Temperature
    MPPT2 Controller Temperature
    MPPT2 12V Aux Supply
    MPPT2 3V Aux Supply
    MPPT2 Max Output Voltage
    MPPT2 Max Input Voltage
    MPPT2 CAN RX error counter 
    MPPT2 CAN TX error counter
    MPPT2 CAN TX overfow counter
    MPPT2 error flag
    MPPT2 limit flags
    MPPT2 Mode
    MPPT2 Reserved
    MPPT2 Test Counter
    MPPT2 Output Voltage

    MPPT3 Input Voltage
    MPPT3 Input Current
    MPPT3 Output Voltage
    MPPT3 Output Current
    MPPT3 Mosfet Temperature
    MPPT3 Controller Temperature
    MPPT3 12V Aux Supply
    MPPT3 3V Aux Supply
    MPPT3 Max Output Voltage
    MPPT3 Max Input Voltage
    MPPT3 CAN RX error counter 
    MPPT3 CAN TX error counter
    MPPT3 CAN TX overfow counter
    MPPT3 error flag
    MPPT3 limit flags
    MPPT3 Mode
    MPPT3 Reserved
    MPPT3 Test Counter
    MPPT3 Output Voltage

    MPPT4 Input Voltage
    MPPT4 Input Current
    MPPT4 Output Voltage
    MPPT4 Output Current
    MPPT4 Mosfet Temperature
    MPPT4 Controller Temperature
    MPPT4 12V Aux Supply
    MPPT4 3V Aux Supply
    MPPT4 Max Output Voltage
    MPPT4 Max Input Voltage
    MPPT4 CAN RX error counter 
    MPPT4 CAN TX error counter
    MPPT4 CAN TX overfow counter
    MPPT4 error flag
    MPPT4 limit flags
    MPPT4 Mode
    MPPT4 Reserved
    MPPT4 Test Counter
    MPPT4 Output Voltage

    --------

    Motor Current
    Motor Velocity
    Bus Current

    Motor Controller Serial Number
    Motor Controller Prohelion ID

    Recieve Error Count
    Transmit Error Count
    Active Motor
    Error Flags
    Limit Flags
    Bus Current
    Bus Voltage
    Vehicle Velocity
    Motor Velocity
    Phase C Current
    Phase B Current
    Vd 
    Vq
    Id
    Iq
    BEMFd
    BEMFq
    15V Supply
    3.3V Supply
    1.9V Supply
    Heat Sink Temp
    Motor Temp
    DSP Board Temp
    DC Bus Ah
    Odometer
    Slip Speed
    Active Motor
    ---------
    ]
          
'''




class control_subscriber(Node):

    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(rosarray,'control_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data
    

class can_subscriber(Node):

    def __init__(self):
        super().__init__('can_node')
        self.subscription = self.create_subscription(rosarray,'can_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data


def main(args=None):
    rclpy.init(args=args)

    control_node = control_subscriber()
    can_node = can_subscriber()

    final_data = []

    while rclpy.ok():
        rclpy.spin_once(control_node)
        rclpy.spin_once(can_node)

        latest_control_data = control_node.get_latest_data()
        latest_can_data = can_node.get_latest_data()

        if latest_control_data is not None:
            print(latest_control_data)

        if latest_control_data is not None:
            print(latest_control_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()