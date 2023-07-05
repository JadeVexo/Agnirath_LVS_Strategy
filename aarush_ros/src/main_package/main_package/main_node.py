import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

"""
KEY FOR DATA LISTS:

Final Data list:[
    
    [0-13]
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

    [14-19] 
    X Acceleration
    Y Acceleration
    Z Acceleration
    X Angular Acceleration
    Y Angular Acceleration
    Z Angular Acceleration

    -----------------

    [28-29]
    BMU device ID - Display
    BMU Serial Number - Display

    [30-40] 
    CMU1 Serial Number - Display
    CMU1 PCB Temperature - Display Warning, Slow Down, Decide Later
    CMU1 Cell Temperature - Display Warning, Slow Down
    CMU1 Cell 1 Voltage - Contactor Open
    CMU1 Cell 2 Voltage - Contactor Open
    CMU1 Cell 3 Voltage - Contactor Open
    CMU1 Cell 4 Voltage - Contactor Open
    CMU1 Cell 5 Voltage - Contactor Open
    CMU1 Cell 6 Voltage - Contactor Open
    CMU1 Cell 7 Voltage - Contactor Open

    [41-51]
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

    [52-62]
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

    [63-73]
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
    
    [74-84]
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

    [85-86]
    SOC(Ah) Used - Display - <0 Open Contactor
    SOC percentage - Display - <0 Open Contactor

    [87-88] - (Not useful for Controls)
    Balance SOC(Ah) Used   - Display 
    Balance SOC percentage - Display 

    [89-92] - (Not useful for Controls)
    Charging Cell Voltage Error   -
    Cell Temperature Margin       -
    Dischaging Cell Voltage Error -
    Total Pack Capacity           - 

    [93-97] - (Not useful for Controls)
    Precharge contactor driver status -
    Precharge state                   -
    12v Contactor supply voltage      -
    Precharge timer activity          -
    Precharge Timer Counter           -

    [98-103] - 
    Minimum cell voltage - 3<, Open Contactor
    Maximum cell voltage - 4>, Open Contactor
    CMU number that has the minimum cell voltage - Display
    Cell number in CMU that is the minimum cell voltage -Display
    CMU number that has the maximum cell voltage - Display
    Cell number in CMU that is the maximum cell voltage - Display

    [104-107] - 
    Minimum cell temperature - Display, Slow Down, Oopen Contactor Stop
    Maximum cell temperature - Display, Slow Down, Open Contactor Stop
    CMU number that has the minimum cell temperature
    CMU number that has the maximum cell temperature

    [108-109] 
    Battery Voltage - Display, 160 > or <120 Open Contactor
    Battery Current - Display, 30 > Open Contactor

    [110-114] 
    Balance voltage threshold - Display
    Balance voltage threshold - Display
    Status Flags - 
        1. 4>, Open Contactor
        2. 3<, Open Contactor
        4. Display, Slow Down, Open Contactor Stop
        8. Display, Cell Measurement Untrusted
        10. Display - Shutdown
        20. Display - Shutdown 
        40. Display
        80. Display

    BMS CMU count - Display
    BMS BMU Firmware Build Number -  Display

    [115-118]
    Fan speed 0 - Display
    Fan speed 1 - Display
    12V current consumption of fans + contactors - Display
    12V current consumption of CMUs - Display

    [119-121]
    Extended Status Flags - Haven't implemented this, using Normal Status Flags
        1. 4>, Open Contactor
        2. 3<, Open Contactor
        4. Display, Slow Down, Open Contactor Stop
        8. Display, Cell Measurement Untrusted
        10. Display - Shutdown
        20. Display - Shutdown 
        40. Display
        80. Display
        100. 
        200. Display, Shutdown
        400. Display, Shutdown
        800. Display, Shutdown
        1000. Display, Shutdown
    BMU Hardware version
    BMU Model ID

    [122]
    EVDC Switch Position - Display

    -------------------
    
    [123-141] (Manually Assigned BaseID of MPPT1 to 0x6A0)
    MPPT1 Input Voltage - below 20 show message
    MPPT1 Input Current - >7A module contactor open post MMPT
    MPPT1 Output Voltage - >175 Motor Current 0
    MPPT1 Output Current - >
    MPPT1 Mosfet Temperature - Open Contactor pre mmpt 
    MPPT1 Controller Temperature - Open Contactor pre mmpt 
    MPPT1 12V Aux Supply - error
    MPPT1 3V Aux Supply - error
    MPPT1 Max Output Voltage - log
    MPPT1 Max Input Current - log
    MPPT1 CAN RX error counter - Log
    MPPT1 CAN TX error counter - Log
    MPPT1 CAN TX overfow counter - Log
    MPPT1 error flag - error
    MPPT1 limit flags 
        7. Display <20 open contactor pre mppt
        6. Open Contactor pre mppt
        5. Display
        4. Open Contactor pre mppt
        3. Display
        2. Reserved
        1. Display
        0. above 175 open post mppt


    MPPT1 Mode
    MPPT1 Reserved
    MPPT1 Test Counter
    MPPT1 Output Voltage

    (Manually Assigned BaseID of MPPT2 to 0x6B0)
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

    (Manually Assigned BaseID of MPPT3 to 0x6C0)
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

    (Manually Assigned BaseID of MPPT4 to 0x6D0)
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
    
    (The following 3 things are transmitted by the EVDC)
    Motor Current
    Motor Velocity
    Bus Current

    (Everything below is transmitted through mc)
    Motor Controller Serial Number - NA
    Motor Controller Prohelion ID - NA

    Recieve Error Count - NA
    Transmit Error Count - NA
    Active Motor - NA
    Error Flags(0)
        8. Motor Over Speed -> Set motor v to 0 -> set current
        7.
        6. Open Contactor for MC and Motor
        5. 
        4.
        3. Ignore
        2. Set Motor Currnet 0 -> Open Contactor for MC and Motor
        1. Open Contactor for MC and Motor
        0. Open Contactor for MC and Motor
         
    Limit Flags
        6.
        5.
        4.
        3.
        2.
        1.

    Bus Current(1) - 
        Limit A - Drop motor current, Drop velocity
        Limit B - Open Contactor for MC and Motor
    Bus Voltage(2) - Open Contactor for MC and Motor
    Vehicle Velocity - NA
    Motor Velocity - NA
    Phase C Current(3)
        Limit A - Limit motor current
        Limit B - Open Contactor for Motor
    Phase B Current(4)
        Limit A - Limit motor current
        Limit B - Open Contactor for Motor
    Vd - NA
    Vq - NA
    Id - NA
    Iq - NA
    BEMFd(5) - Open Contactor for Motor
    BEMFq(6) - Open Contactor for Motor
    15V Supply(7) - If low Open Contactor for MC and Motor
    3.3V Supply(8) - Open Contactor for MC and Motor
    1.9V Supply(9) - Open Contactor for MC and Motor
    Heat Sink Temp(10) - Slow Down
        
    Motor Temp(11) - Drop Motor Current, slow down
    DSP Board Temp(12) -  Slow down
    DC Bus Ah - NA
    Odometer - NA
    Slip Speed - Dont Care
    Active Motor - Dont Care
    ---------
    ]
          
"""

class MAIN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

   # Control Subscriber
    def init_control_subscriber(self, topic):
        self.control_subscriber = self.create_subscription(
            rosarray, topic, self.receive_control_data, 10
        )
        self.control_subscriber  # prevent unused variable warning
        self.control_sub_data = None

    def receive_control_data(self, msg):
        self.control_sub_data = msg.data

    # CAN Subscriber
    def init_can_subscriber(self, topic):
        self.can_subscriber = self.create_subscription(
            rosarray, topic, self.receive_can_data, 10
        )
        self.can_subscriber  # prevent unused variable warning
        self.can_sub_data = None

    def receive_can_data(self, msg):
        self.can_sub_data = msg.data

    # GPS Subscriber
    def init_gps_subscriber(self, topic):
        self.gps_subscriber = self.create_subscription(
            rosarray, topic, self.receive_gps_data, 10
        )
        self.gps_subscriber  # prevent unused variable warning
        self.gps_sub_data = None

    def receive_gps_data(self, msg):
        self.gps_sub_data = msg.data

        # IMU Subscriber

    def init_imu_subscriber(self, topic):
        self.imu_subscriber = self.create_subscription(
            rosarray, topic, self.receive_imu_data, 10
        )
        self.imu_subscriber  # prevent unused variable warning
        self.imu_sub_data = None

    def receive_imu_data(self, msg):
        self.imu_sub_data = msg.data

    # CAN publisher
    def init_can_publisher(self, topic, timer_period):
        self.can_publisher = self.create_publisher(rosarray, topic, 10)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)
        self.can_pub_data = None

    def publish_can_data(self):
        if self.can_pub_data is not None:
            self.can_pub_msg = rosarray()
            self.can_pub_msg.data = self.can_pub_data
            self.can_publisher.publish(self.can_pub_msg)
            self.can_pub_data = self.can_pub_msg.data
            #print("PUB:", self.can_pub_data)

    # Final Data publisher
    def init_final_data_publisher(self, topic, timer_period):
        self.final_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.final_data_timer = self.create_timer(timer_period, self.publish_final_data)
        self.final_pub_data = None

    def publish_final_data(self):
        if self.final_pub_data is not None:
            self.final_data_pub_msg = rosarray()
            self.final_data_pub_msg.data = self.final_pub_data        
            self.final_data_publisher.publish(self.final_data_pub_msg)
            self.final_pub_data = self.final_data_pub_msg.data
            print("PUB:", self.final_pub_data)

    # Parsed publisher
    def init_parsed_data_publisher(self, topic, timer_period):
        self.parsed_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.parsed_data_timer = self.create_timer(
            timer_period, self.publish_parsed_data
        )
        self.parsed_pub_data = None

    def publish_parsed_data(self):
        if self.parsed_pub_data is not None:
            self.parsed_data_pub_msg = rosarray()
            self.parsed_data_pub_msg.data = self.parsed_pub_data
            self.parsed_data_publisher.publish(self.parsed_data_pub_msg)
            self.parsed_pub_data = self.parsed_data_pub_msg.data
            print("PUB:", self.parsed_pub_data)


def main(args=None):
    rclpy.init(args=args)

    main_node = MAIN_NODE("main_node")
    main_node.init_control_subscriber("control_data")
    main_node.init_can_subscriber("can_rx_data")
    main_node.init_gps_subscriber("gps_data")
    main_node.init_imu_subscriber("imu_data")

    main_node.init_can_publisher("can_tx_data", 1)
    #main_node.init_final_data_publisher("final_data", 1)
    main_node.init_parsed_data_publisher("parsed_data", 1)

    final_data = [0]*200
    parsed_data = [0]*150
    # can_data = None
    print("Starting Main Node")
    while rclpy.ok():
        rclpy.spin_once(main_node)

        control_sub_data = main_node.control_sub_data
        can_sub_data = main_node.can_sub_data  # Recieves CAN Data
        gps_sub_data = main_node.gps_sub_data
        imu_sub_data = main_node.imu_sub_data


        # Base address of MPPTs, Driver's Control and Motor Controller
        mppt_1_base_address = 0x6A0
        mppt_2_base_address = 0x6B0
        mppt_3_base_address = 0x6C0
        mppt_4_base_address = 0x6D0
        evdc_base_address = 0x630 # Will change this later after updating the correct base address
        mc_base_address = 0x640 # Will change this later after updating the correct base address

        # Base index of messages for different components in the parsed data array
        bms_index = 0
        mppt_1_index = 59
        mppt_2_index = 73
        mppt_3_index = 87
        mppt_4_index = 101
        evdc_index = 115
        mc_index = 117


        # Parsing data from CAN Messages; Used in Control Loops
        if can_sub_data is not None:
            # CMU 1
            if can_sub_data[0] == 0x601:
                parsed_data[bms_index+0:bms_index+2] = can_sub_data[2:4]
            if can_sub_data[0] == 0x602:
                parsed_data[bms_index+2:bms_index+6] = can_sub_data[1:5]
            if can_sub_data[0] == 0x603:
                parsed_data[bms_index+6:bms_index+10] = can_sub_data[1:5]

            # CMU 2
            if can_sub_data[0] == 0x604:
                parsed_data[bms_index+10:bms_index+12] = can_sub_data[2:4]
            if can_sub_data[0] == 0x605:
                parsed_data[bms_index+12:bms_index+16] = can_sub_data[1:5]
            if can_sub_data[0] == 0x606:
                parsed_data[bms_index+16:bms_index+20] = can_sub_data[1:5]
            
            # CMU 3
            if can_sub_data[0] == 0x607:
                parsed_data[bms_index+20:bms_index+22] = can_sub_data[2:4]
            if can_sub_data[0] == 0x608:
                parsed_data[bms_index+22:bms_index+26] = can_sub_data[1:5]
            if can_sub_data[0] == 0x609:
                parsed_data[bms_index+26:bms_index+30] = can_sub_data[1:5]

            # CMU 4
            if can_sub_data[0] == 0x610:
                parsed_data[bms_index+30:bms_index+32] = can_sub_data[2:4]
            if can_sub_data[0] == 0x611:
                parsed_data[bms_index+32:bms_index+36] = can_sub_data[1:5]
            if can_sub_data[0] == 0x612:
                parsed_data[bms_index+36:bms_index+40] = can_sub_data[1:5]

            # CMU 5
            if can_sub_data[0] == 0x613:
                parsed_data[bms_index+40:bms_index+42] = can_sub_data[2:4]
            if can_sub_data[0] == 0x614:
                parsed_data[bms_index+42:bms_index+46] = can_sub_data[1:5]
            if can_sub_data[0] == 0x615:
                parsed_data[bms_index+46:bms_index+50] = can_sub_data[1:5]

            # SOC Information
            if can_sub_data[0] == 0x6F4:
                parsed_data[bms_index+50:bms_index+52] = can_sub_data[1:3]

            # Minimum/ Maximum Cell Voltage 
            if can_sub_data[0] == 0x6F8:
                parsed_data[bms_index+52:bms_index+54] = can_sub_data[1:3]

            # Minimum/ Maximum Cell Temperature
            if can_sub_data[0] == 0x6F9:
                parsed_data[bms_index+54:bms_index+56] = can_sub_data[1:3]

            # Battery Voltage/ Current
            if can_sub_data[0] == 0x6FA:
                parsed_data[bms_index+56:bms_index+58] = can_sub_data[1:3]

            # Battery Pack Status
            if can_sub_data[0] == 0x6FB:
                parsed_data[bms_index+58] = can_sub_data[5]
            
            '''------------------------------------------------------'''

            # MPPT 1
            # Input Measurements
            if can_sub_data[0] == mppt_1_base_address + 0x000: 
                parsed_data[mppt_1_index+0:mppt_1_index+2] = can_sub_data[1:3]

            # Output Measurements
            if can_sub_data[0] == mppt_1_base_address + 0x001:
                parsed_data[mppt_1_index+2:mppt_1_index+4] = can_sub_data[1:3]

            # Temperature
            if can_sub_data[0] == mppt_1_base_address + 0x002:
                parsed_data[mppt_1_index+4:mppt_1_index+6] = can_sub_data[1:3]

            # Auxillary Power Supply 
            if can_sub_data[0] == mppt_1_base_address + 0x003:
                parsed_data[mppt_1_index+6:mppt_1_index+8] = can_sub_data[1:3]

            #  Status - 3 strings for controls are "error flags", "limit flags" and "mode"
            if can_sub_data[0] == mppt_1_base_address + 0x005:
                parsed_data[mppt_1_index+8:mppt_1_index+11] = can_sub_data[4:7]
            
            # Power Connector
            if can_sub_data[0] == mppt_1_base_address + 0x006:
                parsed_data[mppt_1_index+11:mppt_1_index+13] = can_sub_data[1:3]

            '''------------------------------------------------------------------'''

            # MPPT 2
            # Input Measurements
            if can_sub_data[0] == mppt_2_base_address + 0x000: 
                parsed_data[mppt_2_index+0:mppt_2_index+2] = can_sub_data[1:3]

            # Output Measurements
            if can_sub_data[0] == mppt_2_base_address + 0x001:
                parsed_data[mppt_2_index+2:mppt_2_index+4] = can_sub_data[1:3]

            # Temperature
            if can_sub_data[0] == mppt_2_base_address + 0x002:
                parsed_data[mppt_2_index+4:mppt_2_index+6] = can_sub_data[1:3]

            # Auxillary Power Supply 
            if can_sub_data[0] == mppt_2_base_address + 0x003:
                parsed_data[mppt_2_index+6:mppt_2_index+8] = can_sub_data[1:3]

            #  Status - 3 strings for controls are "error flags", "limit flags" and "mode"
            if can_sub_data[0] == mppt_2_base_address + 0x005:
                parsed_data[mppt_2_index+8:mppt_2_index+11] = can_sub_data[4:7]
            
            # Power Connector
            if can_sub_data[0] == mppt_2_base_address + 0x006:
                parsed_data[mppt_2_index+11:mppt_2_index+13] = can_sub_data[1:3]

            '''-----------------------------------------------------------'''

            # MPPT 3
            # Input Measurements
            if can_sub_data[0] == mppt_3_base_address + 0x000: 
                parsed_data[mppt_3_index+0:mppt_3_index+2] = can_sub_data[1:3]

            # Output Measurements
            if can_sub_data[0] == mppt_3_base_address + 0x001:
                parsed_data[mppt_3_index+2:mppt_3_index+4] = can_sub_data[1:3]

            # Temperature
            if can_sub_data[0] == mppt_3_base_address + 0x002:
                parsed_data[mppt_3_index+4:mppt_3_index+6] = can_sub_data[1:3]

            # Auxillary Power Supply 
            if can_sub_data[0] == mppt_3_base_address + 0x003:
                parsed_data[mppt_3_index+6:mppt_3_index+8] = can_sub_data[1:3]

            #  Status - 3 strings for controls are "error flags", "limit flags" and "mode"
            if can_sub_data[0] == mppt_3_base_address + 0x005:
                parsed_data[mppt_3_index+8:mppt_3_index+11] = can_sub_data[4:7]
            
            # Power Connector
            if can_sub_data[0] == mppt_3_base_address + 0x006:
                parsed_data[mppt_3_index+11:mppt_3_index+13] = can_sub_data[1:3]

            '''--------------------------------------------------------------'''

            # MPPT 4
            # Input Measurements
            if can_sub_data[0] == mppt_4_base_address + 0x000: 
                parsed_data[mppt_4_index+0:mppt_4_index+2] = can_sub_data[1:3]

            # Output Measurements
            if can_sub_data[0] == mppt_4_base_address + 0x001:
                parsed_data[mppt_4_index+2:mppt_4_index+4] = can_sub_data[1:3]

            # Temperature
            if can_sub_data[0] == mppt_4_base_address + 0x002:
                parsed_data[mppt_4_index+4:mppt_4_index+6] = can_sub_data[1:3]

            # Auxillary Power Supply 
            if can_sub_data[0] == mppt_4_base_address + 0x003:
                parsed_data[mppt_4_index+6:mppt_4_index+8] = can_sub_data[1:3]

            #  Status - 3 strings for controls are "error flags", "limit flags" and "mode"
            if can_sub_data[0] == mppt_4_base_address + 0x005:
                parsed_data[mppt_4_index+8:mppt_4_index+11] = can_sub_data[4:7]
            
            # Power Connector
            if can_sub_data[0] == mppt_4_base_address + 0x006:
                parsed_data[mppt_4_index+11:mppt_4_index+13] = can_sub_data[1:3]

            '''-----------------------------------------------------------'''

            # EVDC ### FIX INDEXING
            # Motor Current and Motor Velocity
            if can_sub_data[0] == evdc_base_address + 0x001:
                parsed_data[evdc_index+0:evdc_index+2] = can_sub_data[1:3]

            # Bus Current
            if can_sub_data[0] == evdc_base_address + 0x002:
                parsed_data[evdc_index+3:evdc_index+3] = can_sub_data[1]

            '''--------------------------------------------------'''

            # Motor Controller
            # Status Information
            if can_sub_data[0] == mc_base_address + 0x01:
                parsed_data[mc_index+0: mc_index+2] = can_sub_data[1:3]

            # Bus Current, Voltage
            if can_sub_data[0] == mc_base_address + 0x02:
                parsed_data[mc_index+2] = can_sub_data[1]

            # Phase B,C Current
            if can_sub_data[0] == mc_base_address + 0x04:
                parsed_data[mc_index+3] = can_sub_data[1]

            # Back EMF Measurement
            if can_sub_data[0] == mc_base_address + 0x07:
                parsed_data[mc_index+4] = can_sub_data[1]

            # 15V Voltage Rail
            if can_sub_data[0] == mc_base_address + 0x08:
                parsed_data[mc_index+5] = can_sub_data[1]

            # 3.3V, 1.9V Voltage Rails
            if can_sub_data[0] == mc_base_address + 0x09:
                parsed_data[mc_index+6] = can_sub_data[1]

            # Heat Sink and Motor Temperature
            if can_sub_data[0] == mc_base_address + 0x0B:
                parsed_data[mc_index+7] = can_sub_data[1]

            # DSP Board Temperature
            if can_sub_data[0] == mc_base_address + 0x0C:
                parsed_data[mc_index+8] = can_sub_data[1]

            main_node.parsed_pub_data = parsed_data


        # if control_sub_data is not None:
        #     print("SUB:", control_sub_data)
        # if can_sub_data is not None:
        #     print("SUB:", can_sub_data)
        # if gps_sub_data is not None:
        #     print("SUB:", gps_sub_data)
        # if imu_sub_data is not None:
        #     print("SUB:", imu_sub_data)

        # rclpy.spin_once(main_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
