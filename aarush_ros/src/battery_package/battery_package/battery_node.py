import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import time

class Timer:
    def __init__(self, threshold_crossing_time):
        self.threshold_crossing_time = threshold_crossing_time
        self.start_time = None

    def start(self):
        self.start_time = time.time()

    def elapsed(self):
        return time.time() - self.start_time

    def reset(self):
        self.start_time = None


class BATTERY_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # Battery Subscriber
    def init_battery_subscriber(self, topic):
        self.battery_subscriber = self.create_subscription(
            rosarray, topic, self.receive_battery_data, 10
        )
        self.battery_subscriber  # prevent unused variable warning
        self.battery_sub_data = None

    def receive_battery_data(self, msg):
        self.battery_sub_data = msg.data

    # Peripheral Subscriber
    def init_peripheral_subscriber(self, topic):
        self.peripheral_subscriber = self.create_subscription(
            rosarray, topic, self.receive_peripheral_data, 10
        )
        self.peripheral_subscriber  # prevent unused variable warning
        self.peripheral_sub_data = None

    def receive_peripheral_data(self, msg):
        self.peripheral_sub_data = msg.data

    # Control Publisher
    def init_control_data_publisher(self, topic, timer_period):
        self.control_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.control_data_timer = self.create_timer(
            timer_period, self.publish_control_data
        )
        self.control_pub_data = None

    def publish_control_data(self):
        if self.control_pub_data is not None:
            self.control_data_pub_msg = rosarray()
            self.control_data_pub_msg.data = self.control_pub_data
            self.control_data_publisher.publish(self.control_data_pub_msg)
            self.control_pub_data = self.control_data_pub_msg.data
            print("PUB:", self.control_pub_data)


def main(args=None):
    rclpy.init(args=args)

    battery_node = BATTERY_NODE("battery_node")
    battery_node.init_battery_subscriber("battery_data")
    battery_node.init_peripheral_subscriber("peripheral_data")
    battery_node.init_control_data_publisher("control_data", 1)

    threshold_crossing_time = 1  # Number of seconds to check for threshold crossing

    cmu_1_pcb_temp_timer = Timer(threshold_crossing_time)
    cmu_1_cell_temp_timer = Timer(threshold_crossing_time)
    cmu_1_cell_0_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_1_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_2_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_3_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_4_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_5_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_6_voltage_timer = Timer(threshold_crossing_time)
    cmu_1_cell_7_voltage_timer = Timer(threshold_crossing_time)

    cmu_2_pcb_temp_timer = Timer(threshold_crossing_time)
    cmu_2_cell_temp_timer = Timer(threshold_crossing_time)
    cmu_2_cell_0_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_1_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_2_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_3_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_4_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_5_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_6_voltage_timer = Timer(threshold_crossing_time)
    cmu_2_cell_7_voltage_timer = Timer(threshold_crossing_time)

    cmu_3_pcb_temp_timer = Timer(threshold_crossing_time)
    cmu_3_cell_temp_timer = Timer(threshold_crossing_time)
    cmu_3_cell_0_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_1_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_2_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_3_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_4_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_5_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_6_voltage_timer = Timer(threshold_crossing_time)
    cmu_3_cell_7_voltage_timer = Timer(threshold_crossing_time)

    cmu_4_pcb_temp_timer = Timer(threshold_crossing_time)
    cmu_4_cell_temp_timer = Timer(threshold_crossing_time)
    cmu_4_cell_0_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_1_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_2_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_3_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_4_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_5_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_6_voltage_timer = Timer(threshold_crossing_time)
    cmu_4_cell_7_voltage_timer = Timer(threshold_crossing_time)

    cmu_5_pcb_temp_timer = Timer(threshold_crossing_time)
    cmu_5_cell_temp_timer = Timer(threshold_crossing_time)
    cmu_5_cell_0_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_1_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_2_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_3_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_4_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_5_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_6_voltage_timer = Timer(threshold_crossing_time)
    cmu_5_cell_7_voltage_timer = Timer(threshold_crossing_time)

    battery_soc_ah_timer = Timer(threshold_crossing_time)
    battery_soc_percent_timer = Timer(threshold_crossing_time)

    min_cell_voltage_timer = Timer(threshold_crossing_time)
    max_cell_voltage_timer = Timer(threshold_crossing_time)

    min_cell_temp_timer = Timer(threshold_crossing_time)
    max_cell_temp_timer = Timer(threshold_crossing_time)

    battery_voltage_timer = Timer(threshold_crossing_time)
    battery_current_timer = Timer(threshold_crossing_time)

    battery_status_1_timer = Timer(threshold_crossing_time)
    battery_status_2_timer = Timer(threshold_crossing_time)
    battery_status_3_timer = Timer(threshold_crossing_time)
    battery_status_4_timer = Timer(threshold_crossing_time)
    battery_status_5_timer = Timer(threshold_crossing_time)
    battery_status_6_timer = Timer(threshold_crossing_time)
    battery_status_7_timer = Timer(threshold_crossing_time)
    battery_status_8_timer = Timer(threshold_crossing_time)



    while rclpy.ok():
        rclpy.spin_once(battery_node)

        # Peripheral Status 
        # Assume we have DPSTs for MPPTs, Triple Pole Contactor from MC to Motor
        peripherals = [
            0,0,0,0,0,0,0,0, #8x MPPT
            0,0,0,0, # MPPT Precharge
            0,0,0,0,  # MC Precharge
            0,         # Motor Contactor
            0, #Battery Contactor
                       ]

        # Parsing the Data
        control_data_list = []

        # CMU 1 Data
        cmu_1_pcb_temp = battery_node.battery_sub_data[0]*10
        cmu_1_cell_temp = battery_node.battery_sub_data[1]
        cmu_1_cell_0_voltage = battery_node.battery_sub_data[2]
        cmu_1_cell_1_voltage = battery_node.battery_sub_data[3]
        cmu_1_cell_2_voltage = battery_node.battery_sub_data[4]
        cmu_1_cell_3_voltage = battery_node.battery_sub_data[5]
        cmu_1_cell_4_voltage = battery_node.battery_sub_data[6]
        cmu_1_cell_5_voltage = battery_node.battery_sub_data[7]
        cmu_1_cell_6_voltage = battery_node.battery_sub_data[8]
        cmu_1_cell_7_voltage = battery_node.battery_sub_data[9]

        # CMU 2 Data
        cmu_2_pcb_temp = battery_node.battery_sub_data[10]
        cmu_2_cell_temp = battery_node.battery_sub_data[11]
        cmu_2_cell_0_voltage = battery_node.battery_sub_data[12]
        cmu_2_cell_1_voltage = battery_node.battery_sub_data[13]
        cmu_2_cell_2_voltage = battery_node.battery_sub_data[14]
        cmu_2_cell_3_voltage = battery_node.battery_sub_data[15]
        cmu_2_cell_4_voltage = battery_node.battery_sub_data[16]
        cmu_2_cell_5_voltage = battery_node.battery_sub_data[17]
        cmu_2_cell_6_voltage = battery_node.battery_sub_data[18]
        cmu_2_cell_7_voltage = battery_node.battery_sub_data[19]

        # CMU 3 Data
        cmu_3_pcb_temp = battery_node.battery_sub_data[20]
        cmu_3_cell_temp = battery_node.battery_sub_data[21]
        cmu_3_cell_0_voltage = battery_node.battery_sub_data[22]
        cmu_3_cell_1_voltage = battery_node.battery_sub_data[23]
        cmu_3_cell_2_voltage = battery_node.battery_sub_data[24]
        cmu_3_cell_3_voltage = battery_node.battery_sub_data[25]
        cmu_3_cell_4_voltage = battery_node.battery_sub_data[26]
        cmu_3_cell_5_voltage = battery_node.battery_sub_data[27]
        cmu_3_cell_6_voltage = battery_node.battery_sub_data[28]
        cmu_3_cell_7_voltage = battery_node.battery_sub_data[29]

        # CMU 4 Data
        cmu_4_pcb_temp = battery_node.battery_sub_data[30]
        cmu_4_cell_temp = battery_node.battery_sub_data[31]
        cmu_4_cell_0_voltage = battery_node.battery_sub_data[32]
        cmu_4_cell_1_voltage = battery_node.battery_sub_data[33]
        cmu_4_cell_2_voltage = battery_node.battery_sub_data[34]
        cmu_4_cell_3_voltage = battery_node.battery_sub_data[35]
        cmu_4_cell_4_voltage = battery_node.battery_sub_data[36]
        cmu_4_cell_5_voltage = battery_node.battery_sub_data[37]
        cmu_4_cell_6_voltage = battery_node.battery_sub_data[38]
        cmu_4_cell_7_voltage = battery_node.battery_sub_data[39]

        # CMU 5 Data
        cmu_5_pcb_temp = battery_node.battery_sub_data[40]
        cmu_5_cell_temp = battery_node.battery_sub_data[41]
        cmu_5_cell_0_voltage = battery_node.battery_sub_data[42]
        cmu_5_cell_1_voltage = battery_node.battery_sub_data[43]
        cmu_5_cell_2_voltage = battery_node.battery_sub_data[44]
        cmu_5_cell_3_voltage = battery_node.battery_sub_data[45]
        cmu_5_cell_4_voltage = battery_node.battery_sub_data[46]
        cmu_5_cell_5_voltage = battery_node.battery_sub_data[47]
        cmu_5_cell_6_voltage = battery_node.battery_sub_data[48]
        cmu_5_cell_7_voltage = battery_node.battery_sub_data[49]

        # SOC Data
        battery_soc_ah = battery_node.battery_sub_data[50]
        battery_soc_percent = battery_node.battery_sub_data[51]
    
        # Cell Voltage Data
        min_cell_voltage = battery_node.battery_sub_data[52]
        max_cell_voltage = battery_node.battery_sub_data[53]

        # Cell Temperature Data
        min_cell_temp = battery_node.battery_sub_data[54]
        max_cell_temp = battery_node.battery_sub_data[55]

        # Battery Data
        battery_voltage = battery_node.battery_sub_data[56]
        battery_current = battery_node.battery_sub_data[57]

        # Battery Status
        battery_status = battery_node.battery_sub_data[58]

        # CMU PCB Temperature Error - Temperature is in 1/10th of C
        #print(cmu_1_pcb_temp)
        if cmu_1_pcb_temp > 40 and cmu_1_pcb_temp < 50:
            if cmu_1_pcb_temp_timer.start_time is None:
                cmu_1_pcb_temp_timer.start()

            if cmu_1_pcb_temp_timer.elapsed()>=threshold_crossing_time:
                control_data_list.append(1)
        else: 
            cmu_1_pcb_temp_timer.reset()

        if cmu_2_pcb_temp*10 > 40 and cmu_2_pcb_temp*10 < 50:
            if cmu_2_pcb_temp_timer.start_time is None:
                cmu_2_pcb_temp_timer.start()

            if cmu_2_pcb_temp_timer.elapsed()>=threshold_crossing_time:
                control_data_list.append(2)
        else:
            cmu_2_pcb_temp_timer.reset()

        # if cmu_3_pcb_temp*10 > 40 and cmu_3_pcb_temp*10 < 50:
        #     if cmu_3_pcb_temp_timer.start_time is None:
        #         cmu_3_pcb_temp_timer.start()
            
        #     if cmu_3_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 3

        # else:
        #     cmu_3_pcb_temp_timer.reset()

        # if cmu_4_pcb_temp*10 > 40 and cmu_4_pcb_temp*10 < 50:
        #     if cmu_4_pcb_temp_timer.start_time is None:
        #         cmu_4_pcb_temp_timer.start()
            
        #     if cmu_4_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 4

        # else:
        #     cmu_4_pcb_temp_timer.reset()

        # if cmu_5_pcb_temp*10 > 40 and cmu_5_pcb_temp*10 < 50:
        #     if cmu_5_pcb_temp_timer.start_time is None:
        #         cmu_5_pcb_temp_timer.start()
            
        #     if cmu_5_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 5

        # else:
        #     cmu_5_pcb_temp_timer.reset()

        # if cmu_1_pcb_temp*10 > 50:
        #     if cmu_1_pcb_temp_timer.start_time is None:
        #         cmu_1_pcb_temp_timer.start()
            
        #     if cmu_1_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 6

        # else:
        #     cmu_1_pcb_temp_timer.reset()

        # if cmu_2_pcb_temp*10 > 50:
        #     if cmu_2_pcb_temp_timer.start_time is None:
        #         cmu_2_pcb_temp_timer.start()

        #     if cmu_2_pcb_temp_timer.elapsed()>=threshold_crossing_time:

        #         battery_node.control_pub_data = 7
        # else:
        #     cmu_2_pcb_temp_timer.reset()

        # if cmu_3_pcb_temp*10 > 50:
        #     if cmu_3_pcb_temp_timer.start_time is None:
        #         cmu_3_pcb_temp_timer.start()

        #     if cmu_3_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 8

        # else:
        #     cmu_3_pcb_temp_timer.reset()

        # if cmu_4_pcb_temp*10 > 50:
        #     if cmu_4_pcb_temp_timer.start_time is None:
        #         cmu_4_pcb_temp_timer.start()
            
        #     if cmu_4_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 9

        # else:
        #     cmu_4_pcb_temp_timer.reset()

        # if cmu_5_pcb_temp*10 > 50:
        #     if cmu_5_pcb_temp_timer.start_time is None:
        #         cmu_5_pcb_temp_timer.start()

        #     if cmu_5_pcb_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 10

        # else:
        #     cmu_5_pcb_temp_timer.reset()

        # # CMU Cell Temperature Error - Temperature is in 1/10th of C
 
        # if cmu_1_cell_temp*10 > 40 and cmu_1_cell_temp*10 < 50:
        #     if cmu_1_cell_temp_timer.start_time is None:
        #         cmu_1_cell_temp_timer.start()

        #     if cmu_1_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 11 
        # else:
        #     cmu_1_cell_temp_timer.reset()

        # if cmu_2_cell_temp*10 > 40 and cmu_2_cell_temp*10 < 50:
        #     if cmu_2_cell_temp_timer.start_time is None:
        #         cmu_2_cell_temp_timer.start()

        #     if cmu_2_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 12

        # else:
        #     cmu_2_cell_temp_timer.reset()

        # if cmu_3_cell_temp*10 > 40 and cmu_3_cell_temp*10 < 50:
        #     if cmu_3_cell_temp_timer.start_time is None:
        #         cmu_3_cell_temp_timer.start()
            
        #     if cmu_3_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 13 

        # else:
        #     cmu_3_cell_temp_timer.reset()

        # if cmu_4_cell_temp*10 > 40 and cmu_4_cell_temp*10 < 50:
        #     if cmu_4_cell_temp_timer.start_time is None:
        #         cmu_4_cell_temp_timer.start()

        #     if cmu_4_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 14

        # else:
        #     cmu_4_cell_temp_timer.reset()

        # if cmu_5_cell_temp*10 > 40 and cmu_5_cell_temp*10 < 50:
        #     if cmu_5_cell_temp_timer.start_time is None:
        #         cmu_5_cell_temp_timer.start()
            
        #     if cmu_5_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 15

        # else:
        #     cmu_5_cell_temp_timer.reset()

        # if cmu_1_cell_temp*10 > 50:
        #     if cmu_1_cell_temp_timer.start_time is None:
        #         cmu_1_cell_temp_timer.start()

        #     if cmu_1_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 16

        # else:
        #     cmu_1_cell_temp_timer.reset()
        # if cmu_2_cell_temp*10 > 50:
        #     if cmu_2_cell_temp_timer.start_time is None:
        #         cmu_2_cell_temp_timer.start()
            
        #     if cmu_2_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 17

        # else:
        #     cmu_2_cell_temp_timer.reset()

        # if cmu_3_cell_temp*10 > 50:
        #     if cmu_3_cell_temp_timer.start_time is None:
        #         cmu_3_cell_temp_timer.start()

        #     if cmu_3_cell_temp_timer.elapsed()>=threshold_crossing_time:

        #         battery_node.control_pub_data = 18

        # else:
        #     cmu_3_cell_temp_timer.reset()

        # if cmu_4_cell_temp*10 > 50:
        #     if cmu_4_cell_temp_timer.start_time is None:
        #         cmu_4_cell_temp_timer.start()

        #     if cmu_4_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 19

        # else:
        #     cmu_4_cell_temp_timer.reset()
        # if cmu_5_cell_temp*10 > 50:
        #     if cmu_5_cell_temp_timer.start_time is None:
        #         cmu_5_cell_temp_timer.start()

        #     if cmu_5_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 20

        # else:
        #     cmu_5_cell_temp_timer.reset()

        # # Voltages are in MilliVolts
        # # CMU_1 Cell Undervoltage or Overvoltage Error

        # if cmu_1_cell_0_voltage*1000 < 3:
        #     if cmu_1_cell_0_voltage_timer.start_time is None:
        #         cmu_1_cell_0_voltage_timer.start()
            
        #     if cmu_1_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 21

        # else:
        #     cmu_1_cell_0_voltage_timer.reset()

        # if cmu_1_cell_1_voltage*1000 < 3:
        #     if cmu_1_cell_1_voltage_timer.start_time is None:
        #         cmu_1_cell_1_voltage_timer.start()

        #     if cmu_1_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 22

        # else:
        #     cmu_1_cell_1_voltage_timer.reset()

        # if cmu_1_cell_2_voltage*1000 < 3:
        #     if cmu_1_cell_2_voltage_timer.start_time is None:
        #         cmu_1_cell_2_voltage_timer.start()
            
        #     if cmu_1_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 23

        # else:
        #     cmu_1_cell_2_voltage_timer.reset()

        # if cmu_1_cell_3_voltage*1000 < 3:
        #     if cmu_1_cell_3_voltage_timer.start_time is None:
        #         cmu_1_cell_3_voltage_timer.start()

        #     if cmu_1_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 24
        
        # else:
        #     cmu_1_cell_3_voltage_timer.reset()

        # if cmu_1_cell_4_voltage*1000 < 3:
        #     if cmu_1_cell_4_voltage_timer.start_time is None:
        #         cmu_1_cell_4_voltage_timer.start()

        #     if cmu_1_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 25

        # else:
        #     cmu_1_cell_4_voltage_timer.reset()
            
        # if cmu_1_cell_5_voltage*1000 < 3:
        #     if cmu_1_cell_5_voltage_timer.start_time is None:
        #         cmu_1_cell_5_voltage_timer.start()

        #     if cmu_1_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 26

        # else:
        #     cmu_1_cell_5_voltage_timer.reset()

        # if cmu_1_cell_6_voltage*1000 < 3:
        #     if cmu_1_cell_6_voltage_timer.start_time is None:
        #         cmu_1_cell_6_voltage_timer.start()

        #     if cmu_1_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 27
        
        # else:
        #     cmu_1_cell_6_voltage_timer.reset()

        # if cmu_1_cell_7_voltage*1000 < 3:
        #     if cmu_1_cell_7_voltage_timer.start_time is None:
        #         cmu_1_cell_7_voltage_timer.start()
            
        #     if cmu_1_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 28
        
        # else:
        #     cmu_1_cell_7_voltage_timer.reset()


        # if cmu_1_cell_0_voltage*1000 > 4:
        #     if cmu_1_cell_0_voltage_timer.start_time is None:
        #         cmu_1_cell_0_voltage_timer.start()
            
        #     if cmu_1_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 29

        # else:
        #     cmu_1_cell_0_voltage_timer.reset()

        # if cmu_1_cell_1_voltage*1000 > 4:
        #     if cmu_1_cell_1_voltage_timer.start_time is None:\
        #         cmu_1_cell_1_voltage_timer.start()

        #     if cmu_1_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:       
        #         battery_node.control_pub_data = 30

        # else:
        #     cmu_1_cell_1_voltage_timer.reset()

        # if cmu_1_cell_2_voltage*1000 > 4:
        #     if cmu_1_cell_2_voltage_timer.start_time is None:
        #         cmu_1_cell_2_voltage_timer.start()

        #     if cmu_1_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 31

        # else:
        #     cmu_1_cell_2_voltage_timer.reset()

        # if cmu_1_cell_3_voltage*1000 > 4:
        #     if cmu_1_cell_3_voltage_timer.start_time is None:
        #         cmu_1_cell_3_voltage_timer.start()

        #     if cmu_1_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 32 
        
        # else:
        #     cmu_1_cell_3_voltage_timer.reset()

        # if cmu_1_cell_4_voltage*1000 > 4:
        #     if cmu_1_cell_4_voltage_timer.start_time is None:
        #         cmu_1_cell_4_voltage_timer.start()

        #     if cmu_1_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 33

        # else:
        #     cmu_1_cell_4_voltage_timer.reset()

        # if cmu_1_cell_5_voltage*1000 > 4:
        #     if cmu_1_cell_5_voltage_timer.start_time is None:
        #         cmu_1_cell_5_voltage_timer.start()

        #     if cmu_1_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 34

        # else:
        #     cmu_1_cell_5_voltage_timer.reset()

        # if cmu_1_cell_6_voltage*1000 > 4:
        #     if cmu_1_cell_6_voltage_timer.start_time is None:
        #         cmu_1_cell_6_voltage_timer.start()

        #     if cmu_1_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 35

        # else:
        #     cmu_1_cell_6_voltage_timer.reset()

        # if cmu_1_cell_7_voltage*1000 > 4:
        #     if cmu_1_cell_7_voltage_timer.start_time is None:
        #         cmu_1_cell_7_voltage_timer.start()

        #     if cmu_1_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 36

        # else:
        #     cmu_1_cell_7_voltage_timer.reset()

        # # CMU_2 Cell Undervoltage or Overvoltage Error

        # if cmu_2_cell_0_voltage*1000 < 3:
        #     if cmu_2_cell_0_voltage_timer.start_time is None:
        #         cmu_2_cell_0_voltage_timer.start()

        #     if cmu_2_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:   
        #         battery_node.control_pub_data = 37

        # else:
        #     cmu_2_cell_0_voltage_timer.reset()

        # if cmu_2_cell_1_voltage*1000 < 3:
        #     if cmu_2_cell_1_voltage_timer.start_time is None:
        #         cmu_2_cell_1_voltage_timer.start()

        #     if cmu_2_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 38

        # else:
        #     cmu_2_cell_1_voltage_timer.reset()

        # if cmu_2_cell_2_voltage*1000 < 3:
        #     if cmu_2_cell_2_voltage_timer.start_time is None:
        #         cmu_2_cell_2_voltage_timer.start()

        #     if cmu_2_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 39

        # else:
        #     cmu_2_cell_2_voltage_timer.reset()

        # if cmu_2_cell_3_voltage*1000 < 3:
        #     if cmu_2_cell_3_voltage_timer.start_time is None:
        #         cmu_2_cell_3_voltage_timer.start()

        #     if cmu_2_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 40

        # else:
        #     cmu_2_cell_3_voltage_timer.reset()

        # if cmu_2_cell_4_voltage*1000 < 3:
        #     if cmu_2_cell_4_voltage_timer.start_time is None:
        #         cmu_2_cell_4_voltage_timer.start()

        #     if cmu_2_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 41

        # else:
        #     cmu_2_cell_4_voltage_timer.reset()

        # if cmu_2_cell_5_voltage*1000 < 3:
        #     if cmu_2_cell_5_voltage_timer.start_time is None:
        #         cmu_2_cell_5_voltage_timer.start()

        #     if cmu_2_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 42

        # else:
        #     cmu_2_cell_5_voltage_timer.reset()

        # if cmu_2_cell_6_voltage*1000 < 3:
        #     if cmu_2_cell_6_voltage_timer.start_time is None:
        #         cmu_2_cell_6_voltage_timer.start()

        #     if cmu_2_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 43

        # else:
        #     cmu_2_cell_6_voltage_timer.reset()

        # if cmu_2_cell_7_voltage*1000 < 3:
        #     if cmu_2_cell_7_voltage_timer.start_time is None:
        #         cmu_2_cell_7_voltage_timer.start()

        #     if cmu_2_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 44

        # else:
        #     cmu_2_cell_7_voltage_timer.reset()
                

        # if cmu_2_cell_0_voltage*1000 > 4:
        #     if cmu_2_cell_0_voltage_timer.start_time is None:
        #         cmu_2_cell_0_voltage_timer.start()
            
        #     if cmu_2_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 45

        # else:
        #     cmu_2_cell_0_voltage_timer.reset()

        # if cmu_2_cell_1_voltage*1000 > 4:
        #     if cmu_2_cell_1_voltage_timer.start_time is None:
        #         cmu_2_cell_1_voltage_timer.start()

        #     if cmu_2_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 46
        
        # else:
        #     cmu_2_cell_1_voltage_timer.reset()
        # if cmu_2_cell_2_voltage*1000 > 4:
        #     if cmu_2_cell_2_voltage_timer.start_time is None:
        #         cmu_2_cell_2_voltage_timer.start()

        #     if cmu_2_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 47

        # else:
        #     cmu_2_cell_2_voltage_timer.reset()

        # if cmu_2_cell_3_voltage*1000 > 4:
        #     if cmu_2_cell_3_voltage_timer.start_time is None:
        #         cmu_2_cell_3_voltage_timer.start()

        #     if cmu_2_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 48

        # else:
        #     cmu_2_cell_3_voltage_timer.reset()

        # if cmu_2_cell_4_voltage*1000 > 4:
        #     if cmu_2_cell_4_voltage_timer.start_time is None:
        #         cmu_2_cell_4_voltage_timer.start()

        #     if cmu_2_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #          battery_node.control_pub_data = 49

        # else:
        #     cmu_2_cell_4_voltage_timer.reset()

        # if cmu_2_cell_5_voltage*1000 > 4:
        #     if cmu_2_cell_5_voltage_timer.start_time is None:
        #         cmu_2_cell_5_voltage_timer.start()

        #     if cmu_2_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 50

        # else:
        #     cmu_2_cell_5_voltage_timer.reset()
        
        # if cmu_2_cell_6_voltage*1000 > 4:
        #     if cmu_2_cell_6_voltage_timer.start_time is None:
        #         cmu_2_cell_6_voltage_timer.start()

        #     if cmu_2_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 51

        # else:
        #     cmu_2_cell_6_voltage_timer.reset()

        # if cmu_2_cell_7_voltage*1000 > 4:
        #     if cmu_2_cell_7_voltage_timer.start_time is None:
        #         cmu_2_cell_7_voltage_timer.start()

        #     if cmu_2_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 52

        # else:
        #     cmu_2_cell_7_voltage_timer.reset()


        # # CMU_3 Cell Undervoltage or Overvoltage Error
        # if cmu_3_cell_0_voltage*1000 < 3:
        #     if cmu_3_cell_0_voltage_timer.start_time is None:
        #         cmu_3_cell_0_voltage_timer.start()

        #     if cmu_3_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 53

        # else:
        #     cmu_3_cell_0_voltage_timer.reset()

        # if cmu_3_cell_1_voltage*1000 < 3:
        #     if cmu_3_cell_1_voltage_timer.start_time is None:
        #         cmu_3_cell_1_voltage_timer.start()

        #     if cmu_3_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 54 
        # if cmu_3_cell_2_voltage*1000 < 3:
        #     if cmu_3_cell_2_voltage_timer.start_time is None:
        #         cmu_3_cell_2_voltage_timer.start()

        #     if cmu_3_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 55

        # else:
        #     cmu_3_cell_2_voltage_timer.reset()

        # if cmu_3_cell_3_voltage*1000 < 3:
        #     if cmu_3_cell_3_voltage_timer.start_time is None:
        #         cmu_3_cell_3_voltage_timer.start()

        #     if cmu_3_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 56

        # else:
        #     cmu_3_cell_3_voltage_timer.reset()

        # if cmu_3_cell_4_voltage*1000 < 3:
        #     if cmu_3_cell_4_voltage_timer.start_time is None:
        #         cmu_3_cell_4_voltage_timer.start()

        #     if cmu_3_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 57
        
        # else:
        #     cmu_3_cell_4_voltage_timer.reset()

        # if cmu_3_cell_5_voltage*1000 < 3:
        #     if cmu_3_cell_5_voltage_timer.start_time is None:
        #         cmu_3_cell_5_voltage_timer.start()

        #     if cmu_3_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 58

        # else:
        #     cmu_3_cell_5_voltage_timer.reset()

        # if cmu_3_cell_6_voltage*1000 < 3:
        #     if cmu_3_cell_6_voltage_timer.start_time is None:
        #         cmu_3_cell_6_voltage_timer.start()

        #     if cmu_3_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 59

        # else:
        #     cmu_3_cell_6_voltage_timer.reset()

        # if cmu_3_cell_7_voltage*1000 < 3:
        #     if cmu_3_cell_7_voltage_timer.start_time is None:
        #         cmu_3_cell_7_voltage_timer.start()

        #     if cmu_3_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 60

        # else:
        #     cmu_3_cell_7_voltage_timer.reset()


        # if cmu_3_cell_0_voltage*1000 > 4:
        #     if cmu_3_cell_0_voltage_timer.start_time is None:
        #         cmu_3_cell_0_voltage_timer.start()

        #     if cmu_3_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 61

        # else:
        #     cmu_3_cell_0_voltage_timer.reset()

        # if cmu_3_cell_1_voltage*1000 > 4:
        #     if cmu_3_cell_1_voltage_timer.start_time is None:
        #         cmu_3_cell_1_voltage_timer.start()

        #     if cmu_3_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 62

        # else:
        #     cmu_3_cell_1_voltage_timer.reset()

        # if cmu_3_cell_2_voltage*1000 > 4:
        #     if cmu_3_cell_2_voltage_timer.start_time is None:
        #         cmu_3_cell_2_voltage_timer.start()

        #     if cmu_3_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 63

        # else:
        #     cmu_3_cell_2_voltage_timer.reset()

        # if cmu_3_cell_3_voltage*1000 > 4:
        #     if cmu_3_cell_3_voltage_timer.start_time is None:
        #         cmu_3_cell_3_voltage_timer.start()
            
        #     if cmu_3_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 64

        # else:
        #     cmu_3_cell_3_voltage_timer.reset()

        # if cmu_3_cell_4_voltage*1000 > 4:
        #     if cmu_3_cell_4_voltage_timer.start_time is None:
        #         cmu_3_cell_4_voltage_timer.start()

        #     if cmu_3_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 65

        # else:
        #     cmu_3_cell_4_voltage_timer.reset()

        # if cmu_3_cell_5_voltage*1000 > 4:
        #     if cmu_3_cell_5_voltage_timer.start_time is None:
        #         cmu_3_cell_5_voltage_timer.start()

        #     if cmu_3_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 66

        # else:
        #     cmu_3_cell_5_voltage_timer.reset()

        # if cmu_3_cell_6_voltage*1000 > 4:
        #     if cmu_3_cell_6_voltage_timer.start_time is None:
        #         cmu_3_cell_6_voltage_timer.start()

        #     if cmu_3_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 67

        # else:
        #     cmu_3_cell_6_voltage_timer.reset()

        # if cmu_3_cell_7_voltage*1000 > 4:
        #     if cmu_3_cell_7_voltage_timer.start_time is None:
        #         cmu_3_cell_7_voltage_timer.start()

        #     if cmu_3_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 68

        # else:
        #     cmu_3_cell_7_voltage_timer.reset()


        # # CMU_4 Cell Undervoltage or Overvoltage Error
        # if cmu_4_cell_0_voltage*1000 < 3:
        #     if cmu_4_cell_0_voltage_timer.start_time is None:
        #         cmu_4_cell_0_voltage_timer.start()

        #     if cmu_4_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 69

        # else:
        #     cmu_4_cell_0_voltage_timer.reset()

        # if cmu_4_cell_1_voltage*1000 < 3:
        #     if cmu_4_cell_1_voltage_timer.start_time is None:
        #         cmu_4_cell_1_voltage_timer.start()

        #     if cmu_4_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 70

        # else:
        #     cmu_4_cell_1_voltage_timer.reset()

        # if cmu_4_cell_2_voltage*1000 < 3:
        #     if cmu_4_cell_2_voltage_timer.start_time is None:
        #         cmu_4_cell_2_voltage_timer.start()
            
        #     if cmu_4_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 71

        # else:
        #     cmu_4_cell_2_voltage_timer.reset()

        # if cmu_4_cell_3_voltage*1000 < 3:
        #     if cmu_4_cell_3_voltage_timer.start_time is None:
        #         cmu_4_cell_3_voltage_timer.start()
            
        #     if cmu_4_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 72

        # else:
        #     cmu_4_cell_3_voltage_timer.reset()

        # if cmu_4_cell_4_voltage*1000 < 3:
        #     if cmu_4_cell_4_voltage_timer.start_time is None:
        #         cmu_4_cell_4_voltage_timer.start()

        #     if cmu_4_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 73

        # else:
        #     cmu_4_cell_4_voltage_timer.reset()

        # if cmu_4_cell_5_voltage*1000 < 3:
        #     if cmu_4_cell_5_voltage_timer.start_time is None:
        #         cmu_4_cell_5_voltage_timer.start()

        #     if cmu_4_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 74

        # else:
        #     cmu_4_cell_5_voltage_timer.reset()

        # if cmu_4_cell_6_voltage*1000 < 3:
        #     if cmu_4_cell_6_voltage_timer.start_time is None:
        #         cmu_4_cell_6_voltage_timer.start()

        #     if cmu_4_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 75

        # else:
        #     cmu_4_cell_6_voltage_timer.reset()

        # if cmu_4_cell_7_voltage*1000 < 3:
        #     if cmu_4_cell_7_voltage_timer.start_time is None:
        #         cmu_4_cell_7_voltage_timer.start()

        #     if cmu_4_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 76

        # if cmu_4_cell_0_voltage*1000 > 4:
        #     if cmu_4_cell_0_voltage_timer.start_time is None:
        #         cmu_4_cell_0_voltage_timer.start()
            
        #     if cmu_4_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 77

        # else:
        #     cmu_4_cell_0_voltage_timer.reset()

        # if cmu_4_cell_1_voltage*1000 > 4:
        #     if cmu_4_cell_1_voltage_timer.start_time is None:
        #         cmu_4_cell_1_voltage_timer.start()
            
        #     if cmu_4_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 78

        # else:
        #     cmu_4_cell_1_voltage_timer.reset()

        # if cmu_4_cell_2_voltage*1000 > 4:
        #     if cmu_4_cell_2_voltage_timer.start_time is None:
        #         cmu_4_cell_2_voltage_timer.start()

        #     if cmu_4_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 79

        # else:
        #     cmu_4_cell_2_voltage_timer.reset()

        # if cmu_4_cell_3_voltage*1000 > 4:
        #     if cmu_4_cell_3_voltage_timer.start_time is None:
        #         cmu_4_cell_3_voltage_timer.start()

        #     if cmu_4_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 80

        # else:
        #     cmu_4_cell_3_voltage_timer.reset()

        # if cmu_4_cell_4_voltage*1000 > 4:
        #     if cmu_4_cell_4_voltage_timer.start_time is None:
        #         cmu_4_cell_4_voltage_timer.start()

        #     if cmu_4_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 81

        # else:
        #     cmu_4_cell_4_voltage_timer.reset()

        # if cmu_4_cell_5_voltage*1000 > 4:
        #     if cmu_4_cell_5_voltage_timer.start_time is None:
        #         cmu_4_cell_5_voltage_timer.start()

        #     if cmu_4_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 82

        # else:
        #     cmu_4_cell_5_voltage_timer.reset()

        # if cmu_4_cell_6_voltage*1000 > 4:
        #     if cmu_4_cell_6_voltage_timer.start_time is None:
        #         cmu_4_cell_6_voltage_timer.start()

        #     if cmu_4_cell_6_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 83

        # else:
        #     cmu_4_cell_6_voltage_timer.reset()

        # if cmu_4_cell_7_voltage*1000 > 4:
        #     if cmu_4_cell_7_voltage_timer.start_time is None:
        #         cmu_4_cell_7_voltage_timer.start()

        #     if cmu_4_cell_7_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 84

        # else:
        #     cmu_4_cell_7_voltage_timer.reset()

        # # CMU_5 Cell Undervoltage or Overvoltage Error - CMU 5 only has 6 cells
        # if cmu_5_cell_0_voltage*1000 < 3:
        #     if cmu_5_cell_0_voltage_timer.start_time is None:
        #         cmu_5_cell_0_voltage_timer.start()

        #     if cmu_5_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 85

        # else:
        #     cmu_5_cell_0_voltage_timer.reset()

        # if cmu_5_cell_1_voltage*1000 < 3:
        #     if cmu_5_cell_1_voltage_timer.start_time is None:
        #         cmu_5_cell_1_voltage_timer.start()

        #     if cmu_5_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 86

        # else:
        #     cmu_5_cell_1_voltage_timer.reset()

        # if cmu_5_cell_2_voltage*1000 < 3:
        #     if cmu_5_cell_2_voltage_timer.start_time is None:
        #         cmu_5_cell_2_voltage_timer.start()

        #     if cmu_5_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 87

        # else:
        #     cmu_5_cell_2_voltage_timer.reset()

        # if cmu_5_cell_3_voltage*1000 < 3:
        #     if cmu_5_cell_3_voltage_timer.start_time is None:
        #         cmu_5_cell_3_voltage_timer.start()

        #     if cmu_5_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 88

        # else:
        #     cmu_5_cell_3_voltage_timer.reset()

        # if cmu_5_cell_4_voltage*1000 < 3:
        #     if cmu_5_cell_4_voltage_timer.start_time is None:
        #         cmu_5_cell_4_voltage_timer.start()

        #     if cmu_5_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 89
        # else:
        #     cmu_5_cell_4_voltage_timer.reset()
            
        # if cmu_5_cell_5_voltage*1000 < 3:
        #     if cmu_5_cell_5_voltage_timer.start_time is None:
        #         cmu_5_cell_5_voltage_timer.start()

        #     if cmu_5_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 90

        # else:
        #     cmu_5_cell_5_voltage_timer.reset()

        # if cmu_5_cell_0_voltage*1000 > 4:
        #     if cmu_5_cell_0_voltage_timer.start_time is None:
        #         cmu_5_cell_0_voltage_timer.start()

        #     if cmu_5_cell_0_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 91

        # else:
        #     cmu_5_cell_0_voltage_timer.reset()

        # if cmu_5_cell_1_voltage*1000 > 4:
        #     if cmu_5_cell_1_voltage_timer.start_time is None:
        #         cmu_5_cell_1_voltage_timer.start()

        #     if cmu_5_cell_1_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 92

        # else:
        #     cmu_5_cell_1_voltage_timer.reset()
        
        # if cmu_5_cell_2_voltage*1000 > 4:
        #     if cmu_5_cell_2_voltage_timer.start_time is None:
        #         cmu_5_cell_2_voltage_timer.start()

        #     if cmu_5_cell_2_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 93

        # else:
        #     cmu_5_cell_2_voltage_timer.reset()

        # if cmu_5_cell_3_voltage*1000 > 4:
        #     if cmu_5_cell_3_voltage_timer.start_time is None:
        #         cmu_5_cell_3_voltage_timer.start()

        #     if cmu_5_cell_3_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 94

        # else:
        #     cmu_5_cell_3_voltage_timer.reset()

        # if cmu_5_cell_4_voltage*1000 > 4:
        #     if cmu_5_cell_4_voltage_timer.start_time is None:
        #         cmu_5_cell_4_voltage_timer.start()

        #     if cmu_5_cell_4_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 95

        # else:
        #     cmu_5_cell_4_voltage_timer.reset()

        # if cmu_5_cell_5_voltage*1000 > 4:
        #     if cmu_5_cell_5_voltage_timer.start_time is None:
        #         cmu_5_cell_5_voltage_timer.start()

        #     if cmu_5_cell_5_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 96

        # else:
        #     cmu_5_cell_5_voltage_timer.reset()

            
        # if battery_soc_ah > 114: # 114 Ah i g
        #     if battery_soc_ah_timer.start_time is None:
        #         battery_soc_ah_timer.start()

        #     if battery_soc_ah_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 97

        # else:
        #     battery_soc_ah_timer.reset()

        # if battery_soc_percent < 10:
        #     if battery_soc_percent_timer.start_time is None:
        #         battery_soc_percent_timer.start()

        #     if battery_soc_percent_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 98

        # else:
        #     battery_soc_percent_timer.reset()

        # if battery_soc_percent > 90:
        #     if battery_soc_percent_timer.start_time is None:
        #         battery_soc_percent_timer.start()

        #     if battery_soc_percent_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 99

        # else:
        #     battery_soc_percent_timer.reset()

        # if min_cell_voltage*1000 < 3:
        #     if min_cell_voltage_timer.start_time is None:
        #         min_cell_voltage_timer.start()

        #     if min_cell_voltage_timer.elapsed()>=threshold_crossing_time:
        #         print(min_cell_voltage_timer.elapsed())
        #         battery_node.control_pub_data = 100

        # else:
        #     min_cell_voltage_timer.reset()

        # if max_cell_voltage*1000 > 4:
        #     if max_cell_voltage_timer.start_time is None:
        #         max_cell_voltage_timer.start()

        #     if max_cell_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 101

        # else:
        #     max_cell_voltage_timer.reset()

        # if min_cell_temp*10 > 40 and min_cell_temp*10 < 50:
        #     if min_cell_temp_timer.start_time is None:
        #         min_cell_temp_timer.start()

        #     if min_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 102

        # else:
        #     min_cell_temp_timer.reset()

        # if min_cell_temp*10 > 50:
        #     if min_cell_temp_timer.start_time is None:
        #         min_cell_temp_timer.start()

        #     if min_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 103

        # else:
        #     min_cell_temp_timer.reset()

        # if max_cell_temp*10 > 40 and max_cell_temp*10 < 50:
        #     if max_cell_temp_timer.start_time is None:
        #         max_cell_temp_timer.start()

        #     if max_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 104

        # else:
        #     max_cell_temp_timer.reset()

        # if max_cell_temp*10 > 50:
        #     if max_cell_temp_timer.start_time is None:
        #         max_cell_temp_timer.start()

        #     if max_cell_temp_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 105 

        # else:
        #     max_cell_temp_timer.reset()

        # if battery_voltage*1000 > 160:
        #     if battery_voltage_timer.start_time is None:
        #         battery_voltage_timer.start()

        #     if battery_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 106

        # else:
        #     battery_voltage_timer.reset()

        # if battery_voltage*1000 < 120:
        #     if battery_voltage_timer.start_time is None:
        #         battery_voltage_timer.start()

        #     if battery_voltage_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 107

        # else:
        #     battery_voltage_timer.reset()

        # if battery_current*1000 > 30:
        #     if battery_current_timer.start_time is None:
        #         battery_current_timer.start()
            
        #     if battery_current_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 108

        # else:
        #     battery_current_timer.reset()

        # if battery_status == 0x01:
        #     if battery_status_1_timer.start_time is None:
        #         battery_status_1_timer.start()

        #     if battery_status_1_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 109

        # else:
        #     battery_status_1_timer.reset()

        # if battery_status == 0x02:
        #     if battery_status_2_timer.start_time is None:
        #         battery_status_2_timer.start()

        #     if battery_status_2_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 110

        # else:
        #     battery_status_2_timer.reset()
        # if battery_status == 0x04:
        #     if battery_status_3_timer.start_time is None:
        #         battery_status_3_timer.start()

        #     if battery_status_3_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 111

        # else:
        #     battery_status_3_timer.reset()

        # if  battery_status == 0x08:
        #     if battery_status_4_timer.start_time is None:
        #         battery_status_4_timer.start()

        #     if battery_status_4_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 112

        # else:
        #     battery_status_4_timer.reset()

        # if  battery_status == 0x10:
        #     if battery_status_5_timer.start_time is None:
        #         battery_status_5_timer.start()

        #     if battery_status_5_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 113

        # else:
        #     battery_status_5_timer.reset()

        # if  battery_status == 0x20:
        #     if battery_status_6_timer.start_time is None:
        #         battery_status_6_timer.start()

        #     if battery_status_6_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 114

        # else:
        #     battery_status_6_timer.reset()

        # if  battery_status == 0x40:
        #     if battery_status_7_timer.start_time is None:
        #         battery_status_7_timer.start()

        #     if battery_status_7_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 115

        # else:
        #     battery_status_7_timer.reset()

        # if  battery_status == 0x80:
        #     if battery_status_8_timer.start_time is None:
        #         battery_status_8_timer.start()

        #     if battery_status_8_timer.elapsed()>=threshold_crossing_time:
        #         battery_node.control_pub_data = 116

        # else:
        #     battery_status_8_timer.reset()

        battery_node.control_pub_data = control_data_list


    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()