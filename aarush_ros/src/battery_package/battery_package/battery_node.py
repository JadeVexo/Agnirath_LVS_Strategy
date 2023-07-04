import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


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

        # CMU 1 Data
        cmu_1_pcb_temp = battery_node.battery_sub_data[0]
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

        cmu_pcb_temps = [cmu_1_pcb_temp,cmu_2_pcb_temp,cmu_3_pcb_temp,cmu_4_pcb_temp,cmu_5_pcb_temp]
        cmu_cell_temps = [cmu_1_cell_temp, cmu_2_cell_temp, cmu_3_cell_temp, cmu_4_cell_temp, cmu_5_cell_temp]
        cmu_1_cell_voltage = [cmu_1_cell_0_voltage,cmu_1_cell_1_voltage,cmu_1_cell_2_voltage,cmu_1_cell_3_voltage,cmu_1_cell_4_voltage,cmu_1_cell_5_voltage,cmu_1_cell_6_voltage,cmu_1_cell_7_voltage]
        cmu_2_cell_voltage = [cmu_2_cell_0_voltage,cmu_2_cell_1_voltage,cmu_2_cell_2_voltage,cmu_2_cell_3_voltage,cmu_2_cell_4_voltage,cmu_2_cell_5_voltage,cmu_2_cell_6_voltage,cmu_2_cell_7_voltage]
        cmu_3_cell_voltage = [cmu_3_cell_0_voltage,cmu_3_cell_1_voltage,cmu_3_cell_2_voltage,cmu_3_cell_3_voltage,cmu_3_cell_4_voltage,cmu_3_cell_5_voltage,cmu_3_cell_6_voltage,cmu_3_cell_7_voltage]
        cmu_4_cell_voltage = [cmu_4_cell_0_voltage,cmu_4_cell_1_voltage,cmu_4_cell_2_voltage,cmu_4_cell_3_voltage,cmu_4_cell_4_voltage,cmu_4_cell_5_voltage,cmu_4_cell_6_voltage,cmu_4_cell_7_voltage]
        cmu_5_cell_voltage = [cmu_5_cell_0_voltage,cmu_5_cell_1_voltage,cmu_5_cell_2_voltage,cmu_5_cell_3_voltage,cmu_5_cell_4_voltage,cmu_5_cell_5_voltage,cmu_5_cell_6_voltage,cmu_5_cell_7_voltage]
        
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
        for index in range(len(cmu_pcb_temps)):
            if cmu_pcb_temps[index]*10 > 40 and cmu_pcb_temps[index]*10 < 50:
                if index == 0:
                    battery_node.control_pub_data = 1
                if index == 1:
                    battery_node.control_pub_data = 2
                if index == 2:
                    battery_node.control_pub_data = 3
                if index == 3:
                    battery_node.control_pub_data = 4
                if index == 4:
                    battery_node.control_pub_data = 5

            if cmu_pcb_temps[index]*10 > 50:
                if index == 0:
                    battery_node.control_pub_data = 6
                if index == 1:
                    battery_node.control_pub_data = 7
                if index == 2:
                    battery_node.control_pub_data = 8
                if index == 3:
                    battery_node.control_pub_data = 9
                if index == 4:
                    battery_node.control_pub_data = 10

        # CMU Cell Temperature Error - Temperature is in 1/10th of C
        for index in range(len(cmu_cell_temps)):
            if cmu_cell_temps[index]*10 > 40 and cmu_cell_temps[index]*10 < 50:
                if index == 0:
                    battery_node.control_pub_data = 11
                if index == 1:
                    battery_node.control_pub_data = 12
                if index == 2:
                    battery_node.control_pub_data = 13
                if index == 3:
                    battery_node.control_pub_data = 14
                if index == 4:
                    battery_node.control_pub_data = 15

            if cmu_cell_temps[index]*10 > 50:
                if index == 0:
                    battery_node.control_pub_data = 16
                if index == 1:
                    battery_node.control_pub_data = 17
                if index == 2:
                    battery_node.control_pub_data = 18
                if index == 3:
                    battery_node.control_pub_data = 19
                if index == 4:
                    battery_node.control_pub_data = 20
        
        # Voltages are in MilliVolts
        # CMU_1 Cell Undervoltage or Overvoltage Error
        for index in range(len(cmu_1_cell_voltage)):
            if cmu_1_cell_voltage[index]*1000 < 3:
                if index == 0:
                    battery_node.control_pub_data = 21
                if index == 1:
                    battery_node.control_pub_data = 22
                if index == 2:
                    battery_node.control_pub_data = 23
                if index == 3:
                    battery_node.control_pub_data = 24
                if index == 4:
                    battery_node.control_pub_data = 25
                if index == 5:
                    battery_node.control_pub_data = 26
                if index == 6:
                    battery_node.control_pub_data = 27
                if index == 7:
                    battery_node.control_pub_data = 28

            if cmu_1_cell_voltage[index]*1000 > 4:
                if index == 0:
                    battery_node.control_pub_data = 29
                if index == 1:
                    battery_node.control_pub_data = 30
                if index == 2:
                    battery_node.control_pub_data = 31
                if index == 3:
                    battery_node.control_pub_data = 32
                if index == 4:
                    battery_node.control_pub_data = 33
                if index == 5:
                    battery_node.control_pub_data = 34
                if index == 6:
                    battery_node.control_pub_data = 35
                if index == 7:
                    battery_node.control_pub_data = 36

        # CMU_2 Cell Undervoltage or Overvoltage Error
        for index in range(len(cmu_2_cell_voltage)):
            if cmu_2_cell_voltage[index]*1000 < 3:
                if index == 0:
                    battery_node.control_pub_data = 37
                if index == 1:
                    battery_node.control_pub_data = 38
                if index == 2:
                    battery_node.control_pub_data = 39
                if index == 3:
                    battery_node.control_pub_data = 40
                if index == 4:
                    battery_node.control_pub_data = 41
                if index == 5:
                    battery_node.control_pub_data = 42
                if index == 6:
                    battery_node.control_pub_data = 43
                if index == 7:
                    battery_node.control_pub_data = 44
                

            if cmu_2_cell_voltage[index]*1000 > 4:
                if index == 0:
                    battery_node.control_pub_data = 45
                if index == 1:
                    battery_node.control_pub_data = 46
                if index == 2:
                    battery_node.control_pub_data = 47
                if index == 3:
                    battery_node.control_pub_data = 48
                if index == 4:
                    battery_node.control_pub_data = 49
                if index == 5:
                    battery_node.control_pub_data = 50
                if index == 6:
                    battery_node.control_pub_data = 51
                if index == 7:
                    battery_node.control_pub_data = 52
                

        
        # CMU_3 Cell Undervoltage or Overvoltage Error
        for index in range(len(cmu_3_cell_voltage)):
            if cmu_3_cell_voltage[index]*1000 < 3:
                if index == 0:
                    battery_node.control_pub_data = 53
                if index == 1:
                    battery_node.control_pub_data = 54
                if index == 2:
                    battery_node.control_pub_data = 55
                if index == 3:
                    battery_node.control_pub_data = 56
                if index == 4:
                    battery_node.control_pub_data = 57
                if index == 5:
                    battery_node.control_pub_data = 58
                if index == 6:
                    battery_node.control_pub_data = 59
                if index == 7:
                    battery_node.control_pub_data = 60

            if cmu_3_cell_voltage[index]*1000 > 4:
                if index == 0:
                    battery_node.control_pub_data = 61
                if index == 1:
                    battery_node.control_pub_data = 62
                if index == 2:
                    battery_node.control_pub_data = 63
                if index == 3:
                    battery_node.control_pub_data = 64
                if index == 4:
                    battery_node.control_pub_data = 65
                if index == 5:
                    battery_node.control_pub_data = 66
                if index == 6:
                    battery_node.control_pub_data = 67
                if index == 7:
                    battery_node.control_pub_data = 68

        
        # CMU_4 Cell Undervoltage or Overvoltage Error
        for index in range(len(cmu_4_cell_voltage)):
            if cmu_4_cell_voltage[index]*1000 < 3:
                if index == 0:
                    battery_node.control_pub_data = 69
                if index == 1:
                    battery_node.control_pub_data = 70
                if index == 2:
                    battery_node.control_pub_data = 71
                if index == 3:
                    battery_node.control_pub_data = 72
                if index == 4:
                    battery_node.control_pub_data = 73
                if index == 5:
                    battery_node.control_pub_data = 74
                if index == 6:
                    battery_node.control_pub_data = 75
                if index == 7:
                    battery_node.control_pub_data = 76

            if cmu_4_cell_voltage[index]*1000 > 4:
                if index == 0:
                    battery_node.control_pub_data = 77
                if index == 1:
                    battery_node.control_pub_data = 78
                if index == 2:
                    battery_node.control_pub_data = 79
                if index == 3:
                    battery_node.control_pub_data = 80
                if index == 4:
                    battery_node.control_pub_data = 81
                if index == 5:
                    battery_node.control_pub_data = 82
                if index == 6:
                    battery_node.control_pub_data = 83
                if index == 7:
                    battery_node.control_pub_data = 84

        # CMU_5 Cell Undervoltage or Overvoltage Error - CMU 5 only has 6 cells
        for index in range(len(cmu_5_cell_voltage)):
            if cmu_5_cell_voltage[index]*1000 < 3:
                if index == 0:
                    battery_node.control_pub_data = 85
                if index == 1:
                    battery_node.control_pub_data = 86
                if index == 2:
                    battery_node.control_pub_data = 87
                if index == 3:
                    battery_node.control_pub_data = 88
                if index == 4:
                    battery_node.control_pub_data = 89
                if index == 5:
                    battery_node.control_pub_data = 90

            if cmu_5_cell_voltage[index]*1000 > 4:
                if index == 0:
                    battery_node.control_pub_data = 91
                if index == 1:
                    battery_node.control_pub_data = 92
                if index == 2:
                    battery_node.control_pub_data = 93
                if index == 3:
                    battery_node.control_pub_data = 94
                if index == 4:
                    battery_node.control_pub_data = 95
                if index == 5:
                    battery_node.control_pub_data = 96

            
        if battery_soc_ah > 114: # 114 Ah i g
            battery_node.control_pub_data = 97

        if battery_soc_percent < 10:
            battery_node.control_pub_data = 98

        if battery_soc_percent > 90:
            battery_node.control_pub_data = 99

        if min_cell_voltage*1000 < 3:
            battery_node.control_pub_data = 100

        if max_cell_voltage*1000 > 4:
            battery_node.control_pub_data = 101

        if min_cell_temp*10 > 40 and min_cell_temp*10 < 50:
            battery_node.control_pub_data = 102

        if min_cell_temp*10 > 50:
            battery_node.control_pub_data = 103

        if max_cell_temp*10 > 40 and max_cell_temp*10 < 50:
            battery_node.control_pub_data = 104

        if max_cell_temp*10 > 50:
            battery_node.control_pub_data = 105 

        if battery_voltage*1000 > 160:
            battery_node.control_pub_data = 106

        if battery_voltage*1000 < 120:
            battery_node.control_pub_data = 107

        if battery_current*1000 > 30:
            battery_node.control_pub_data = 108

        if battery_status == 0x01:
            battery_node.control_pub_data = 109
        if battery_status == 0x02:
            battery_node.control_pub_data = 110
        if battery_status == 0x04:
            battery_node.control_pub_data = 111
        if  battery_status == 0x08:
            battery_node.control_pub_data = 112
        if  battery_status == 0x10:
            battery_node.control_pub_data = 113
        if  battery_status == 0x20:
            battery_node.control_pub_data = 114
        if  battery_status == 0x40:
            battery_node.control_pub_data = 115
        if  battery_status == 0x80:
            battery_node.control_pub_data = 116


    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()