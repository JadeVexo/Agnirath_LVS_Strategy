import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class MPPT_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # MPPT Subscriber
    def init_mppt_subscriber(self, topic):
        self.mppt_subscriber = self.create_subscription(
            rosarray, topic, self.receive_mppt_data, 10
        )
        self.mppt_subscriber  # prevent unused variable warning
        self.mppt_sub_data = None

    def receive_mppt_data(self, msg):
        self.mppt_sub_data = msg.data

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

    mppt_node = MPPT_NODE("mppt_node")
    mppt_node.init_mc_subscriber("mppt_data")
    mppt_node.init_peripheral_subscriber("peripheral_data")
    mppt_node.init_control_data_publisher("control_data", 1)

    while rclpy.ok():
        rclpy.spin_once(mppt_node)

        # Peripheral Status (assumed standard across all control nodes)
        # Assume we have DPSTs for MPPTs, Triple Pole Contactor from MC to Motor
        peripherals = [
            0,0,0,0,0,0,0,0, #8x MPPT
            0,0,0,0, # MPPT Precharge
            0,0,0,0,  # MC Precharge
            0,         # Motor Contactor
            0, #Battery Contactor
                       ]

        # Parsing the Data
        #list combined parameters from all mppts

        mppt_input_volt = [mppt_node.mppt_sub_data[0],mppt_node.mppt_sub_data[14],mppt_node.mppt_sub_data[28],mppt_node.mppt_sub_data[42]]
        mppt_input_current = [mppt_node.mppt_sub_data[1],mppt_node.mppt_sub_data[15],mppt_node.mppt_sub_data[29],mppt_node.mppt_sub_data[43]]
        mppt_output_volt = [mppt_node.mppt_sub_data[2],mppt_node.mppt_sub_data[16],mppt_node.mppt_sub_data[30],mppt_node.mppt_sub_data[44]]
        mppt_output_current = [mppt_node.mppt_sub_data[3],mppt_node.mppt_sub_data[17],mppt_node.mppt_sub_data[31],mppt_node.mppt_sub_data[45]]
        mppt_mosfet_temp = [mppt_node.mppt_sub_data[4],mppt_node.mppt_sub_data[18],mppt_node.mppt_sub_data[32],mppt_node.mppt_sub_data[46]]
        mppt_controller_temp = [mppt_node.mppt_sub_data[5],mppt_node.mppt_sub_data[19],mppt_node.mppt_sub_data[33],mppt_node.mppt_sub_data[47]]
        mppt_12V_aux_supply = [mppt_node.mppt_sub_data[6],mppt_node.mppt_sub_data[20],mppt_node.mppt_sub_data[34],mppt_node.mppt_sub_data[48]]
        mppt_3V_aux_supply = [mppt_node.mppt_sub_data[7],mppt_node.mppt_sub_data[21],mppt_node.mppt_sub_data[35],mppt_node.mppt_sub_data[49]]
        mppt_max_out_volt = [mppt_node.mppt_sub_data[8],mppt_node.mppt_sub_data[22],mppt_node.mppt_sub_data[36],mppt_node.mppt_sub_data[50]]
        mppt_max_in_current = [mppt_node.mppt_sub_data[9],mppt_node.mppt_sub_data[23],mppt_node.mppt_sub_data[37],mppt_node.mppt_sub_data[51]]
        mppt_CAN_RX_err_count = [mppt_node.mppt_sub_data[10],mppt_node.mppt_sub_data[24],mppt_node.mppt_sub_data[38],mppt_node.mppt_sub_data[52]]
        mppt_CAN_TX_err_count = [mppt_node.mppt_sub_data[11],mppt_node.mppt_sub_data[25],mppt_node.mppt_sub_data[39],mppt_node.mppt_sub_data[53]]
        mppt_err_flag = [mppt_node.mppt_sub_data[12],mppt_node.mppt_sub_data[26],mppt_node.mppt_sub_data[40],mppt_node.mppt_sub_data[54]]
        mppt_lim_flag = [mppt_node.mppt_sub_data[13],mppt_node.mppt_sub_data[27],mppt_node.mppt_sub_data[41],mppt_node.mppt_sub_data[55]]

        base_val = 127
        #Fill in the limiting values
        min_input_volt = 20
        max_input_current = 7
        max_output_volt = 175
        max_mosfet_temp = 50
        max_controller_temp =50
        supply_12V_tol = 1
        supply_3V_tol = 1

        for i in range(4):

            if mppt_input_volt[i] < min_input_volt:
                mppt_node.control_pub_data = base_val + 14*i

            if mppt_input_current[i] > max_input_current:
                mppt_node.control_pub_data = base_val + 1 + 14*i
            
            if mppt_output_volt[i] > max_output_volt:
                mppt_node.control_pub_data = base_val + 2 + 14*i

            if mppt_mosfet_temp[i] > max_mosfet_temp:
                mppt_node.control_pub_data = base_val + 3 + 14*i

            if mppt_controller_temp[i] > max_controller_temp:
                mppt_node.control_pub_data = base_val + 4 + 14*i
            
            if mppt_12V_aux_supply[i] < 12 - supply_12V_tol:
                mppt_node.control_pub_data = base_val + 5 + 14*i

            if mppt_3V_aux_supply[i] < 3 - supply_3V_tol:
                mppt_node.control_pub_data = base_val + 6 + 14*i

            if mppt_err_flag[i] == 0:
                mppt_node.control_pub_data = base_val + 7 + 14*i
            if mppt_err_flag[i] == 1:
                mppt_node.control_pub_data = base_val + 8 + 14*i
            if mppt_err_flag[i] == 2:
                mppt_node.control_pub_data = base_val + 9 + 14*i
            if mppt_err_flag[i] == 3:
                mppt_node.control_pub_data = base_val + 10 + 14*i
            if mppt_err_flag[i] == 4:
                mppt_node.control_pub_data = base_val + 11 + 14*i
            if mppt_err_flag[i] == 5:
                mppt_node.control_pub_data = base_val + 12 + 14*i
            if mppt_err_flag[i] == 6:
                mppt_node.control_pub_data = base_val + 13 + 14*i
            if mppt_err_flag[i] == 7:
                mppt_node.control_pub_data = base_val + 14 + 14*i
            
            if mppt_lim_flag[i] == 0:
                mppt_node.control_pub_data = base_val + 15 + 14*i
            if mppt_lim_flag[i] == 1:
                mppt_node.control_pub_data = base_val + 16 + 14*i
            if mppt_lim_flag[i] == 2:
                mppt_node.control_pub_data = base_val + 17 + 14*i
            if mppt_lim_flag[i] == 3:
                mppt_node.control_pub_data = base_val + 18 + 14*i
            if mppt_lim_flag[i] == 4:
                mppt_node.control_pub_data = base_val + 19 + 14*i
            if mppt_lim_flag[i] == 5:
                mppt_node.control_pub_data = base_val + 20 + 14*i
            if mppt_lim_flag[i] == 6:
                mppt_node.control_pub_data = base_val + 21 + 14*i
            if mppt_lim_flag[i] == 7:
                mppt_node.control_pub_data = base_val + 22 + 14*i

if __name__ == '__main__':
    main()
