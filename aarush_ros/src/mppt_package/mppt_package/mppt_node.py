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
    
    mppt_1_input_volt_timer = Timer(threshold_crossing_time)
    mppt_1_input_current_timer = Timer(threshold_crossing_time)
    mppt_1_output_volt_timer = Timer(threshold_crossing_time)
    mppt_1_mosfet_temp_timer = Timer(threshold_crossing_time)
    mppt_1_controller_temp_timer = Timer(threshold_crossing_time)
    mppt_1_12V_aux_timer = Timer(threshold_crossing_time)
    mppt_1_3V_aux_timer = Timer(threshold_crossing_time)
    mppt_1_err_0_timer = Timer(threshold_crossing_time)
    mppt_1_err_1_timer = Timer(threshold_crossing_time)
    mppt_1_err_2_timer = Timer(threshold_crossing_time)
    mppt_1_err_3_timer = Timer(threshold_crossing_time)
    mppt_1_err_4_timer = Timer(threshold_crossing_time)
    mppt_1_err_5_timer = Timer(threshold_crossing_time)
    mppt_1_err_6_timer = Timer(threshold_crossing_time)
    mppt_1_err_7_timer = Timer(threshold_crossing_time)
    mppt_1_lim_0_timer = Timer(threshold_crossing_time)
    mppt_1_lim_1_timer = Timer(threshold_crossing_time)
    mppt_1_lim_2_timer = Timer(threshold_crossing_time)
    mppt_1_lim_3_timer = Timer(threshold_crossing_time)
    mppt_1_lim_4_timer = Timer(threshold_crossing_time)
    mppt_1_lim_5_timer = Timer(threshold_crossing_time)
    mppt_1_lim_6_timer = Timer(threshold_crossing_time)
    mppt_1_lim_7_timer = Timer(threshold_crossing_time)

    mppt_2_input_volt_timer = Timer(threshold_crossing_time)
    mppt_2_input_current_timer = Timer(threshold_crossing_time)
    mppt_2_output_volt_timer = Timer(threshold_crossing_time)
    mppt_2_mosfet_temp_timer = Timer(threshold_crossing_time)
    mppt_2_controller_temp_timer = Timer(threshold_crossing_time)
    mppt_2_12V_aux_timer = Timer(threshold_crossing_time)
    mppt_2_3V_aux_timer = Timer(threshold_crossing_time)
    mppt_2_err_0_timer = Timer(threshold_crossing_time)
    mppt_2_err_1_timer = Timer(threshold_crossing_time)
    mppt_2_err_2_timer = Timer(threshold_crossing_time)
    mppt_2_err_3_timer = Timer(threshold_crossing_time)
    mppt_2_err_4_timer = Timer(threshold_crossing_time)
    mppt_2_err_5_timer = Timer(threshold_crossing_time)
    mppt_2_err_6_timer = Timer(threshold_crossing_time)
    mppt_2_err_7_timer = Timer(threshold_crossing_time)
    mppt_2_lim_0_timer = Timer(threshold_crossing_time)
    mppt_2_lim_1_timer = Timer(threshold_crossing_time)
    mppt_2_lim_2_timer = Timer(threshold_crossing_time)
    mppt_2_lim_3_timer = Timer(threshold_crossing_time)
    mppt_2_lim_4_timer = Timer(threshold_crossing_time)
    mppt_2_lim_5_timer = Timer(threshold_crossing_time)
    mppt_2_lim_6_timer = Timer(threshold_crossing_time)
    mppt_2_lim_7_timer = Timer(threshold_crossing_time)

    mppt_3_input_volt_timer = Timer(threshold_crossing_time)
    mppt_3_input_current_timer = Timer(threshold_crossing_time)
    mppt_3_output_volt_timer = Timer(threshold_crossing_time)
    mppt_3_mosfet_temp_timer = Timer(threshold_crossing_time)
    mppt_3_controller_temp_timer = Timer(threshold_crossing_time)
    mppt_3_12V_aux_timer = Timer(threshold_crossing_time)
    mppt_3_3V_aux_timer = Timer(threshold_crossing_time)
    mppt_3_err_0_timer = Timer(threshold_crossing_time)
    mppt_3_err_1_timer = Timer(threshold_crossing_time)
    mppt_3_err_2_timer = Timer(threshold_crossing_time)
    mppt_3_err_3_timer = Timer(threshold_crossing_time)
    mppt_3_err_4_timer = Timer(threshold_crossing_time)
    mppt_3_err_5_timer = Timer(threshold_crossing_time)
    mppt_3_err_6_timer = Timer(threshold_crossing_time)
    mppt_3_err_7_timer = Timer(threshold_crossing_time)
    mppt_3_lim_0_timer = Timer(threshold_crossing_time)
    mppt_3_lim_1_timer = Timer(threshold_crossing_time)
    mppt_3_lim_2_timer = Timer(threshold_crossing_time)
    mppt_3_lim_3_timer = Timer(threshold_crossing_time)
    mppt_3_lim_4_timer = Timer(threshold_crossing_time)
    mppt_3_lim_5_timer = Timer(threshold_crossing_time)
    mppt_3_lim_6_timer = Timer(threshold_crossing_time)
    mppt_3_lim_7_timer = Timer(threshold_crossing_time)

    mppt_4_input_volt_timer = Timer(threshold_crossing_time)
    mppt_4_input_current_timer = Timer(threshold_crossing_time)
    mppt_4_output_volt_timer = Timer(threshold_crossing_time)
    mppt_4_mosfet_temp_timer = Timer(threshold_crossing_time)
    mppt_4_controller_temp_timer = Timer(threshold_crossing_time)
    mppt_4_12V_aux_timer = Timer(threshold_crossing_time)
    mppt_4_3V_aux_timer = Timer(threshold_crossing_time)
    mppt_4_err_0_timer = Timer(threshold_crossing_time)
    mppt_4_err_1_timer = Timer(threshold_crossing_time)
    mppt_4_err_2_timer = Timer(threshold_crossing_time)
    mppt_4_err_3_timer = Timer(threshold_crossing_time)
    mppt_4_err_4_timer = Timer(threshold_crossing_time)
    mppt_4_err_5_timer = Timer(threshold_crossing_time)
    mppt_4_err_6_timer = Timer(threshold_crossing_time)
    mppt_4_err_7_timer = Timer(threshold_crossing_time)
    mppt_4_lim_0_timer = Timer(threshold_crossing_time)
    mppt_4_lim_1_timer = Timer(threshold_crossing_time)
    mppt_4_lim_2_timer = Timer(threshold_crossing_time)
    mppt_4_lim_3_timer = Timer(threshold_crossing_time)
    mppt_4_lim_4_timer = Timer(threshold_crossing_time)
    mppt_4_lim_5_timer = Timer(threshold_crossing_time)
    mppt_4_lim_6_timer = Timer(threshold_crossing_time)
    mppt_4_lim_7_timer = Timer(threshold_crossing_time)

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
        threshold_crossing_time = 3
        
        #MPPT_1
        if mppt_input_volt[0] < min_input_volt:
            if mppt_1_input_volt_timer.start_time is None:
                mppt_1_input_volt_timer.start()
            if mppt_1_input_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val
        else:
            mppt_1_input_volt_timer.reset()

        if mppt_input_current[0] > max_input_current:
            if mppt_1_input_current_timer.start_time is None:
                mppt_1_input_current_timer.start()
            if mppt_1_input_current_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 1
        else:
            mppt_1_input_current_timer.reset()
        

        if mppt_output_volt[0] > max_output_volt:
            if mppt_1_output_volt_timer.start_time is None:
                mppt_1_output_volt_timer.start()
            if mppt_1_output_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 2
        else:
            mppt_1_output_volt_timer.reset()

        if mppt_mosfet_temp[0] > max_mosfet_temp:
            if mppt_1_mosfet_temp_timer.start_time is None:
                mppt_1_mosfet_temp_timer.start()
            if mppt_1_mosfet_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 3
        else:
            mppt_1_mosfet_temp_timer.reset()

        if mppt_controller_temp[0] > max_controller_temp:
            if mppt_1_controller_temp_timer.start_time is None:
                mppt_1_controller_temp_timer.start()
            if mppt_1_controller_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 4
        else:
            mppt_1_controller_temp_timer.reset()
        
        if mppt_12V_aux_supply[0] < 12 - supply_12V_tol:
            if mppt_1_12V_aux_timer.start_time is None:
                mppt_1_12V_aux_timer.start()
            if mppt_1_12V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 5
        else:
            mppt_1_12V_aux_timer.reset()

        if mppt_3V_aux_supply[0] < 3 - supply_12V_tol:
            if mppt_1_3V_aux_timer.start_time is None:
                mppt_1_3V_aux_timer.start()
            if mppt_1_3V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 6
        else:
            mppt_1_3V_aux_timer.reset()


        if mppt_err_flag[0] == 0:
            if mppt_1_err_0_timer.start_time is None:
                mppt_1_err_0_timer.start()
            if mppt_1_err_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 7
        else:
            mppt_1_err_0_timer.reset()

        if mppt_err_flag[0] == 1:
            if mppt_1_err_1_timer.start_time is None:
                mppt_1_err_1_timer.start()
            if mppt_1_err_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 8
        else:
            mppt_1_err_1_timer.reset()

        if mppt_err_flag[0] == 2:
            if mppt_1_err_2_timer.start_time is None:
                mppt_1_err_2_timer.start()
            if mppt_1_err_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 9
        else:
            mppt_1_err_2_timer.reset()

        if mppt_err_flag[0] == 3:
            if mppt_1_err_3_timer.start_time is None:
                mppt_1_err_3_timer.start()
            if mppt_1_err_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 10
        else:
            mppt_1_err_3_timer.reset()

        if mppt_err_flag[0] == 4:
            if mppt_1_err_4_timer.start_time is None:
                mppt_1_err_4_timer.start()
            if mppt_1_err_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 11
        else:
            mppt_1_err_4_timer.reset()

        if mppt_err_flag[0] == 5:
            if mppt_1_err_5_timer.start_time is None:
                mppt_1_err_5_timer.start()
            if mppt_1_err_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 12
        else:
            mppt_1_err_5_timer.reset()

        if mppt_err_flag[0] == 6:
            if mppt_1_err_6_timer.start_time is None:
                mppt_1_err_6_timer.start()
            if mppt_1_err_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 13
        else:
            mppt_1_err_6_timer.reset()

        if mppt_err_flag[0] == 7:
            if mppt_1_err_7_timer.start_time is None:
                mppt_1_err_7_timer.start()
            if mppt_1_err_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 14
        else:
            mppt_1_err_7_timer.reset()
        
        
        if mppt_lim_flag[0] == 0:
            if mppt_1_lim_0_timer.start_time is None:
                mppt_1_lim_0_timer.start()
            if mppt_1_lim_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 15
        else:
            mppt_1_lim_0_timer.reset()

        if mppt_lim_flag[0] == 1:
            if mppt_1_lim_1_timer.start_time is None:
                mppt_1_lim_1_timer.start()
            if mppt_1_lim_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 16
        else:
            mppt_1_lim_1_timer.reset()

        if mppt_lim_flag[0] == 2:
            if mppt_1_lim_2_timer.start_time is None:
                mppt_1_lim_2_timer.start()
            if mppt_1_lim_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 17
        else:
            mppt_1_lim_2_timer.reset()

        if mppt_lim_flag[0] == 3:
            if mppt_1_lim_3_timer.start_time is None:
                mppt_1_lim_3_timer.start()
            if mppt_1_lim_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 18
        else:
            mppt_1_lim_3_timer.reset()

        if mppt_lim_flag[0] == 4:
            if mppt_1_lim_4_timer.start_time is None:
                mppt_1_lim_4_timer.start()
            if mppt_1_lim_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 19
        else:
            mppt_1_lim_4_timer.reset()

        if mppt_lim_flag[0] == 5:
            if mppt_1_lim_5_timer.start_time is None:
                mppt_1_lim_5_timer.start()
            if mppt_1_lim_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 20
        else:
            mppt_1_lim_5_timer.reset()

        if mppt_lim_flag[0] == 6:
            if mppt_1_lim_6_timer.start_time is None:
                mppt_1_lim_6_timer.start()
            if mppt_1_lim_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 21
        else:
            mppt_1_lim_6_timer.reset()

        if mppt_lim_flag[0] == 7:
            if mppt_1_lim_7_timer.start_time is None:
                mppt_1_lim_7_timer.start()
            if mppt_1_lim_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 22
        else:
            mppt_1_lim_7_timer.reset()


        #MPPT_2
        if mppt_input_volt[1] < min_input_volt:
            if mppt_2_input_volt_timer.start_time is None:
                mppt_2_input_volt_timer.start()
            if mppt_2_input_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val +23
        else:
            mppt_2_input_volt_timer.reset()

        if mppt_input_current[1] > max_input_current:
            if mppt_2_input_current_timer.start_time is None:
                mppt_2_input_current_timer.start()
            if mppt_2_input_current_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 24
        else:
            mppt_2_input_current_timer.reset()
        

        if mppt_output_volt[1] > max_output_volt:
            if mppt_2_output_volt_timer.start_time is None:
                mppt_2_output_volt_timer.start()
            if mppt_2_output_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 25
        else:
            mppt_2_output_volt_timer.reset()

        if mppt_mosfet_temp[1] > max_mosfet_temp:
            if mppt_2_mosfet_temp_timer.start_time is None:
                mppt_2_mosfet_temp_timer.start()
            if mppt_2_mosfet_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 26
        else:
            mppt_2_mosfet_temp_timer.reset()

        if mppt_controller_temp[1] > max_controller_temp:
            if mppt_2_controller_temp_timer.start_time is None:
                mppt_2_controller_temp_timer.start()
            if mppt_2_controller_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 27
        else:
            mppt_2_controller_temp_timer.reset()
        
        if mppt_12V_aux_supply[1] < 12 - supply_12V_tol:
            if mppt_2_12V_aux_timer.start_time is None:
                mppt_2_12V_aux_timer.start()
            if mppt_2_12V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 28
        else:
            mppt_2_12V_aux_timer.reset()

        if mppt_3V_aux_supply[1] < 3 - supply_12V_tol:
            if mppt_2_3V_aux_timer.start_time is None:
                mppt_2_3V_aux_timer.start()
            if mppt_2_3V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 29
        else:
            mppt_2_3V_aux_timer.reset()


        if mppt_err_flag[1] == 0:
            if mppt_2_err_0_timer.start_time is None:
                mppt_2_err_0_timer.start()
            if mppt_2_err_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 30
        else:
            mppt_2_err_0_timer.reset()

        if mppt_err_flag[1] == 1:
            if mppt_2_err_1_timer.start_time is None:
                mppt_2_err_1_timer.start()
            if mppt_2_err_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 31
        else:
            mppt_2_err_1_timer.reset()

        if mppt_err_flag[1] == 2:
            if mppt_2_err_2_timer.start_time is None:
                mppt_2_err_2_timer.start()
            if mppt_2_err_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 32
        else:
            mppt_2_err_2_timer.reset()

        if mppt_err_flag[1] == 3:
            if mppt_2_err_3_timer.start_time is None:
                mppt_2_err_3_timer.start()
            if mppt_2_err_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 33
        else:
            mppt_2_err_3_timer.reset()

        if mppt_err_flag[1] == 4:
            if mppt_2_err_4_timer.start_time is None:
                mppt_2_err_4_timer.start()
            if mppt_2_err_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 34
        else:
            mppt_2_err_4_timer.reset()

        if mppt_err_flag[1] == 5:
            if mppt_2_err_5_timer.start_time is None:
                mppt_2_err_5_timer.start()
            if mppt_2_err_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 35
        else:
            mppt_2_err_5_timer.reset()

        if mppt_err_flag[1] == 6:
            if mppt_2_err_6_timer.start_time is None:
                mppt_2_err_6_timer.start()
            if mppt_2_err_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 36
        else:
            mppt_2_err_6_timer.reset()

        if mppt_err_flag[1] == 7:
            if mppt_2_err_7_timer.start_time is None:
                mppt_2_err_7_timer.start()
            if mppt_2_err_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 37
        else:
            mppt_2_err_7_timer.reset()
        
        
        if mppt_lim_flag[1] == 0:
            if mppt_2_lim_0_timer.start_time is None:
                mppt_2_lim_0_timer.start()
            if mppt_2_lim_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 38
        else:
            mppt_2_lim_0_timer.reset()

        if mppt_lim_flag[1] == 1:
            if mppt_2_lim_1_timer.start_time is None:
                mppt_2_lim_1_timer.start()
            if mppt_2_lim_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 39
        else:
            mppt_2_lim_1_timer.reset()

        if mppt_lim_flag[1] == 2:
            if mppt_2_lim_2_timer.start_time is None:
                mppt_2_lim_2_timer.start()
            if mppt_2_lim_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 40
        else:
            mppt_2_lim_2_timer.reset()

        if mppt_lim_flag[1] == 3:
            if mppt_2_lim_3_timer.start_time is None:
                mppt_2_lim_3_timer.start()
            if mppt_2_lim_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 41
        else:
            mppt_2_lim_3_timer.reset()

        if mppt_lim_flag[1] == 4:
            if mppt_2_lim_4_timer.start_time is None:
                mppt_2_lim_4_timer.start()
            if mppt_2_lim_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 42
        else:
            mppt_2_lim_4_timer.reset()

        if mppt_lim_flag[1] == 5:
            if mppt_2_lim_5_timer.start_time is None:
                mppt_2_lim_5_timer.start()
            if mppt_2_lim_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 43
        else:
            mppt_2_lim_5_timer.reset()

        if mppt_lim_flag[1] == 6:
            if mppt_2_lim_6_timer.start_time is None:
                mppt_2_lim_6_timer.start()
            if mppt_2_lim_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 44
        else:
            mppt_2_lim_6_timer.reset()

        if mppt_lim_flag[1] == 7:
            if mppt_2_lim_7_timer.start_time is None:
                mppt_2_lim_7_timer.start()
            if mppt_2_lim_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 45
        else:
            mppt_2_lim_7_timer.reset()

        #MPPT_3
        if mppt_input_volt[2] < min_input_volt:
            if mppt_3_input_volt_timer.start_time is None:
                mppt_3_input_volt_timer.start()
            if mppt_3_input_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val +46
        else:
            mppt_3_input_volt_timer.reset()

        if mppt_input_current[2] > max_input_current:
            if mppt_3_input_current_timer.start_time is None:
                mppt_3_input_current_timer.start()
            if mppt_3_input_current_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 47
        else:
            mppt_3_input_current_timer.reset()
        

        if mppt_output_volt[2] > max_output_volt:
            if mppt_3_output_volt_timer.start_time is None:
                mppt_3_output_volt_timer.start()
            if mppt_3_output_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 48
        else:
            mppt_3_output_volt_timer.reset()

        if mppt_mosfet_temp[2] > max_mosfet_temp:
            if mppt_3_mosfet_temp_timer.start_time is None:
                mppt_3_mosfet_temp_timer.start()
            if mppt_3_mosfet_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 49
        else:
            mppt_3_mosfet_temp_timer.reset()

        if mppt_controller_temp[2] > max_controller_temp:
            if mppt_3_controller_temp_timer.start_time is None:
                mppt_3_controller_temp_timer.start()
            if mppt_3_controller_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 50
        else:
            mppt_3_controller_temp_timer.reset()
        
        if mppt_12V_aux_supply[2] < 12 - supply_12V_tol:
            if mppt_3_12V_aux_timer.start_time is None:
                mppt_3_12V_aux_timer.start()
            if mppt_3_12V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 51
        else:
            mppt_3_12V_aux_timer.reset()

        if mppt_3V_aux_supply[2] < 3 - supply_12V_tol:
            if mppt_3_3V_aux_timer.start_time is None:
                mppt_3_3V_aux_timer.start()
            if mppt_3_3V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 52
        else:
            mppt_3_3V_aux_timer.reset()


        if mppt_err_flag[2] == 0:
            if mppt_3_err_0_timer.start_time is None:
                mppt_3_err_0_timer.start()
            if mppt_3_err_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 53
        else:
            mppt_3_err_0_timer.reset()

        if mppt_err_flag[2] == 1:
            if mppt_3_err_1_timer.start_time is None:
                mppt_3_err_1_timer.start()
            if mppt_3_err_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 54
        else:
            mppt_3_err_1_timer.reset()

        if mppt_err_flag[2] == 2:
            if mppt_3_err_2_timer.start_time is None:
                mppt_3_err_2_timer.start()
            if mppt_3_err_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 55
        else:
            mppt_3_err_2_timer.reset()

        if mppt_err_flag[2] == 3:
            if mppt_3_err_3_timer.start_time is None:
                mppt_3_err_3_timer.start()
            if mppt_3_err_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 56
        else:
            mppt_3_err_3_timer.reset()

        if mppt_err_flag[2] == 4:
            if mppt_3_err_4_timer.start_time is None:
                mppt_3_err_4_timer.start()
            if mppt_3_err_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 57
        else:
            mppt_3_err_4_timer.reset()

        if mppt_err_flag[2] == 5:
            if mppt_3_err_5_timer.start_time is None:
                mppt_3_err_5_timer.start()
            if mppt_3_err_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 58
        else:
            mppt_3_err_5_timer.reset()

        if mppt_err_flag[2] == 6:
            if mppt_3_err_6_timer.start_time is None:
                mppt_3_err_6_timer.start()
            if mppt_3_err_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 59
        else:
            mppt_3_err_6_timer.reset()

        if mppt_err_flag[2] == 7:
            if mppt_3_err_7_timer.start_time is None:
                mppt_3_err_7_timer.start()
            if mppt_3_err_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 60
        else:
            mppt_3_err_7_timer.reset()
        
        
        if mppt_lim_flag[2] == 0:
            if mppt_3_lim_0_timer.start_time is None:
                mppt_3_lim_0_timer.start()
            if mppt_3_lim_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 61
        else:
            mppt_3_lim_0_timer.reset()

        if mppt_lim_flag[2] == 1:
            if mppt_3_lim_1_timer.start_time is None:
                mppt_3_lim_1_timer.start()
            if mppt_3_lim_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 62
        else:
            mppt_3_lim_1_timer.reset()

        if mppt_lim_flag[2] == 2:
            if mppt_3_lim_2_timer.start_time is None:
                mppt_3_lim_2_timer.start()
            if mppt_3_lim_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 63
        else:
            mppt_3_lim_2_timer.reset()

        if mppt_lim_flag[2] == 3:
            if mppt_3_lim_3_timer.start_time is None:
                mppt_3_lim_3_timer.start()
            if mppt_3_lim_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 64
        else:
            mppt_3_lim_3_timer.reset()

        if mppt_lim_flag[2] == 4:
            if mppt_3_lim_4_timer.start_time is None:
                mppt_3_lim_4_timer.start()
            if mppt_3_lim_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 65
        else:
            mppt_3_lim_4_timer.reset()

        if mppt_lim_flag[2] == 5:
            if mppt_3_lim_5_timer.start_time is None:
                mppt_3_lim_5_timer.start()
            if mppt_3_lim_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 66
        else:
            mppt_3_lim_5_timer.reset()

        if mppt_lim_flag[2] == 6:
            if mppt_3_lim_6_timer.start_time is None:
                mppt_3_lim_6_timer.start()
            if mppt_3_lim_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 67
        else:
            mppt_3_lim_6_timer.reset()

        if mppt_lim_flag[2] == 7:
            if mppt_3_lim_7_timer.start_time is None:
                mppt_3_lim_7_timer.start()
            if mppt_3_lim_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 68
        else:
            mppt_3_lim_7_timer.reset()
        
        #MPPT_4
        if mppt_input_volt[3] < min_input_volt:
            if mppt_4_input_volt_timer.start_time is None:
                mppt_4_input_volt_timer.start()
            if mppt_4_input_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 69
        else:
            mppt_4_input_volt_timer.reset()

        if mppt_input_current[3] > max_input_current:
            if mppt_4_input_current_timer.start_time is None:
                mppt_4_input_current_timer.start()
            if mppt_4_input_current_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 70
        else:
            mppt_4_input_current_timer.reset()
        

        if mppt_output_volt[3] > max_output_volt:
            if mppt_4_output_volt_timer.start_time is None:
                mppt_4_output_volt_timer.start()
            if mppt_4_output_volt_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 71
        else:
            mppt_4_output_volt_timer.reset()

        if mppt_mosfet_temp[3] > max_mosfet_temp:
            if mppt_4_mosfet_temp_timer.start_time is None:
                mppt_4_mosfet_temp_timer.start()
            if mppt_4_mosfet_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 72
        else:
            mppt_4_mosfet_temp_timer.reset()

        if mppt_controller_temp[3] > max_controller_temp:
            if mppt_4_controller_temp_timer.start_time is None:
                mppt_4_controller_temp_timer.start()
            if mppt_4_controller_temp_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 73
        else:
            mppt_4_controller_temp_timer.reset()
        
        if mppt_12V_aux_supply[3] < 12 - supply_12V_tol:
            if mppt_4_12V_aux_timer.start_time is None:
                mppt_4_12V_aux_timer.start()
            if mppt_4_12V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 74
        else:
            mppt_4_12V_aux_timer.reset()

        if mppt_3V_aux_supply[3] < 3 - supply_12V_tol:
            if mppt_4_3V_aux_timer.start_time is None:
                mppt_4_3V_aux_timer.start()
            if mppt_4_3V_aux_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 75
        else:
            mppt_4_3V_aux_timer.reset()


        if mppt_err_flag[3] == 0:
            if mppt_4_err_0_timer.start_time is None:
                mppt_4_err_0_timer.start()
            if mppt_4_err_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 76
        else:
            mppt_4_err_0_timer.reset()

        if mppt_err_flag[3] == 1:
            if mppt_4_err_1_timer.start_time is None:
                mppt_4_err_1_timer.start()
            if mppt_4_err_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 77
        else:
            mppt_4_err_1_timer.reset()

        if mppt_err_flag[3] == 2:
            if mppt_4_err_2_timer.start_time is None:
                mppt_4_err_2_timer.start()
            if mppt_4_err_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 78
        else:
            mppt_4_err_2_timer.reset()

        if mppt_err_flag[3] == 3:
            if mppt_4_err_3_timer.start_time is None:
                mppt_4_err_3_timer.start()
            if mppt_4_err_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 79
        else:
            mppt_4_err_3_timer.reset()

        if mppt_err_flag[3] == 4:
            if mppt_4_err_4_timer.start_time is None:
                mppt_4_err_4_timer.start()
            if mppt_4_err_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 80
        else:
            mppt_4_err_4_timer.reset()

        if mppt_err_flag[3] == 5:
            if mppt_4_err_5_timer.start_time is None:
                mppt_4_err_5_timer.start()
            if mppt_4_err_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 81
        else:
            mppt_4_err_5_timer.reset()

        if mppt_err_flag[3] == 6:
            if mppt_4_err_6_timer.start_time is None:
                mppt_4_err_6_timer.start()
            if mppt_4_err_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 82
        else:
            mppt_4_err_6_timer.reset()

        if mppt_err_flag[3] == 7:
            if mppt_4_err_7_timer.start_time is None:
                mppt_4_err_7_timer.start()
            if mppt_4_err_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 83
        else:
            mppt_4_err_7_timer.reset()
        
        
        if mppt_lim_flag[3] == 0:
            if mppt_4_lim_0_timer.start_time is None:
                mppt_4_lim_0_timer.start()
            if mppt_4_lim_0_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 84
        else:
            mppt_4_lim_0_timer.reset()

        if mppt_lim_flag[3] == 1:
            if mppt_4_lim_1_timer.start_time is None:
                mppt_4_lim_1_timer.start()
            if mppt_4_lim_1_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 85
        else:
            mppt_4_lim_1_timer.reset()

        if mppt_lim_flag[3] == 2:
            if mppt_4_lim_2_timer.start_time is None:
                mppt_4_lim_2_timer.start()
            if mppt_4_lim_2_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 86
        else:
            mppt_4_lim_2_timer.reset()

        if mppt_lim_flag[3] == 3:
            if mppt_4_lim_3_timer.start_time is None:
                mppt_4_lim_3_timer.start()
            if mppt_4_lim_3_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 87
        else:
            mppt_4_lim_3_timer.reset()

        if mppt_lim_flag[3] == 4:
            if mppt_4_lim_4_timer.start_time is None:
                mppt_4_lim_4_timer.start()
            if mppt_4_lim_4_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 88
        else:
            mppt_4_lim_4_timer.reset()

        if mppt_lim_flag[3] == 5:
            if mppt_4_lim_5_timer.start_time is None:
                mppt_4_lim_5_timer.start()
            if mppt_4_lim_5_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 89
        else:
            mppt_4_lim_5_timer.reset()

        if mppt_lim_flag[3] == 6:
            if mppt_4_lim_6_timer.start_time is None:
                mppt_4_lim_6_timer.start()
            if mppt_4_lim_6_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 90
        else:
            mppt_4_lim_6_timer.reset()

        if mppt_lim_flag[3] == 7:
            if mppt_4_lim_7_timer.start_time is None:
                mppt_4_lim_7_timer.start()
            if mppt_4_lim_7_timer.elapsed() >= threshold_crossing_time:
                mppt_node.control_pub_data = base_val + 91
        else:
            mppt_4_lim_7_timer.reset()
        
if __name__== '__main__':
    main()