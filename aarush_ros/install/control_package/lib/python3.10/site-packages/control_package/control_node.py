import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random


class CONTROL_NODE(Node):

    def __init__(self,node):
        super().__init__(node)
    
    # Parsed Data Subscriber
    def init_parsed_data_subscriber(self, topic):
        self.parsed_data_subscriber = self.create_subscription(
            rosarray, topic, self.receive_parsed_data, 10
        )
        self.parsed_data_subscriber  # prevent unused variable warning
        self.parsed_sub_data = None

    def receive_parsed_data(self, msg):
        self.parsed_sub_data = msg.data
        #print("SUB:",self.parsed_sub_data)

    def init_battery_data_publisher(self, topic, timer_period):
        self.battery_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.battery_data_timer = self.create_timer(timer_period, self.publish_battery_data)
        self.battery_pub_data = None

    def publish_battery_data(self):
        if self.battery_pub_data is not None:
            self.battery_data_pub_msg = rosarray()
            self.battery_data_pub_msg.data = self.battery_pub_data
            self.battery_data_publisher.publish(self.battery_data_pub_msg)
            self.battery_pub_data = self.battery_data_pub_msg.data
            print("PUB:", self.battery_pub_data)

    def init_mppt_data_publisher(self, topic, timer_period):
        self.mppt_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.mppt_data_timer = self.create_timer(timer_period, self.publish_mppt_data)
        self.mppt_pub_data = None

    def publish_mppt_data(self):
        if self.mppt_pub_data is not None:
            self.mppt_data_pub_msg = rosarray()
            self.mppt_data_pub_msg.data = self.mppt_pub_data
            self.mppt_data_publisher.publish(self.mppt_data_pub_msg)
            self.mppt_pub_data = self.mppt_data_pub_msg.data
            print("PUB:", self.mppt_pub_data)

    def init_mc_data_publisher(self, topic, timer_period):
        self.mc_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.mc_data_timer = self.create_timer(timer_period, self.publish_mc_data)
        self.mc_pub_data = None

    def publish_mc_data(self):
        if self.mc_pub_data is not None:
            self.mc_data_pub_msg = rosarray()
            self.mc_data_pub_msg.data = self.mc_pub_data
            self.mc_data_publisher.publish(self.mc_data_pub_msg)
            self.mc_pub_data = self.mc_data_pub_msg.data
            print("PUB:", self.mc_pub_data)




def main(args=None):
    rclpy.init(args=args)

    control_node = CONTROL_NODE("control_node")
    control_node.init_parsed_data_subscriber("parsed_data")
    control_node.init_battery_data_publisher("battery_data",1)
    control_node.init_mppt_data_publisher("mppt_data",1)
    control_node.init_mc_data_publisher("mc_data",1)

    while rclpy.ok():
        rclpy.spin_once(control_node)

        control_node.battery_pub_data = control_node.parsed_sub_data[0:59]
        #control_node.mppt_pub_data = control_node.parsed_sub_data[59:115]
        #control_node.mc_pub_data = control_node.parsed_sub_data[115:123]
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()