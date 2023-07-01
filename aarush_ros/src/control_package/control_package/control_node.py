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
        print("SUB:",self.parsed_sub_data)

    # Control Data publisher
    def init_control_data_publisher(self, topic, timer_period):
        self.control_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.control_data_timer = self.create_timer(timer_period, self.publish_control_data)
        self.control_pub_data = None

    def publish_control_data(self):
        self.control_data_pub_msg = rosarray()
        self.control_data_pub_msg.data = self.control_pub_data
        if self.control_pub_data is not None:
            self.control_data_publisher.publish(self.control_data_pub_msg)
        self.control_pub_data = self.control_data_pub_msg.data
        print("PUB:", self.control_pub_data)

    def init_battery_data_publisher(self, topic, timer_period):
        self.battery_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.battery_data_timer = self.create_timer(timer_period, self.publish_battery_data)
        self.battery_pub_data = None

    def publish_battery_data(self):
        self.battery_data_pub_msg = rosarray()
        self.battery_data_pub_msg.data = self.battery_pub_data
        if self.battery_pub_data is not None:
            self.battery_data_publisher.publish(self.battery_data_pub_msg)
        self.battery_pub_data = self.battery_data_pub_msg.data
        print("PUB:", self.battery_pub_data)

    def init_mppt_data_publisher(self, topic, timer_period):
        self.mppt_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.mppt_data_timer = self.create_timer(timer_period, self.publish_mppt_data)
        self.mppt_data_pub_data = None

    def publish_mppt_data(self):
        self.mppt_data_pub_msg = rosarray()
        self.mppt_data_pub_msg.data = self.mppt_data_pub_data
        if self.battery_pub_data is not None:
            self.mppt_data_publisher.publish(self.mppt_data_pub_msg)
        self.mppt_data_pub_data = self.mppt_data_pub_msg.data
        print("PUB:", self.battery_pub_data)




def main(args=None):
    rclpy.init(args=args)

    control_node = CONTROL_NODE("control_node")
    control_node.init_parsed_data_subscriber("parsed_data")
    control_node.init_control_data_publisher("control_data")
    control_node.init_battery_data_publisher("battery_data")
    control_node.init_mppt_data_publisher("mppt_data")

    while rclpy.ok():
        rclpy.spin_once(control_node)

        control_node.battery_pub_data = 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()