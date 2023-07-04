import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class MC_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # MC Subscriber
    def init_mc_subscriber(self, topic):
        self.mc_subscriber = self.create_subscription(
            rosarray, topic, self.receive_mc_data, 10
        )
        self.mc_subscriber  # prevent unused variable warning
        self.mc_sub_data = None

    def receive_mc_data(self, msg):
        self.mc_sub_data = msg.data

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

    mc_node = MC_NODE("mc_node")
    mc_node.init_mc_subscriber("mc_data")
    mc_node.init_peripheral_subscriber("peripheral_data")
    mc_node.init_control_data_publisher("control_data", 1)

    while rclpy.ok():
        rclpy.spin_once(mc_node)

    mc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
