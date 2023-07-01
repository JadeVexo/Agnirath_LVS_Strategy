import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class CAN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # CAN publisher
    def init_can_publisher(self, topic, timer_period):
        self.can_pub = self.create_publisher(rosarray, topic, 10)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)
        self.can_pub_data = []

    def publish_can_data(self):
        self.can_pub_msg = rosarray()
        self.can_pub_msg.data = self.can_pub_data
        self.can_pub.publish(self.can_pub_msg)
        self.can_pub_data = self.can_pub_msg.data
        print("PUB:", self.can_pub_data)

def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_NODE("can_node")
    can_node.init_can_publisher("can_rx_data", 1)

    while rclpy.ok():
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
