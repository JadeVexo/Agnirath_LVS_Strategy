import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random
import adafruit_rfm9x

class rf_publisher_subscriber(Node):
    def __init__(self,node):
        super().__init__(node)

    def init_sub(self):
        self.subscription = self.create_subscription(
            rosarray, "/final_data", self.receive_data, 10
        )
        self.subscription  # prevent unused variable warning
        self.latest_sub_data = None

    def receive_data(self, sub_msg):
        self.latest_sub_data = sub_msg.data
        #print("SUB:",self.latest_pub_data)


def main(args=None):
    rclpy.init(args=args)

    pub_sub_node = rf_publisher_subscriber("rf_node")
    pub_sub_node.init_sub()

    while rclpy.ok():

        rclpy.spin_once(pub_sub_node)

        latest_sub_data = pub_sub_node.latest_sub_data

        if latest_sub_data is not None:
            print("SUB:",latest_sub_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
