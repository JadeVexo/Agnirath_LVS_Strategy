import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random


class control_publisher(Node):

    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(rosarray, 'control_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.recieve_data)
        self.data = [0,0,0,0,0]
        self.latest_data = None

    def recieve_data(self):
        msg = rosarray()
        msg.data = self.data
        self.publisher_.publish(msg)
        #self.get_logger().info(msg.data)
        self.data = [random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1)]
        self.latest_data = msg.data
        
    def get_latest_data(self):
        return self.latest_data


def main(args=None):
    rclpy.init(args=args)

    control_node = control_publisher()

    while rclpy.ok():
        rclpy.spin_once(control_node)
        latest_data = control_node.get_latest_data()
        if latest_data is not None:
            print(latest_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()