import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray


class control_subscriber(Node):

    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(rosarray,'control_data',self.recieve_data,10)
        self.subscription  # prevent unused variable warning
        self.latest_data = None

    def recieve_data(self, msg):
        #self.get_logger().info(msg.data)
        self.latest_data = msg.data
    
    def get_latest_data(self):
        return self.latest_data


def main(args=None):
    rclpy.init(args=args)

    control_node = control_subscriber()

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