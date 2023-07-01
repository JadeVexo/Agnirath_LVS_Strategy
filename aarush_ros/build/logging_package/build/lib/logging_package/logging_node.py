import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class IMU_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.imu_node = node

    
def main(args=None):
    rclpy.init(args=args)

    imu_node = IMU_NODE("imu_data")
    imu_node.init_imu_publisher("imu_data",1,2)
    
    while rclpy.ok():
        rclpy.spin_once(imu_node)

    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
