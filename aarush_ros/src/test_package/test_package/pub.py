import rclpy
# from OpenCVnodetest import cameraModule
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

class Anglepublisher(Node):
    # _camera_module = cameraModule()
    def __init__(self):
        super().__init__("maximumLikelihood")
        self.publisher_ = self.create_publisher(Float64MultiArray,'Angles',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [20.0,20.0,320.0.i,20.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: ',msg.data)
        self.i +=1

def main(args=None):
    rclpy.init(args=args)
    anglepublisher = Anglepublisher()

    rclpy.spin(anglepublisher)

    anglepublisher.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()