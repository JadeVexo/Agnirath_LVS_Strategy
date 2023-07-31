import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random
import can
#import struct
import cantools

class CAN_RX_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # CAN publisher
    def init_can_publisher(self, topic, timer_period):
        self.can_publisher = self.create_publisher(rosarray, topic, 100)
        self.can_timer = self.create_timer(timer_period, self.publish_can_data)
        #self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/ubuntu/Agnirath_LVS_Strategy/aarush_ros/src/can_package/can_package/DBC_Files/mc_can.dbc')
        self.can_pub_data = None

    def init_control_data_publisher(self, topic, timer_period):
        self.control_data_publisher = self.create_publisher(rosarray, topic, 10)
        self.control_data_timer = self.create_timer(
            timer_period, self.publish_can_data
        )
        self.control_pub_data = None

    def publish_can_data(self):

        # self.can_pub_data 
        
        with can.interface.Bus('can0', bustype='socketcan', bitrate=500000) as can_bus:
        
            self.response = self.bus.recv(timeout=2)

            try:
                self.decoded_data =  self.db.decode_message(message.arbitration_id, message.data)
                self.message_decoded = list(self.decoded_data.values())
            except KeyError:
                pass




        #self.id = [self.response.arbitration_id]
        #self.data = list(struct.unpack('f',self.response.data[:4]))
        #self.data +=list(struct.unpack('f',self.response.data[4:]))
        #print(self.id,self.data)
        #print("REC:", self.can_pub_data) 
        self.can_pub_data = self.message_decoded
        self.can_pub_msg = rosarray()
        self.can_pub_msg.data = self.can_pub_data
        if self.can_pub_data is not None:
            self.can_publisher.publish(self.can_pub_msg)
        self.can_pub_data = self.can_pub_msg.data
        print("PUB:", self.can_pub_data)

def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_RX_NODE("can_rx_node")
    can_node.init_can_publisher("can_rx_data", 0.01)

    while rclpy.ok():
        rclpy.spin_once(can_node)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

