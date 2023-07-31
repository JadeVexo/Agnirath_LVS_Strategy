import rclpy
from rclpy.node import Node
import struct
import can
from std_msgs.msg import Float32MultiArray as rosarray



def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])

slow_inst = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
open_bms_c = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
open_mc_c = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
open_m_c = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
set_current_0 = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
red_current = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
open_c_pre_mppt = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
open_c_post_mppt = [0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00]

bus = can.interface.Bus('can0', bustype='socketcan', bitrate=500000)
        

class CAN_NODE(Node):
    def __init__(self, node):
        super().__init__(node)

    # CAN Subscriber
    def init_can_subscriber(self, topic):
        self.can_subscriber = self.create_subscription(
            rosarray, topic, self.receive_can_data, 10
        )
        self.can_subscriber  # prevent unused variable warning
        self.can_sub_data = None

    def receive_can_data(self, msg):
        self.can_sub_data = msg.data


def main(args=None):
    rclpy.init(args=args)

    can_node = CAN_NODE("can_tx_node")
    can_node.init_can_subscriber("can_tx_data")

    while rclpy.ok():
        rclpy.spin_once(can_node)

        if can_node.can_sub_data is not None:
            print(can_node.can_sub_data) 
            error_flags = can_node.can_sub_data

            for error in error_codes:

                if error in range(1,6) or error in range(11,16) or error == 102 or error == 104:
                    message = can.Message(arbitration_id = placeholder, data = slow_inst)
                    bus.send(message)

                if error in range(6,11) or error in range(16,102) or error == 103 or error == 105 or error == 106:
                    message = can.Message(arbitration_id = placeholder, data = slow_inst)
                    bus.send(message)
                    #Arbitration I D must be according to EVDC datatsheet
                    message = can.Message(arbitration_id = 512, data = open_bms_c)
                    bus.send(message)
                    

                if error in range(107,111) or error in range(113,115) or error == 116 or error == 118 or error in range(121,124):
                    message = can.Message(arbitration_id = 513, data = open_mc_c)
                    bus.send(message)
                    message = can.Message(arbitration_id = 514, data = open_m_c)
                    bus.send(message)

                if error == 111:
                    #ID according to EVDC Datasheet
                    message = can.Message(arbitration_id = placeholder, data = set_current_0)
                    bus.send(message)

                if error in range(119,121):
                    message = can.Message(arbitration_id = 514, data = open_m_c)
                    bus.send(message)

                if error == 124 or error ==126 : 
                    message = can.Message(arbitration_id = placeholder, data = slow_inst)
                    bus.send(message)
                
                if error == 125: 
                    message = can.Message(arbitration_id = placeholder, data = slow_inst)
                    bus.send(message)
                    message = can.Message(arbitration_id = placeholder, data = red_current)
                    bus.send(message)

                #MPPT 1 errors

                if error == 128 or error == 149 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_post_mppt)
                    bus.send(message)

                if error == 142 or error == 143 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_pre_mppt)
                    bus.send(message)

                #MPPT 2 error
                    
                if error == 151 or error == 172 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_post_mppt)
                    bus.send(message)

                if error == 165 or error == 166 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_pre_mppt)
                    bus.send(message)

                #MPPT 3 errror

                if error == 174 or error == 195 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_post_mppt)
                    bus.send(message)

                if error == 188 or error == 189 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_pre_mppt)
                    bus.send(message)

                #MPPT 4 error

                if error == 197 or error == 218 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_post_mppt)
                    bus.send(message)

                if error == 211 or error == 212 :
                    message = can.Message(arbitration_id = placeholder, data = open_c_pre_mppt)
                    bus.send(message)


                

        # Map Error codes to corresponding messages



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

