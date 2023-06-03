#   Control Systems Design - Aarush
#          Team Agnirath
 
#   Created By: Jaay ED21
#
#   With the support of:
#    - Jaivel EE22
#    - Vishu EE22
#    - Veadesh CE22 

# Class Imports
from gps import ZED_F9P
from can import CANbus

#Library Imports
import time
import threading
import numpy

if __name__ == "__main__":

    #GPS Initialization
    zed_f9p = ZED_F9P('/dev/ttyS1', 9600)
    if not zed_f9p.initialize():
        exit(1)
    zed_f9p.start_processing()

    #CANBus Initalization
    can_bus = CANbus()
    can_bus.initialize_can()
    can_bus.initialize_uart()
    if can_bus.is_initialized():
        # Start the thread
        can_bus.start()

    # Main loop
    while True:
        # Sending a CAN message
        can_bus.send_message(0x123, [0x01, 0x02, 0x03])

        # Receiving CAN messages for 1 second
        received = can_bus.receive_messages(timeout=1.0)
        for message in received:
            print(f"Received message: ID={message.arbitration_id} Data={message.data}")

        # GPS output
        zed_f9p.print_output_data()

    
    #Thread Termination
    can_bus.stop()
    zed_f9p.stop_processing()
    zed_f9p.close()
