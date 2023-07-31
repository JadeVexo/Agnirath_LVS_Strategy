import can
import cantools

db = cantools.database.load_file('/home/ubuntu/Agnirath_LVS_Strategy/aarush_ros/src/can_package/can_package/DBC_Files/mc_can.dbc')

# Create and manage the CAN interface using the 'with' statement
with can.interface.Bus('can0', bustype='socketcan', bitrate=500000) as can_bus:
    # Create a message notifier for receiving CAN messages
    while True:

        message = can_bus.recv()
        try:
            decoded_data =  db.decode_message(message.arbitration_id, message.data)
            # print(decoded_data)
            message_list = list(decoded_data.values())
            print(message_list) 
        except KeyError:
            pass
    # notifier = can.Notifier(can_bus, [can.Printer()])

   # try:
       # while True:
            # The 'recv()' function is not used here since the 'Notifier' takes care of message reception
           # pass
   # except KeyboardInterrupt:
       # print("CAN error")
