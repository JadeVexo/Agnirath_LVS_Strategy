# import sys
# import socket

# def receive_speed():
#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     client_socket.connect(('localhost', 8888))

#     while True:
#         speed_data = client_socket.recv(1024)
#         # rpm_data = client_socket.recv(1024)
#         if not speed_data:
#             break
#         # rpm = int(rpm_data.strip())
#         speed = int(speed_data.strip())
#         print(f"Received speed: {speed}")
#         # print(f"Received speed: {rpm}")

#     client_socket.close()


# if __name__ == '__main__':
#     receive_speed()

import sys
import socket

def receive_variables():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 8888))

    variables = ["speed", "rpm", "regen", "battery", "disrem", "horn", "radio", "cruise", "Lindicator", "Rindicator"]

    while True:
        variable_data = client_socket.recv(1024)
        if not variable_data:
            break
        variable_values = variable_data.decode().strip().split()  # Split the received data into individual values
        
        for i, value in enumerate(variable_values):
            variable_name = variables[i]  # Get the corresponding variable name
            variable_value = int(value)  # Convert the value to an integer
            
            print(f"Received {variable_name}: {variable_value}")

    client_socket.close()


if __name__ == '__main__':
    receive_variables()
