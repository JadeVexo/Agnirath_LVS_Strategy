import socket
import time
import random


def send_data():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_address = ("", 8888)
    server_socket.bind(server_address)
    server_socket.listen(1)

    client_socket, client_address = server_socket.accept()
    server_socket.close()

    speed = 0
    rpm = 0
    regen = 0
    battery = 0
    disrem = 3000
    mode = 0
    brake = False
    horn = False
    radio = False
    cruise = False
    Lindicator = False
    Rindicator = False

    while True:
        speed += 1
        if speed > 150:
            speed = 0

        mode += 1
        if mode > 3:
            mode = 0

        rpm += 10
        if rpm > 1700:
            rpm = 0

        regen += 1
        if regen > 100:
            regen = 0

        battery += 1
        if battery > 100:
            battery = 0

        disrem -= 10
        if disrem < 0:
            disrem = 3000

        brake = random.randint(0, 1)
        horn = random.randint(0, 1)
        radio = random.randint(0, 1)
        cruise = random.randint(0, 1)
        Lindicator = random.randint(0, 1)
        Rindicator = random.randint(0, 1)

        variable_data = f"{speed} {rpm} {mode} {regen} {battery} {disrem} {brake} {horn} {radio} {cruise} {Lindicator} {Rindicator}\n"
        data = [
            speed,
            mode,
            rpm,
            regen,
            battery,
            disrem,
            brake,
            horn,
            radio,
            cruise,
            Lindicator,
            Rindicator,
        ]
        # print(data)
        print(variable_data)
        client_socket.sendall(variable_data.encode())
        time.sleep(0.1)

    client_socket.close()


send_data()
