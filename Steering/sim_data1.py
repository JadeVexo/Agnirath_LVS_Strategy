import time
import random

class DataSender:
    def __init__(self):
        self.speed = 0
        self.rpm = 0
        self.regen = 0
        self.battery = 0
        self.disrem = 3000
        self.mode = 0
        self.brake = False
        self.horn = False
        self.radio = False
        self.cruise = False
        self.Lindicator = False
        self.Rindicator = False

    def generate_data(self):
        self.speed += 1
        if self.speed > 150:
            self.speed = 0

        self.mode += 1
        if self.mode > 3:
            self.mode = 0

        self.rpm += 10
        if self.rpm > 1700:
            self.rpm = 0

        self.regen += 1
        if self.regen > 100:
            self.regen = 0

        self.battery += 1
        if self.battery > 100:
            self.battery = 0

        self.disrem -= 10
        if self.disrem < 0:
            self.disrem = 3000

        self.brake = bool(random.randint(0, 1))
        self.horn = bool(random.randint(0, 1))
        self.radio = bool(random.randint(0, 1))
        self.cruise = bool(random.randint(0, 1))
        self.Lindicator = bool(random.randint(0, 1))
        self.Rindicator = bool(random.randint(0, 1))

        variable_data = f"{self.speed} {self.rpm} {self.mode} {self.regen} {self.battery} {self.disrem} {self.brake} {self.horn} {self.radio} {self.cruise} {self.Lindicator} {self.Rindicator}\n"
        data = [
            self.speed,
            self.mode,
            self.rpm,
            self.regen,
            self.battery,
            self.disrem,
            self.brake,
            self.horn,
            self.radio,
            self.cruise,
            self.Lindicator,
            self.Rindicator,
        ]
        # print(data)
        # print(variable_data)
        time.sleep(1)

    def send_data(self):
        while True:
            self.generate_data()


# Usage
sender = DataSender()
sender.send_data()
