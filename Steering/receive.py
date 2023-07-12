from sim_data1 import DataSender

# Create an instance of DataSender
sender = DataSender()
print('hi')
# Access the variables
speed = sender.speed
mode = sender.mode
rpm = sender.rpm
regen = sender.regen
battery = sender.battery
disrem = sender.disrem
brake = sender.brake
horn = sender.horn
radio = sender.radio
cruise = sender.cruise
Lindicator = sender.Lindicator
Rindicator = sender.Rindicator

# Use the values as needed
print(speed, mode, rpm, regen, battery, disrem, brake, horn, radio, cruise, Lindicator, Rindicator)
