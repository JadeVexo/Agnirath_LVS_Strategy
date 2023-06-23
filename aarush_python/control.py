import time
from ros import *
import random


class control_publisher:
    def __init__(self):
        self.data = None
        self.latest_data = None

    def recieve_data(self):
        self.data = [random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1),
                     random.randint(0, 1)]
        self.latest_data = self.data
        return self.latest_data
        

def main():
    publisher = Publisher()
    control = control_publisher() 
    try:
        while True:
            message1 = control.recieve_data()
            publisher.publish_to_csv('control', message1)
            print(f"Published message to topic 'topic1': {message1}")

            time.sleep(1)  # Wait for 1 second
    except KeyboardInterrupt:
        print("Program Exiting.....")

if __name__ == '__main__':
    main()
