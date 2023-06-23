import time
from ros import *

def main():
    publisher = Publisher()

    x = 1
    try:
        while True:
            message1 = str(x)
            publisher.publish_to_csv('topic1', message1)
            print(f"Published message: '{message1}' to topic 'topic1'")

            message2 = [x*2,str(x*3)]
            publisher.publish_to_csv('topic2', message2)
            print(f"Published message: '{message2}' to topic 'topic2'")

            x += 1
            time.sleep(1)  # Wait for 1 second
    except KeyboardInterrupt:
        print("Program Exiting.....")

if __name__ == '__main__':
    main()
