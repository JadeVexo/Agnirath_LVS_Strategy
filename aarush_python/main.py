from ros import *

def subscriber_main():
    topic1_subscriber = Subscriber('control')

    try:
        while True:
            topic1_message = topic1_subscriber.subscribe_to_csv()

            if topic1_message is not None:
                print(f"Received message for topic 'topic1': {topic1_message}")

    except KeyboardInterrupt:
        print("Program Exiting.....")

if __name__ == '__main__':
    subscriber_main()