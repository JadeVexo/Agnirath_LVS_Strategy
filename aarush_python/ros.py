import csv
import time
import uuid
import ast

class Publisher:
    def publish_to_csv(self, topic, message):
        timestamp = time.time()
        message_id = str(uuid.uuid4())
        data = []
        topic_found = False

        with open('data.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if row[0] == topic:
                    row[1] = message
                    row[2] = str(timestamp)
                    row[3] = message_id
                    topic_found = True
                data.append(row)

        if not topic_found:
            data.append([topic, message, str(timestamp), message_id])

        with open('data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(data)

class Subscriber:
    def __init__(self, topic):
        self.topic = topic
        self.subscribed_messages = set()

    def check_subscribed(self, message_id):
        return message_id not in self.subscribed_messages

    def subscribe_to_csv(self):
        received_message = None

        with open('data.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if row[0] == self.topic:
                    message = row[1]
                    timestamp = float(row[2])
                    message_id = row[3]
                    current_time = time.time()
                    time_diff = current_time - timestamp
                    if time_diff < 2 and self.check_subscribed(message_id):
                        if message[0] == "[" and message[-1] == "]":
                            received_message = ast.literal_eval(message)
                        else:
                            received_message = message
                        self.subscribed_messages.add(message_id)
                        break
        
        return received_message
