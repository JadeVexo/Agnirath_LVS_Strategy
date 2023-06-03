import can
from can.interfaces.interface import Bus
import threading


class CANbus(threading.Thread):
    def __init__(self, can_interface='socketcan', can_channel='can0'):
        super().__init__()
        self.bus = Bus(interface=can_interface, channel=can_channel)
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            received_message = self.bus.recv(timeout=1.0)
            if received_message is not None:
                print(f"Received message: ID={received_message.arbitration_id} Data={received_message.data}")

    def send_message(self, arbitration_id, data, is_extended_id=False):
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id
        )
        self.bus.send(message)

    def receive_messages(self, timeout=1.0):
        received_messages = []
        while True:
            received_message = self.bus.recv(timeout=timeout)
            if received_message is not None:
                received_messages.append(received_message)
            else:
                break
        return received_messages

    def stop(self):
        self._stop_event.set()