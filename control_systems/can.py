import can
from can.interfaces.interface import Bus
import threading


class CANbus(threading.Thread):
    def __init__(self, can_interface='socketcan', can_channel='can0'):
        super().__init__()
        self.bus = None
        self.can_initialized = False
        self.uart_initialized = False

        self.can_interface = can_interface
        self.can_channel = can_channel

        self._stop_event = threading.Event()

    def initialize_can(self):
        try:
            self.bus = Bus(interface=self.can_interface, channel=self.can_channel)
            self.can_initialized = True
            print("CANbus initialized successfully.")
        except Exception as e:
            print(f"Failed to initialize CANbus: {e}")

    def initialize_uart(self):
        # Add your UART initialization code here
        # Set self.uart_initialized = True once initialized successfully
        pass

    def is_initialized(self):
        return self.can_initialized and self.uart_initialized

    def run(self):
        if not self.is_initialized():
            print("CANbus or UART connection not initialized properly.")
            return

        while not self._stop_event.is_set():
            received_message = self.bus.recv(timeout=1.0)
            if received_message is not None:
                print(f"Received message: ID={received_message.arbitration_id} Data={received_message.data}")

    def send_message(self, arbitration_id, data, is_extended_id=False):
        if not self.is_initialized():
            print("CANbus or UART connection not initialized properly.")
            return

        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id
        )
        self.bus.send(message)

    def receive_messages(self, timeout=1.0):
        if not self.is_initialized():
            print("CANbus or UART connection not initialized properly.")
            return []

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
