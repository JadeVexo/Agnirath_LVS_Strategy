import serial
import threading

class ZED_F9P:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None
        self.received_data = ""
        self.data_mutex = threading.Lock()

        # Output data variables
        self.time_utc = ""
        self.latitude = ""
        self.lat_direction = ""
        self.longitude = ""
        self.lon_direction = ""
        self.fix_quality = ""
        self.num_satellites = ""
        self.hdop = ""
        self.altitude = ""
        self.altitude_unit = ""
        self.geoid_separation = ""
        self.geoid_separation_unit = ""
        self.age_of_dgps = ""
        self.dgps_reference_id = ""

    def initialize(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            return True
        except serial.SerialException as e:
            print("Error opening serial port:", str(e))
            return False

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def start_processing(self):
        self.stop_processing = False
        self.processing_thread = threading.Thread(target=self.process_serial_data)
        self.processing_thread.start()

    def stop_processing(self):
        self.stop_processing = True
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join()

    def print_output_data(self):
        with self.data_mutex:
            print("Time (UTC):", self.time_utc)
            print("Latitude:", self.latitude, self.lat_direction)
            print("Longitude:", self.longitude, self.lon_direction)
            print("Fix Quality:", self.fix_quality)
            print("Number of Satellites:", self.num_satellites)
            print("HDOP:", self.hdop)
            print("Altitude:", self.altitude, self.altitude_unit)
            print("Geoid Separation:", self.geoid_separation, self.geoid_separation_unit)
            print("Age of DGPS Data:", self.age_of_dgps)
            print("DGPS Reference ID:", self.dgps_reference_id)
            print("----------------------")

    def process_serial_data(self):
        while not self.stop_processing:
            try:
                received_data = self.serial_port.read(256).decode("utf-8")
                if received_data:
                    with self.data_mutex:
                        self.received_data += received_data

                    sentence_end = self.received_data.find("\r\n")
                    while sentence_end != -1:
                        sentence = self.received_data[:sentence_end]
                        self.received_data = self.received_data[sentence_end + 2:]

                        self.parse_gga(sentence)

                        sentence_end = self.received_data.find("\r\n")
            except serial.SerialException as e:
                print("Error reading serial data:", str(e))
                break

    def parse_gga(self, sentence):
        tokens = sentence.split(",")
        if len(tokens) >= 15 and tokens[0] == "$GPGGA":
            self.time_utc = tokens[1]
            self.latitude = tokens[2]
            self.lat_direction = tokens[3]
            self.longitude = tokens[4]
            self.lon_direction = tokens[5]
            self.fix_quality = tokens[6]
            self.num_satellites = tokens[7]
            self.hdop = tokens[8]
            self.altitude = tokens[9]
            self.altitude_unit = tokens[10]
            self.geoid_separation = tokens[11]
            self.geoid_separation_unit = tokens[12]
            self.age_of_dgps = tokens[13]
            self.dgps_reference_id = tokens[14]


if __name__ == "__main__":
    port = "/dev/ttyS1"
    baud_rate = 9600

    zed_f9p = ZED_F9P(port, baud_rate)

    if not zed_f9p.initialize():
        exit(1)

    zed_f9p.start_processing()

    while True:
        zed_f9p.print_output_data()

    zed_f9p.stop_processing()
    zed_f9p.close()
