import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial

# TOKEN KEY
# self.time_utc = tokens[1]
# self.latitude = tokens[2]
# self.lat_direction = tokens[3]
# self.longitude = tokens[4]
# self.lon_direction = tokens[5]
# self.fix_quality = tokens[6]
# self.num_satellites = tokens[7]
# self.hdop = tokens[8]
# self.altitude = tokens[9]
# self.altitude_unit = tokens[10]
# self.geoid_separation = tokens[11]
# self.geoid_separation_unit = tokens[12]
# self.age_of_dgps = tokens[13]
# self.dgps_reference_id = tokens[14]


class GPS_NODE(Node):
    def __init__(self, node):
        super().__init__(node)
        self.gps_node = node

    def init_gps_publisher(self, topic, timer_period, port, baud_rate):
        self.gps_pub = self.create_publisher(rosarray, topic, 10)
        self.gps_timer = self.create_timer(timer_period, self.publish_gps_data)
        self.gps_pub_data = []
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            print("GPS is connected and working")
        except serial.SerialException:
            print("GPS is not working")

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def read_serial_data(self):
        try:
            received_data = self.serial_port.read(256).decode("utf-8")
            if received_data:
                self.received_data += received_data

                sentence_end = self.received_data.find("\r\n")
                while sentence_end != -1:
                    sentence = self.received_data[:sentence_end]
                    self.received_data = self.received_data[sentence_end + 2 :]
                    sentence_end = self.received_data.find("\r\n")
                    self.parse_gga(sentence)

        except serial.SerialException as e:
            print("Error reading serial data:", str(e))

    def parse_gga(self, sentence):
        tokens = sentence.split(",")
        if len(tokens) >= 15 and tokens[0] == "$GPGGA":
            self.gps_pub_data = [
                tokens[0],
                tokens[1],
                tokens[2],
                tokens[3],
                tokens[4],
                tokens[5],
                tokens[6],
                tokens[7],
                tokens[8],
                tokens[9],
                tokens[10],
                tokens[11],
                tokens[12],
                tokens[13],
                tokens[14],
            ]

    def publish_gps_data(self):
        self.gps_pub_msg = rosarray()
        self.gps_pub_msg.data = self.gps_pub_data
        self.gps_pub.publish(self.gps_pub_msg)
        self.gps_pub_data = self.gps_pub_msg.data
        print("PUB to", self.gps_node, ":", self.gps_pub_data)


def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS_NODE("gps_node")
    gps_node.init_gps_publisher("gps_data", 1, "/dev/ttyS1", 9600)

    while rclpy.ok():
        rclpy.spin_once(gps_node)

    gps_node.close()
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
