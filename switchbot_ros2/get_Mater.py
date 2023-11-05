import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # String message type を使います
import json  # JSON エンコードのためのライブラリ
import binascii
from bluepy.btle import Scanner, DefaultDelegate

macaddr = 'd6:03:21:1e:72:d1'

class ScanDelegate(DefaultDelegate):
    def __init__(self, node):
        DefaultDelegate.__init__(self)
        self.node = node

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr == macaddr:
            for (adtype, desc, value) in dev.getScanData():
                if adtype == 22:
                    servicedata = binascii.unhexlify(value[4:])
                    battery_level = float(servicedata[2] & 0b01111111)
                    isTemperatureAboveFreezing = servicedata[4] & 0b10000000
                    temperature = (servicedata[3] & 0b00001111) / 10 + (servicedata[4] & 0b01111111)
                    if not isTemperatureAboveFreezing:
                        temperature = -temperature
                    humidity = float(servicedata[5] & 0b01111111)

                    # Create a JSON string
                    data_json = {
                        'temperature': temperature,
                        'humidity': humidity,
                        'battery_level': battery_level
                    }
                    data_str = json.dumps(data_json)

                    # Publish the data
                    data_msg = String(data=data_str)
                    self.node.data_publisher.publish(data_msg)

                    # Print to standard output
                    self.node.get_logger().info(f'Published data: {data_str}')

class BluetoothScannerNode(Node):
    def __init__(self):
        super().__init__('bluetooth_scanner')
        # Initialize the publisher
        self.data_publisher = self.create_publisher(String, 'switchbot/data', 10)  # Float64MultiArray -> String
        self.scanner = Scanner().withDelegate(ScanDelegate(self))
        self.timer = self.create_timer(1.0, self.scan)

    def scan(self):
        self.scanner.scan(0)

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
