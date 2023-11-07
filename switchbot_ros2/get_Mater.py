import rclpy
from rclpy.node import Node

# 必要なメッセージ型をインポート
from std_msgs.msg import Float64, UInt8

import binascii
from bluepy.btle import Scanner, DefaultDelegate

macaddr = 'd6:03:21:1e:72:d1'

class ScanDelegate(DefaultDelegate):
    def __init__(self, node):
        DefaultDelegate.__init__(self)
        self.node = node

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr == macaddr:
            for ( adtype, desc, value ) in dev.getScanData():
                if adtype == 22:
                    servicedata = binascii.unhexlify(value[4:])

                    battery = UInt8()
                    battery.data = servicedata[2] & 0b01111111

                    isTemperatureAboveFreezing = servicedata[4] & 0b10000000
                    temperature = Float64()
                    temperature.data = (servicedata[3] & 0b00001111) / 10 + (servicedata[4] & 0b01111111)
                    if not isTemperatureAboveFreezing:
                        temperature.data = -temperature.data

                    humidity = UInt8()
                    humidity.data = servicedata[5] & 0b01111111


                    # データをそれぞれのトピックに公開
                    self.node.battery_publisher.publish(battery)
                    self.node.temperature_publisher.publish(temperature)
                    self.node.humidity_publisher.publish(humidity)

                    # 標準出力にデータを出力
                    self.node.get_logger().info(f'Published battery: {battery.data}')
                    self.node.get_logger().info(f'Published temperature: {temperature.data}')
                    self.node.get_logger().info(f'Published humidity: {humidity.data}')

class BluetoothScannerNode(Node):
    def __init__(self):
        super().__init__('bluetooth_scanner')
        # パブリッシャの初期化
        self.battery_publisher = self.create_publisher(UInt8, 'switchbot/battery', 10)
        self.temperature_publisher = self.create_publisher(Float64, 'switchbot/temperature', 10)
        self.humidity_publisher = self.create_publisher(UInt8, 'switchbot/humidity', 10)

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