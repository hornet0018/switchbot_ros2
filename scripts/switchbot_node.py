#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8, Float64

import binascii
from bluepy.btle import Scanner, DefaultDelegate

#macaddr = 'd6:03:21:1e:72:d1'
#macaddr = 'e2:a4:0f:6a:8b:79'

class ScanDelegate(DefaultDelegate):
    def __init__(self, node):
        DefaultDelegate.__init__(self)
        self.node = node

    def handleDiscovery(self, dev, isNewDev, isNewData):
        #rospy.loginfo("Discovered device: {}".format(dev.addr))
        if dev.addr == self.node.macaddr:  # パラメータを使用
            for (adtype, desc, value) in dev.getScanData():
                if adtype == 22:
                    servicedata = binascii.unhexlify(value[4:])

                    battery = UInt8()
                    battery.data = ord(servicedata[2]) & 0b01111111


                    isTemperatureAboveFreezing = ord(servicedata[4]) & 0b10000000

                    temperature = Float64()
                    temperature.data = (ord(servicedata[3]) & 0b00001111) / 10.0 + (ord(servicedata[4]) & 0b01111111)

                    if not isTemperatureAboveFreezing:
                        temperature.data = -temperature.data

                    humidity = UInt8()
                    humidity.data = ord(servicedata[5]) & 0b01111111

                    # データをそれぞれのトピックに公開
                    self.node.battery_publisher.publish(battery)
                    self.node.temperature_publisher.publish(temperature)
                    self.node.humidity_publisher.publish(humidity)

                    # 標準出力にデータを出力
                    rospy.loginfo('Published battery: {}'.format(battery.data))
                    rospy.loginfo('Published temperature: {}'.format(temperature.data))
                    rospy.loginfo('Published humidity: {}'.format(humidity.data))

class BluetoothScannerNode:
    def __init__(self):
        rospy.init_node('bluetooth_scanner', anonymous=True)
        # macaddr パラメータを取得（デフォルト値も設定）
        self.macaddr = rospy.get_param('~macaddr', 'd6:03:21:1e:72:d1')
        self.battery_publisher = rospy.Publisher('switchbot/battery', UInt8, queue_size=10)
        self.temperature_publisher = rospy.Publisher('switchbot/temperature', Float64, queue_size=10)
        self.humidity_publisher = rospy.Publisher('switchbot/humidity', UInt8, queue_size=10)

        self.scanner = Scanner().withDelegate(ScanDelegate(self))
        rospy.Timer(rospy.Duration(1), self.scan)

    def scan(self, event):
        self.scanner.scan(0)

if __name__ == '__main__':
    node = BluetoothScannerNode()
    rospy.spin()
