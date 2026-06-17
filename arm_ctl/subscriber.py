import rclpy
import numpy as np
import time
def overflow_add(a,b):

    return int(np.array(a,dtype=np.int32) + np.array(b,dtype=np.int32))
import argparse
from rclpy.node import Node
import dynamixel_sdk
import std_msgs.msg as msg
id_base = 5
id_linkage_a = 6 
id_linkage_b = 7

class ArmSubscriber(Node):
    # MAKE SURE `dir' ends with a / to keep topics in 1 directory
    def __init__(self, dir="arm_ctl/", device_name='/dev/ttyUSB0'):
        super().__init__('arm_subscriber',parameter_overrides=[])
        self.declare_parameter('a', 0)
        self.declare_parameter('b', 0)
        self.declare_parameter('base', 0)
        self.port_handler = dynamixel_sdk.PortHandler(device_name)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)
        self.port_handler.openPort() or quit()
        self.port_handler.setBaudRate(1000000)
        toggle_torque = 64
        for id in [id_base, id_linkage_a, id_linkage_b]:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 0) # turn motors off to configure EEPROM
            self.packet_handler.write4ByteTxRx(self.port_handler, id, 112, 100) # turn motors off to configure EEPROM
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 1) # turn motors on
        self.subscription_base = self.create_subscription(
            msg.Int32,
            dir + 'base',
            self.move_base,
            1
        )
        self.publish_err_base = self.create_publisher(msg.Bool, dir + "err/base", 1)
        self.subscription_linkage_a = self.create_subscription(
            msg.Int32,
            dir + 'linkage_a',
            self.move_linkage_a,
            1
        )
        self.publish_err_linkage_a = self.create_publisher(msg.Bool, dir + "err/linkage_a", 1)
        self.subscription_linkage_b = self.create_subscription(
            msg.Int32,
            dir + 'linkage_b',
            self.move_linkage_b,
            1
        )
        self.publish_err_linkage_b  = self.create_publisher(msg.Bool, dir + "err/linkage_b", 1)
        self.subscription_reset = self.create_subscription(
            msg.Empty,
            dir + 'reset',
            self.reset,
            1
        )

    def move_base(self, payload):
        goal_position = 116
        err = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_base,
            goal_position,
            payload.data + self.get_parameter('base').get_parameter_value().integer_value
        )[1]
        payload = msg.Bool()
        payload.data = err != 0
        self.publish_err_base.publish(payload)
    def move_linkage_a(self, payload):
        goal_position = 116
        err = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_linkage_a,
            goal_position,
            payload.data + self.get_parameter('a').get_parameter_value().integer_value
        )[1]
        payload = msg.Bool()
        payload.data = err != 0
        self.publish_err_linkage_a.publish(payload)

    def move_linkage_b(self, payload):
        goal_position = 116
        err = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_linkage_b,
            goal_position,
            payload.data + self.get_parameter('b').get_parameter_value().integer_value
        )[1]
        payload = msg.Bool()
        payload.data = err != 0
        self.publish_err_linkage_b.publish(payload)
    def reset(self, payload):
        toggle_torque = 64
        for i in [id_base, id_linkage_a, id_linkage_b]:
            self.packet_handler.reboot(self.port_handler, i)
        for id in [id_base, id_linkage_a, id_linkage_b]:
            while self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 0)[0] != 0: time.sleep(0.01) # turn motors off to configure EEPROM
            self.packet_handler.write4ByteTxRx(self.port_handler, id, 112, 100) # turn motors off to configure EEPROM
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 1) # turn motors on
def main(args=None):
    rclpy.init(args=args)
    subscriber = ArmSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
