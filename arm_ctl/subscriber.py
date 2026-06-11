import rclpy
import numpy as np
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
            msg.UInt32,
            dir + 'base',
            self.move_base,
            1
        )
        self.subscription_linkage_a = self.create_subscription(
            msg.UInt32,
            dir + 'linkage_a',
            self.move_linkage_a,
            1
        )
        self.subscription_linkage_b = self.create_subscription(
            msg.UInt32,
            dir + 'linkage_b',
            self.move_linkage_b,
            1
        )

    def move_base(self, payload):
        goal_position = 116
        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_base,
            goal_position,
            payload.data + positions['base']
        )
    def move_linkage_a(self, payload):
        goal_position = 116
        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_linkage_a,
            goal_position,
            payload.data + positions['a']
        )
    def move_linkage_b(self, payload):
        goal_position = 116
        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            id_linkage_b,
            goal_position,
            payload.data + positions['b']
        )

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", help="ros topic directory", default="arm_ctl")
    parser.add_argument("--period", help="how long to publish values (in seconds)", default="0.03")
    parser.add_argument("--device_name", help="device file location", default="/dev/ttyUSB0")
    args = parser.parse_args()
    if args.dir[-1] != '/': args.dir += '/'
    subscriber = ArmSubscriber(dir=args.dir, device_name=args.device_name)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
