import rclpy
import argparse
import numpy as np
from rclpy.node import Node
import dynamixel_sdk
import std_msgs.msg as msg
id_base = 5
id_linkage_a = 6 
id_linkage_b = 7

class ArmModelPublisher(Node):
    # MAKE SURE `dir' ends with a / to keep topics in 1 directory
    def __init__(self, dir="arm_ctl/", timer_period=0.03, device_name='/dev/ttyUSB0'):
        super().__init__('arm_model_publisher',parameter_overrides=[])
        self.publish_orientation_base = self.create_publisher(
            msg.Int32, dir + "base", 1)
        self.publish_orientation_linkage_a = self.create_publisher(
            msg.Int32, dir + "linkage_a", 1)
        self.publish_orientation_linkage_b = self.create_publisher(
            msg.Int32, dir + "linkage_b", 1)
        self.timer = self.create_timer(timer_period, self.publish_motor_data)

        self.port_handler = dynamixel_sdk.PortHandler(device_name)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)
        self.port_handler.openPort() or quit()
        self.port_handler.setBaudRate(1000000)
        toggle_torque = 64
        for id in [id_base, id_linkage_a, id_linkage_b]:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 0) # turn motors off so they can be moved

    def publish_motor_data(self):
        global id_base,id_linkage_a, id_linkage_b
        present_position = 132
        orientation_base = self.packet_handler.read4ByteTxRx(self.port_handler, id_base, present_position)[0]
        if (orientation_base > 2 ** 31): orientation_base -= 2 ** 32
        orientation_linkage_a = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_a, present_position)[0]
        if (orientation_linkage_a > 2 ** 31): orientation_linkage_a -= 2 ** 32
        orientation_linkage_b = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_b, present_position)[0]
        if (orientation_linkage_b > 2 ** 31): orientation_linkage_b -= 2 ** 32
        payload = msg.Int32()
        payload.data = orientation_base - positions['base']
        self.publish_orientation_base.publish(payload)
        payload.data = orientation_linkage_a - positions['a']
        self.publish_orientation_linkage_a.publish(payload)
        payload.data = orientation_linkage_b - positions['b']
        self.publish_orientation_linkage_b.publish(payload)

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", help="ros topic directory", default="arm_ctl")
    parser.add_argument("--period", help="how long to publish values (in seconds)", default="0.03")
    parser.add_argument("--device_name", help="device file location", default="/dev/ttyUSB0")
    args = parser.parse_args()
    if args.dir[-1] != '/': args.dir += '/'
    publisher = ArmModelPublisher(dir=args.dir, timer_period=float(args.period), device_name=args.device_name)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
