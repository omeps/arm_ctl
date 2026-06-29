import rclpy
import time
import argparse
import numpy as np
import json
from rclpy.node import Node
import dynamixel_sdk
import std_msgs.msg as msg
id_base = 5
id_linkage_a = 6 
id_linkage_b = 7
id_claw = 8
class ArmModelPublisher(Node):
    # MAKE SURE `dir' ends with a / to keep topics in 1 directory
    def __init__(self, dir="arm_ctl/", timer_period=0.03, device_name='/dev/ttyUSB0'):
        super().__init__('arm_model_publisher',parameter_overrides=[])
        self.declare_parameter('a', 0)
        self.declare_parameter('b', 0)
        self.declare_parameter('base', 0)
        self.declare_parameter('claw', 0)
        self.publish_orientation_base = self.create_publisher(
            msg.Int32, dir + "base", 1)
        self.publish_orientation_linkage_a = self.create_publisher(
            msg.Int32, dir + "linkage_a", 1)
        self.publish_orientation_linkage_b = self.create_publisher(
            msg.Int32, dir + "linkage_b", 1)
        self.publish_orientation_claw = self.create_publisher(
            msg.Int32, dir + "claw", 1)
        self.subscription_reposition = self.create_subscription(
                msg.String,
                dir + 'reposition',
                self.reposition,
                1,
        )
        self.publishing = True
        self.subscription_toggle = self.create_subscription(
                msg.Bool,
                dir + 'publish',
                self.toggle,
                1,
        )
        self.timer = self.create_timer(timer_period, self.publish_motor_data)

        self.port_handler = dynamixel_sdk.PortHandler(device_name)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)
        self.port_handler.openPort() or quit()
        self.port_handler.setBaudRate(1000000)

        toggle_torque = 64
        for id in [id_base, id_linkage_a, id_linkage_b,id_claw]:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 0) # turn motors off so they can be moved

    def publish_motor_data(self):
        if not self.publishing: return
        global id_base,id_linkage_a, id_linkage_b, id_claw
        present_position = 132
        orientation_base = self.packet_handler.read4ByteTxRx(self.port_handler, id_base, present_position)[0]
        if (orientation_base > 2 ** 31): orientation_base -= 2 ** 32
        orientation_linkage_a = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_a, present_position)[0]
        if (orientation_linkage_a > 2 ** 31): orientation_linkage_a -= 2 ** 32
        orientation_linkage_b = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_b, present_position)[0]
        if (orientation_linkage_b > 2 ** 31): orientation_linkage_b -= 2 ** 32
        orientation_claw = self.packet_handler.read4ByteTxRx(self.port_handler, id_claw, present_position)[0]
        if (orientation_claw > 2 ** 31): orientation_claw -= 2 ** 32
        payload = msg.Int32()
        payload.data = orientation_base - self.get_parameter('base').get_parameter_value().integer_value
        self.publish_orientation_base.publish(payload)
        payload.data = orientation_linkage_a - self.get_parameter('a').get_parameter_value().integer_value
        self.publish_orientation_linkage_a.publish(payload)
        payload.data = orientation_linkage_b - self.get_parameter('b').get_parameter_value().integer_value
        self.publish_orientation_linkage_b.publish(payload)
        payload.data = orientation_claw - self.get_parameter('claw').get_parameter_value().integer_value
        self.publish_orientation_claw.publish(payload)
    def reposition(self, payload):
        data = json.loads(payload.data)

        present_position = 132
        orientation_base = self.packet_handler.read4ByteTxRx(self.port_handler, id_base, present_position)[0]
        if (orientation_base > 2 ** 31): orientation_base -= 2 ** 32
        orientation_linkage_a = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_a, present_position)[0]
        if (orientation_linkage_a > 2 ** 31): orientation_linkage_a -= 2 ** 32
        orientation_linkage_b = self.packet_handler.read4ByteTxRx(self.port_handler, id_linkage_b, present_position)[0]
        if (orientation_linkage_b > 2 ** 31): orientation_linkage_b -= 2 ** 32
        orientation_claw = self.packet_handler.read4ByteTxRx(self.port_handler, id_claw, present_position)[0]
        if (orientation_claw > 2 ** 31): orientation_claw -= 2 ** 32

        toggle_torque = 64
        for id in [id_base, id_linkage_a, id_linkage_b, id_claw]:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 1) 
        set_position = 116
        goal_base = data['base'] + self.get_parameter('base').get_parameter_value().integer_value
        self.packet_handler.write4ByteTxRx(
                self.port_handler, 
                id_base, 
                set_position,
                (goal_base - orientation_base + 2048) % 4096 - 2048 + orientation_base,
        )
        goal_a = data['a'] + self.get_parameter('a').get_parameter_value().integer_value
        self.packet_handler.write4ByteTxRx(
                self.port_handler, 
                id_linkage_a, 
                set_position,
                (goal_a - orientation_linkage_a + 2048) % 4096 - 2048 + orientation_linkage_a,
                
        )
        goal_b = data['b'] + self.get_parameter('b').get_parameter_value().integer_value
        self.packet_handler.write4ByteTxRx(
                self.port_handler, 
                id_linkage_b, 
                set_position,
                (goal_b - orientation_linkage_b + 2048) % 4096 - 2048 + orientation_linkage_b,
        )
        goal_claw = data['claw'] + self.get_parameter('claw').get_parameter_value().integer_value
        self.packet_handler.write4ByteTxRx(
                self.port_handler, 
                id_claw, 
                set_position,
                (goal_claw - orientation_claw + 2048) % 4096 - 2048 + orientation_claw,
        )
        time.sleep(3.0)
        for id in [id_base, id_linkage_a, id_linkage_b, id_claw]:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, toggle_torque, 0) 
    def toggle(self,payload):
        self.publishing = payload.data;

        

def main(args=None):
    rclpy.init(args=args)
    publisher = ArmModelPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
