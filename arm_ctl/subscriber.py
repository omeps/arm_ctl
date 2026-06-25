import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8
import numpy as np
import time
def overflow_add(a,b):

    return int(np.array(a,dtype=np.int32) + np.array(b,dtype=np.int32))
import argparse
import dynamixel_sdk
import json
import std_msgs.msg as msg
id_base = 5
id_linkage_a = 6 
id_linkage_b = 7
id_claw = 8
FRONT_LEFT_ADDR = 0
BACK_LEFT_ADDR = 2
FRONT_RIGHT_ADDR = 1
BACK_RIGHT_ADDR = 3

BROAD_PERCENT = .2

TIGHT_PERCENT = .6

class ArmSubscriber(Node):
    # MAKE SURE `dir' ends with a / to keep topics in 1 directory
    def __init__(self, dir="arm_ctl/", device_name='/dev/ttyUSB0'):
        super().__init__('arm_subscriber',parameter_overrides=[])
        self.declare_parameter('a', 0)
        self.declare_parameter('b', 0)
        self.declare_parameter('base', 0)
        self.declare_parameter('claw', 0)
        self.dyna_controller = DynamixelController("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TFUT7-if00-port0")
        self.base = Dynamixel("base", id_base, self.dyna_controller, "arm")
        self.claw = Dynamixel("claw", id_claw, self.dyna_controller, "arm")
        self.linkage_a = Dynamixel("linkage_a", id_linkage_a, self.dyna_controller, "arm")
        self.linkage_b = Dynamixel("linkage_b", id_linkage_b, self.dyna_controller, "arm")
        self.accept_messages = True
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
        self.subscription_claw = self.create_subscription(
            msg.Int32,
            dir + 'claw',
            self.move_claw,
            1
        )
        self.publish_err_claw = self.create_publisher(msg.Bool, dir + "err/claw", 1)
        self.reposition = self.create_publisher(String, dir + 'reposition', 1)

        self.front_left = Dynamixel("front_left", FRONT_LEFT_ADDR, self.dyna_controller, "wheel")
        self.back_left = Dynamixel("back_left", BACK_LEFT_ADDR, self.dyna_controller, "wheel")
        self.front_right = Dynamixel("front_right", FRONT_RIGHT_ADDR, self.dyna_controller, "wheel")
        self.back_right = Dynamixel("back_right", BACK_RIGHT_ADDR, self.dyna_controller, "wheel")

        self.speed_mult = 0.5

        self.motors_arr = [self.front_left, self.back_left, self.front_right, self.back_right]

        self.base_motion_sub = self.create_subscription(
            String,
            '/motor_states/drive',
            self.drive_direction,
            1)
        self.base_motion_sub

        self.speed_sub = self.create_subscription(
            Int8,
            '/speed',
            self.get_speed,
            1)
        self.speed_sub

        self.direction = "still"

        # timer_period = 1/30
        # self.timer = self.create_timer(timer_period, self.send_motor_cmds)
    def move_base(self, payload):
        if not self.accept_messages: return
        print('move_base')
        message = msg.Bool()
        message.data = self.base.set_position(-payload.data + self.get_parameter('base').get_parameter_value().integer_value)
        self.publish_err_base.publish(message)
    def move_claw(self, payload):
        if not self.accept_messages: return
        print('move_claw')
        message = msg.Bool()
        message.data = self.claw.set_position(payload.data + self.get_parameter('claw').get_parameter_value().integer_value)
        self.publish_err_claw.publish(message)
    def move_linkage_a(self, payload):
        if not self.accept_messages: return
        print('move_linkage_a')
        message = msg.Bool()
        message.data = self.linkage_a.set_position(payload.data + self.get_parameter('a').get_parameter_value().integer_value)
        self.publish_err_linkage_a.publish(message)
    def move_linkage_b(self, payload):
        if not self.accept_messages: return
        print('move_linkage_b')
        message = msg.Bool()
        message.data = self.linkage_b.set_position(-payload.data + self.get_parameter('b').get_parameter_value().integer_value)
        self.publish_err_linkage_b.publish(message)
    def reset(self, payload):
        print('reset')
        self.accept_messages = False
        toggle_torque = 64
        message = msg.String()
        message.data = json.dumps({
            'base': -(self.base.reboot() - self.get_parameter('base').get_parameter_value().integer_value),
            'claw': (self.claw.reboot() - self.get_parameter('claw').get_parameter_value().integer_value),
            'a': (self.linkage_a.reboot() - self.get_parameter('a').get_parameter_value().integer_value),
            'b': -(self.linkage_b.reboot() - self.get_parameter('b').get_parameter_value().integer_value),
        })
        self.reposition.publish(message)
        self.accept_timer = self.create_timer(3.0, self.accept_again)
    def accept_again(self):
        print('accept_again')
        self.accept_messages = True
        self.accept_timer.cancel()
    def drive_direction(self, direction_msg):
        direction = direction_msg.data

        if self.direction != direction:
            self.direction = direction
        
            if direction == "forward":
                self.front_left.direction_mult = 1.0
                self.back_left.direction_mult = 1.0
                self.front_right.direction_mult = -1.0
                self.back_right.direction_mult = -1.0
            elif direction == "reverse":
                self.front_left.direction_mult = -1.0
                self.back_left.direction_mult = -1.0
                self.front_right.direction_mult = 1.0
                self.back_right.direction_mult = 1.0
            elif direction == "forward_left":
                self.front_left.direction_mult = 1.0*BROAD_PERCENT
                self.back_left.direction_mult = 1.0*BROAD_PERCENT
                # self.front_left.direction_mult = 0
                # self.back_left.direction_mult = 0
                self.front_right.direction_mult = -1.0
                self.back_right.direction_mult = -1.0
            elif direction == "forward_right":
                self.front_left.direction_mult = 1.0
                self.back_left.direction_mult = 1.0
                self.front_right.direction_mult = -1.0*BROAD_PERCENT
                self.back_right.direction_mult = -1.0*BROAD_PERCENT
                # self.front_right.direction_mult = 0
                # self.back_right.direction_mult = 0
            elif direction == "reverse_left":
                self.front_left.direction_mult = -1.0*BROAD_PERCENT
                self.back_left.direction_mult = -1.0*BROAD_PERCENT
                # self.front_left.direction_mult = 0
                # self.back_left.direction_mult = 0
                self.front_right.direction_mult = 1.0
                self.back_right.direction_mult = 1.0
            elif direction == "reverse_right":
                self.front_left.direction_mult = -1.0
                self.back_left.direction_mult = -1.0
                # self.front_right.direction_mult = 0
                # self.back_right.direction_mult = 0
                self.front_right.direction_mult = 1.0*BROAD_PERCENT
                self.back_right.direction_mult = 1.0*BROAD_PERCENT
            elif direction == "left":
                self.front_left.direction_mult = -TIGHT_PERCENT
                self.back_left.direction_mult = -TIGHT_PERCENT
                self.front_right.direction_mult = -TIGHT_PERCENT
                self.back_right.direction_mult = -TIGHT_PERCENT
            elif direction == "right":
                self.front_left.direction_mult = TIGHT_PERCENT
                self.back_left.direction_mult = TIGHT_PERCENT
                self.front_right.direction_mult = TIGHT_PERCENT
                self.back_right.direction_mult = TIGHT_PERCENT
            else:
                # direction == "still"
                self.front_left.direction_mult = 0.0
                self.back_left.direction_mult = 0.0
                self.front_right.direction_mult = 0.0
                self.back_right.direction_mult = 0.0
            
            self.send_motor_cmds()
    
    def send_motor_cmds(self):
        for motor in self.motors_arr:
            # send motor commands
            speed = motor.direction_mult*self.speed_mult
            print(f"Set {motor.name} speed: {speed}")
            motor.set_speed(speed)

    def get_speed(self, speed_msg):
        speed = self.speed_mult*100
        if speed != speed_msg.data:
            self.speed_mult = speed_msg.data/100
            self.send_motor_cmds()


def main(args=None):
    rclpy.init(args=args)
    subscriber = ArmSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
class DynamixelController:
    def __init__(self,dev: str):
        self.port_handler = dynamixel_sdk.PortHandler(dev)
        self.packet_handler = dynamixel_sdk.PacketHandler(2.0)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(1000000)
    def toggle_torque(self, addr: int, value: int):
        return self.packet_handler.write1ByteTxRx(self.port_handler, addr, 64, value)[0]
    def set_mode_run(self, addr: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, addr, 11, 1)
    def set_mode_position(self, addr: int):
        self.packet_handler.write1ByteTxRx(self.port_handler, addr, 11, 4)
    def read_position(self, addr: int):
        pos = self.packet_handler.read4ByteTxRx(self.port_handler, addr, 132)[0] # turn motors on
        if (pos > 2 ** 31): pos -= 2 ** 32
        pos = pos % 4096;
        return pos
    # returns true on error
    def set_speed(self, addr: int, speed: float):
        return self.packet_handler.write4ByteTxRx(self.port_handler, addr, 104, int(speed * 265))[1] != 0
    # returns true on error
    def set_position(self, addr: int, goal_position: int):
        return self.packet_handler.write4ByteTxRx(self.port_handler, addr, 116, goal_position)[1] != 0
    def reboot(self,addr: int):
        self.packet_handler.reboot(self.port_handler, addr)
        while self.toggle_torque(addr, 1) != 0: time.sleep(0.01)
        pos = self.read_position(addr)
        return pos
class Dynamixel:
    def __init__(self, name: str, motor_addr: int, controller: DynamixelController, ty: str):
        self.controller = controller
        self.name = name
        self.motor_addr = motor_addr
        self.controller.toggle_torque(self.motor_addr, 0)
        if ty == "wheel":
            self.controller.set_mode_run(self.motor_addr)
        elif ty == "arm":
            self.controller.set_mode_position(self.motor_addr)
            self.last_position=self.controller.read_position(self.motor_addr)
        else:
            raise ValueError("ty must be \"arm\" or \"wheel\"")
        self.controller.toggle_torque(self.motor_addr, 1)
        self.direction_mult = 0.0
    # returns true on error
    def set_speed(self, speed: float):
        return self.controller.set_speed(self.motor_addr, speed)
    # returns true on error
    def set_position(self, position: int):
        bounded_position = (position - self.last_position + 2048) % 4096 + self.last_position - 2048
        self.last_position = bounded_position
        return self.controller.set_position(self.motor_addr, bounded_position)
    def reboot(self):
        self.last_position = self.controller.reboot(self.motor_addr)
        return self.last_position
