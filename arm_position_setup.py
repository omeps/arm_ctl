import dynamixel_sdk
import json
print("tty path: ")
path = input()
print('opening port handler...')
port_handler = dynamixel_sdk.PortHandler(path)
port_handler.openPort() or quit()
port_handler.setBaudRate(1000000)
packet_handler = dynamixel_sdk.PacketHandler(2.0)
present_position = 132
def get_position(mid: int) -> int:
    position = 0
    while input() != '':
        position = packet_handler.read4ByteTxRx(port_handler, mid, present_position)[0]
        if (position > 2 ** 31): position -= 2 ** 32
        print(position, end='\r')
    return position
positions = {}
for (link,mid) in [('base', 5), ('a', 6), ('b', 7), ('claw', 8)]:
    print(f'determining link {link} ... ')
    positions[link] = get_position(mid)
with open('launch/subscriber.py','w') as conf:
    conf.write(f'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_ctl',
            executable='arm',
            name='subscriber',
            output='screen',
            emulate_tty=True,
            parameters=[
                {{'a': {positions['a']}}},
                {{'b': {positions['b']}}},
                {{'base': {positions['base']}}},
                {{'claw': {positions['claw']}}},
            ]
        )
    ])
''')
with open('launch/publisher.py','w') as conf:
    conf.write(f'''
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_ctl',
            executable='arm_model',
            name='publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {{'a': {positions['a']}}},
                {{'b': {positions['b']}}},
                {{'claw': {positions['claw']}}},
            ]
        )
    ])
''')
