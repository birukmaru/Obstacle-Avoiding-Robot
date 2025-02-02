#!/usr/bin/env python3

import os
import sys
import select
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

msg = """
Control Your Robot:
---------------------------
Moving around:
        W
   A    S    D
        X

W/X : Increase/Decrease forward speed
A/D : Turn left/right
S   : Stop
CTRL-C to quit
"""

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.2


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'teleop_cmd_vel', qos)

    target_linear_velocity = 0.1
    target_angular_velocity = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity += LIN_VEL_STEP_SIZE
            elif key == 'x':
                target_linear_velocity -= LIN_VEL_STEP_SIZE
            elif key == 'a':
                target_angular_velocity += ANG_VEL_STEP_SIZE
            elif key == 'd':
                target_angular_velocity -= ANG_VEL_STEP_SIZE
            elif key == 's':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
            elif key == '\x03':
                break

            twist = Twist()
            twist.linear.x = target_linear_velocity
            twist.angular.z = target_angular_velocity
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

