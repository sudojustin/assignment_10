#!/usr/bin/env python3

import time
import sys
sys.path.append("../fanuc_ethernet_ip_drivers/src")
from robot_controller import robot

ROBOT_IP_A = "10.8.4.16"
ROBOT_IP_B = "10.8.4.6"
HOME_POS = [0, 0, 0, 0, -90, -45]

# Parameters to open onRobot gripper
OPEN_GRIPPER_PARAMS = {
    "width_in_mm":138,
    "force_in_newtons":40,
    "wait":True
}

# Parameters to close onRobot gripper
CLOSE_GRIPPER_PARAMS = {
    "width_in_mm":79,
    "force_in_newtons":40,
    "wait":True
}

# rb_a coords: [223.48193359375, 891.1599731445312, 286.2806091308594, -96.1849136352539, -31.8389949798584, 4.006237030029297]
# rb_b coords: [260.37615966796875, -892.7490844726562, 256.4281005859375, 115.22403717041016, 89.51708984375, 20.2786922454834]
# rb.write_cartesian_position(pos1)

def main():

    # rb_a = robot(ROBOT_IP_A)
    rb_b = robot(ROBOT_IP_B)

    # Open gripper
    rb_b.onRobot_gripper(**OPEN_GRIPPER_PARAMS)
    # rb_b.onRobot_gripper(**CLOSE_GRIPPER_PARAMS)
    time.sleep(0.5)

    # Set robot speed to 300
    rb_b.set_speed(300)

    # inter = [127.65243530273438, -939.7383422851562, 42.84657287597656, -55.654029846191406, -88.24526977539062, 143.8533935546875]
    # rb_b.write_cartesian_position(new_inter)

    rb_b.write_joint_pose(HOME_POS)

    # inter = [301.5372314453125, -295.48968505859375, 295.58026123046875, 174.59715270996094, 2.542015552520752, 92.2433853149414]
    # rb_b.write_cartesian_position(inter)

    # rb_b.write_joint_pose(HOME_POS)

if __name__ == '__main__':
    main()

