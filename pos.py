#!/usr/bin/env python3

import sys
sys.path.append("../fanuc_ethernet_ip_drivers/src")
from robot_controller import robot

# robot_ip_a = "10.8.4.16"
robot_ip_b = "10.8.4.6"

def main():
    # rb_a = robot(robot_ip_a)
    rb_b = robot(robot_ip_b)
    # print(f"rb_a coords: {rb_a.read_current_cartesian_pose()}")
    print(f"rb_b coords: {rb_b.read_current_cartesian_pose()}")

if __name__ == "__main__":
    main()
