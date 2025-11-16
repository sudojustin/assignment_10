#!/usr/bin/env python3

from standardbots import StandardBotsRobot, models

ROBOT_IP = "10.8.4.11"
HOME_POS = [0, 0, -1.57, 0, 1.57, -3.14]

sdk = StandardBotsRobot(
    url='http://10.8.4.11:3000',
    token='8geqfqu0-qbbkig-ozwgr4-tl2xfj7',
    robot_kind=StandardBotsRobot.RobotKind.Live,
)

def move_robot_joint(j):
    with sdk.connection():
        sdk.movement.brakes.unbrake().ok()
        arm_rotations = models.ArmJointRotations(joints=(j[0], j[1], j[2], j[3], j[4], j[5]))
        position_request = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=arm_rotations,
		)
        sdk.movement.position.set_arm_position(body=position_request).ok()

def gripper_request(WIDTH, FORCE):
    with sdk.connection():
        response = sdk.equipment.control_gripper(
            models.GripperCommandRequest(
                kind=models.GripperKindEnum.Onrobot2Fg14,
                onrobot_2fg14=models.OnRobot2FG14GripperCommandRequest(
                    grip_direction=models.LinearGripDirectionEnum.Inward,
                    target_grip_width=models.LinearUnit(
                        value=WIDTH, unit_kind=models.LinearUnitKind.Meters
					),
                    target_force=models.ForceUnit(
                        value=FORCE,
                        unit_kind=models.ForceUnitKind.Newtons,
					),
                    control_kind=models.OnRobot2FG14ControlKindEnum.Move,
				),
			)
		)
    try:
        data = response.ok()
    except Exception:
        print(response.data.message)

def gripper_command(STRING):
    if STRING == "CLOSE":
        gripper_request(0.011, 10.0)
    if STRING == "OPEN":
        gripper_request(0.078, 10.0)


def main():
    move_robot_joint(HOME_POS)
    gripper_command("OPEN")

if __name__ == "__main__":
    main()

