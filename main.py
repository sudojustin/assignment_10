#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import mvsdk
from paho.mqtt import client as mqtt_client
import platform
import json
import sys
import time
sys.path.append("../fanuc_ethernet_ip_drivers/src")
from robot_controller import robot
# import robot_controller
# import paho.mqtt.client as mqtt

BROKER = "localhost"          # Broker being hosted on my machine
PORT = 1883                   # Default port for MQTT
TOPIC_A = "python/mqtt/a"     # Topic for Robot A
TOPIC_B = "python/mqtt/b"     # Topic for Robot B
CLIENT_ID_PUB = "pub-justin"  # Client ID for publishing
CLIENT_ID_SUB = "sub-justin"  # Client ID for subscribing

YELLOW_LOWER = (10, 40, 40)
YELLOW_UPPER = (45, 255, 255)
ROBOT_IP = "10.8.4.6"
HOME_POS = [0, 0, 0, 0, -90, -45]
# subtract 80ish from x
DROP_OFF_POS = [676.0311279296875, -189.9359588623047, 39.56829833984375, -179.99996948242188, -2.994090027641505e-05, -45.00004959106445]

Z_AXIS = 103.27935791015625
Z_AXIS_STAGING = 159.27935791015625
W = 179.89998474121094
P = -2.48556552833179e-05
R = -45.0  # must be between -90 and 0

# Function to initialize and connect an MQTT client
def connect_mqtt(client_id):
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id=client_id)

    client.on_connect = on_connect
    client.connect(BROKER, PORT)  # Connect using localhost and port 1883
    return client

# Function to publish messages on a given topic
def publish(client, msg):
    msg_count = 1
    while True:
        time.sleep(1)
        # msg = f"messages: {msg_count}"
        result = client.publish(TOPIC_B, msg, retain=False)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{TOPIC_B}`")
        else:
            print(f"Failed to send message to topic {TOPIC_B}")
        msg_count += 1
        if msg_count > 5:  # Publish 5 messages to ensure subscriber recieves messages
            break

# Function to subscribe to messages in a loop
def wait_for_message(client, topic):
    message = {"payload": None}

    # Drain any old messages to ensure stale data isn't used
    client.loop(timeout=0.5)
    time.sleep(0.2)

    def on_message(client, userdata, msg):
        if msg.retain:  # Ignore retained messages left over in the buffer
            print("Ignoring retained message")
            return
        message["payload"] = msg.payload.decode()  # Decode old messages
        print(f"Received `{message['payload']}` from `{msg.topic}` topic")

    client.reconnect()

    # Unsubscribe/resubscribe to ensure a fresh callback
    client.unsubscribe(topic)
    client.subscribe(topic)
    client.on_message = on_message

    # Wait for new payload in a loop to ensure message is received
    while message["payload"] is None:
        client.loop(timeout=0.1)
        time.sleep(0.1)

    return message["payload"]

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

PIXEL_POINTS = np.float32([
    [403, 372],  # bottom left
    [406, 294],  # middle left
    [412, 218],  # top left
    [468, 375],  # bottom middle
    [469, 296],  # middle
    [474, 219],  # top middle
    [525, 375],  # bottom right
    [528, 299],  # middle right
    [529, 221]   # top right
])

ROBOT_POINTS = np.float32([
    [617.7599487304688, -928.4739990234375],
    [372.47955322265625, -928.4741821289062],
    [130.65554809570312, -922.1380615234375],
    [621.6953125, -743.4808959960938],
    [371.32684326171875, -740.4089965820312],
    [130.94314575195312, -731.9608764648438],
    [623.2314453125, -556.8556518554688],
    [380.7349853515625, -556.8556518554688],
    [132.2861785888672, -561.0795288085938]
])


# Compute homography (least squares solution)
H, _ = cv.findHomography(PIXEL_POINTS, ROBOT_POINTS)

def pixel_to_robot(u: float, v: float, z: float = Z_AXIS_STAGING) -> tuple[float, float, float]:
    pixel = np.array([[[u, v]]], dtype=np.float32)
    xy_mm = cv.perspectiveTransform(pixel, H)   # shape (1,1,2)
    X, Y = xy_mm[0][0]
    return (float(X), float(Y), float(z))

def detect_dice(frame):
    frame = cv.resize(frame, (640, 480))
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    result = frame.copy()
    centers = []

    dice_num = 1
    for c in contours:
        if cv.contourArea(c) < 500:
            continue

        rect = cv.minAreaRect(c)
        center_px = (int(rect[0][0]), int(rect[0][1]))
        w, h = rect[1]
        angle = rect[2]

        if w < h:
            visual_angle = -angle
        else:
            visual_angle = 90 - angle

        centers.append((center_px, visual_angle))

        box = np.intp(cv.boxPoints(rect))
        cv.drawContours(result, [box], -1, (0, 255, 0), 2)
        cv.putText(result, f"{dice_num}", (center_px[0]-15, center_px[1]-10),
                    cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv.putText(result, str(center_px), (center_px[0], center_px[1]),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv.LINE_AA)
        dice_num += 1

    if len(centers) == 0:
        print("No dice detected — check lighting or HSV range.")

    return result, centers

def capture_one_frame():
    DevList = mvsdk.CameraEnumerateDevice()
    if len(DevList) < 1:
        print("No camera found!")
        return None

    DevInfo = DevList[0]
    print("Using camera:", DevInfo.GetFriendlyName())
    hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 60 * 1000)
    mvsdk.CameraPlay(hCamera)

    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    frame = None
    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if monoCamera else 3))
        return frame.copy()
    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)


# 26.56505012 -> -72.20169067
# 71.56505584 -> -25.53129959
#  3.01278734 -> -43.07048034
# 20.55604553 -> -64.03687286

# wrong angles
# VA:  -71.57° → R:   26.57°
# VA:   59.74° → R:  -14.74°
# VA:  -78.69° → R:   33.69°
# VA:  -90.00° → R:   45.00°
# VA:   63.43° → R:  -18.43°
# VA:  -51.34° → R:    6.34°

# 1: -71.57 -> -26.18641281
# 2: 59.74  -> -76.73046112
# 3: -78.69 -> -32.73843002
# 4: -90.00 -> -45.00
# 5: 63.43  -> -68.11922454
# 6: -51.34 -> -95.07603454

def pixel_angle_to_robot_yaw(visual_angle: float) -> float:
    # raw linear mapping
    R = visual_angle + 45.0

    # normalize into robot range [-90, 0]
    if R > 0:
        R -= 180
    if R < -90:
        R += 180

    return R
	# clumps_remaining = False
	#        for idx, info in enumerate(dice_info, start=1):
	#            u, v = info['center']
	#            rect_width, rect_height = info.get('rect_size', (0,0))
	#
	#            if rect_width >= 40 or rect_height >= 40:
	#                clumps_remaining = True
	#                print(f"Clump detected at Dice {idx} ({info['robot_name']}): Pixel=({u},{v})")
	#
	#                x, y, _ = pixel_to_world(u, v, z=Z_HOVER)
	#                if rect_width >= 40:
	#                    x -= 120
	#                    y -= 40
	#                if rect_height >= 40:
	#                    y -= 120
	#                    x -= 40
	#                hover_pose = [np.float32(x), np.float32(y), 135, 179, 0, R_HORIZONTAL]
	#
	#                crx10.write_cartesian_position(hover_pose)
	#                crx10.schunk_gripper("close")
	#                time.sleep(0.5)
	#
	#                bulldoze = hover_pose.copy()
	#                if rect_width > 40:
	#                    bulldoze[0] += 225
	#                if rect_height > 40:
	#                    bulldoze[1] += 225
	#                crx10.write_cartesian_position(bulldoze)
	#                crx10.schunk_gripper("open")
	#                crx10.write_cartesian_position(INTER_HOME)
	#
	#        if clumps_remaining:
	#            print("Waiting 2 seconds before rechecking for remaining clumps...")
	#            time.sleep(2)

# PUSHING PROCESS
def push_dice(robot, dice_positions):
    robot.onRobot_gripper(**CLOSE_GRIPPER_PARAMS)
    time.sleep(0.5)
    for i, (center, angle) in enumerate(dice_positions):
        px, py = center
        rx, ry, _ = pixel_to_robot(px, py)
        robot_angle = pixel_angle_to_robot_yaw(angle)

        above_bulldoze_pos = [rx + 100, ry + 100, Z_AXIS_STAGING, W, P, robot_angle]
        robot.write_cartesian_position(above_bulldoze_pos)

        bulldoze_pos = [rx + 100, ry + 100, Z_AXIS, W, P, robot_angle]
        robot.write_cartesian_position(bulldoze_pos)

        die_pos = [rx, ry, Z_AXIS, W, P, robot_angle]
        robot.write_cartesian_position(die_pos)

        above_die_pos = [rx, ry, Z_AXIS_STAGING, W, P, robot_angle]
        robot.write_cartesian_position(die_pos)
        time.sleep(0.25)

        robot.write_joint_pose(HOME_POS)
        robot.onRobot_gripper(**OPEN_GRIPPER_PARAMS)
        time.sleep(0.5)


# PICKUP PROCESS
def pick_dice(robot, dice_positions):
    for i, (center, angle) in enumerate(dice_positions):
        px, py = center
        rx, ry, _ = pixel_to_robot(px, py)
        robot_angle = pixel_angle_to_robot_yaw(angle)
        # print(f"Pixel {i+1:2d} ({px:3.0f},{py:3.0f}), angle: ({angle}) → Robot ({rx:7.8f}, {ry:7.8f}) mm")

        robot.write_cartesian_position([rx, ry, Z_AXIS_STAGING, W, P, robot_angle])
        time.sleep(0.25)

        robot.write_cartesian_position([rx, ry, Z_AXIS, W, P, robot_angle])
        robot.onRobot_gripper(**CLOSE_GRIPPER_PARAMS)
        time.sleep(0.5)

        robot.write_cartesian_position([rx, ry, Z_AXIS_STAGING, W, P, robot_angle])
        time.sleep(0.25)

        robot.write_joint_pose(HOME_POS)
        robot.onRobot_gripper(**OPEN_GRIPPER_PARAMS)
        time.sleep(0.5)


def main():
    client_pub = connect_mqtt(CLIENT_ID_PUB)  # Client publisher
    client_sub = connect_mqtt(CLIENT_ID_SUB)  # Client subscriber

    rb = robot(ROBOT_IP)

    # Set robot speed to 300
    rb.set_speed(300)

    # Open gripper
    rb.onRobot_gripper(**OPEN_GRIPPER_PARAMS)
    time.sleep(0.5)

    while True:
        # wait for instructions
        msg = wait_for_message(client_sub, TOPIC_A)
        status = json.loads(msg)

        print(f"\nReceived status: {status}")

        if status["status"] == "pushing":
            push_dice(rb, status["positions"])

            # publish completion
            push_status = {"status": "done"}
            client_pub.loop_start()
            publish(client_pub, json.dumps(push_status))
            client_pub.loop_stop()
        else:
            pick_dice(rb, status["positions"])
            break

    # if status["status"] == "pushing":
    #     print("we are pushing")
    #     push_dice(rb, status["positions"])
    #
    #     push_status = {"status": "done"}
    #     msg = json.dumps(push_status)
    #     client_pub.loop_start()
    #     publish(client_pub, msg)
    #     client_pub.loop_stop()
    # else:
    #     print("we are picking")
    #     pick_dice(rb, status["positions"])
    #
    # msg = wait_for_message(client_sub, TOPIC_A)
    # # print(f"\nmsg: {msg}")
    # status = json.loads(msg)


    # frame = capture_one_frame()
    # if frame is None:
    #     print("Failed to capture frame.")
    #     return

    # result, centers = detect_dice(frame)
    # print(f"centers: {centers}")
    # cv.imshow("Dice Detection", result)
    # cv.waitKey(0)

    # pixel_angles = []
    # dice_robot_coords = []
    # for center in centers:
    #     x, y, z = pixel_to_robot(center[0][0], center[0][1])
    #     pixel_angles.append(center[1])
    #     print(f"Pixel ({center[0][0]}, {center[0][1]}, angle {center[1]}) → Robot ({x:.4f}, {y:.4f}, {z:.4f})")
    #     dice_robot_coords.append((x, y, z))

    # print(f"dice_robot_coords: {dice_robot_coords}")

    # if len(centers) == 0:
        # return

    # cv.destroyAllWindows()

    # robot_angles = []

    # test_angles = [26.56505012, 71.56505584, 3.01278734, 20.55604553]
    # for va in pixel_angles:
    #     R = pixel_angle_to_robot_yaw(va)
    #     print(f"VA: {va:7.2f}° → R: {R:7.2f}°")

    # for pixel_angle in pixel_angles:
    #     robot_angle = pixel_angle_to_robot_yaw(pixel_angle)
    #     robot_angles.append(robot_angle)

    # for i, ang in enumerate(robot):

    # rb.write_joint_pose(HOME_POS)


if __name__ == "__main__":
    main()

