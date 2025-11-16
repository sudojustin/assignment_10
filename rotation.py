#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import time
import sys
sys.path.append('../FANUC-Ethernet_IP_Drivers/src')
from robot_controller import robot

from robot_controller import robot

# -------------------------------------------------
# HOMOGRAPHY MATRIX
# -------------------------------------------------
HOME_POSITION = [0,0,0,0,-90,30]
INTER_HOME = [445, 340, 550, -179, 0 , 85]
#DICE_PLACE = [600, 155, 115, 179, -1 , 29]
DICE_PLACE = [600, 155, 115, -179.9, 0 , 30]

H = np.array([
    [-3.17671413e-01,  2.56553507e+00, -4.00757545e+02],
    [ 2.17781567e+00, -8.46632744e-02,  3.09571998e+02],
    [-4.03784236e-04, -2.91491300e-04,  1.00000000e+00]
], dtype=np.float64)

# -------------------------------------------------
# ROBOT CONFIG
# -------------------------------------------------
DRIVE_PATH = '10.8.4.16'  # your robot IP
Z_HOVER = 180
R_HORIZONTAL = 30
R_VERTICAL = 120
SPEED = 200  # mm/sec
HOVER_DELAY = 1  # seconds above each dice

# -------------------------------------------------
# UTILS
# -------------------------------------------------
def pixel_to_world(u, v, z0=Z_HOVER):
    uv1 = np.array([u, v, 1.0], dtype=np.float64)
    xyw = np.dot(H, uv1)
    xyw /= xyw[2]
    x, y = xyw[0], xyw[1]
    return x, y, z0

def normalize_R(R):
    while R > 180:
        R -= 360
    while R < -180:
        R += 360
    return R

# -------------------------------------------------
# DICE DETECTION
# -------------------------------------------------
def detect_dice(frame):
    DICE_PLACE = [600, 155, 115, 179, -1 , 29]
    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (15, 60, 60), (45, 255, 255))

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = frame.copy()
    dice_info = []

    dice_num = 1
    for c in contours:
        if cv2.contourArea(c) < 500:
            continue

        rect = cv2.minAreaRect(c)
        center_px = (int(rect[0][0]), int(rect[0][1]))
        width, height = rect[1]
        angle = rect[2]

        # Corrected visual angle calculation
        if width < height:
            visual_angle = -angle
        else:
            visual_angle = 90 - angle

        dice_info.append({'center': center_px, 'visual_angle': visual_angle})

        # Draw rectangle and label
        box = np.intp(cv2.boxPoints(rect))
        cv2.drawContours(result, [box], -1, (0, 255, 0), 2)
        cv2.putText(result, f"{dice_num}", (center_px[0]-15, center_px[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        dice_num += 1

    return result, dice_info

# -------------------------------------------------
# CAMERA CAPTURE
# -------------------------------------------------
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

# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main():
    crx10 = robot(DRIVE_PATH)
    crx10.set_speed(SPEED)

    frame = capture_one_frame()
    if frame is None:
        print("Failed to capture frame.")
        return

    result, dice_info = detect_dice(frame)
    cv2.imshow("Dice Detection", result)
    cv2.waitKey(1)
    dice_place = DICE_PLACE.copy()

    print("\n--- Moving Robot Above Each Dice ---")
    for idx, info in enumerate(dice_info, start=1):
        u, v = info['center']
        visual_angle = info['visual_angle']
        x, y, z = pixel_to_world(u, v, z0=Z_HOVER)
        x_adj = x + 25
        y_adj = y + 5
        # Compute robot R based on dice rotation
        R = R_HORIZONTAL + (visual_angle / 90.0) * (R_VERTICAL - R_HORIZONTAL)
        R = normalize_R(R)

        hover_pose = [np.float64(x_adj), np.float64(y_adj), Z_HOVER, 179, 0.5, R]
        print(f"Dice {idx}: Pixel=({u},{v}) → Robot Coords=({x_adj:.1f}, {y_adj:.1f}, {Z_HOVER}), R={R:.1f}")

        # Move robot above dice
        crx10.write_cartesian_position(hover_pose)
        time.sleep(HOVER_DELAY)
        
        pick_pose = hover_pose.copy()
        pick_pose[2] -= 50   # decrease Z
        print(f"Moving down to Z={pick_pose[2]:.1f}")
        crx10.write_cartesian_position(pick_pose)
        time.sleep(HOVER_DELAY)
        
        crx10.schunk_gripper("close")

        pick1_pose = hover_pose.copy()
        print(f"Moving down to Z={pick_pose[2]:.1f}")
        crx10.write_cartesian_position(pick1_pose)
        time.sleep(HOVER_DELAY)
        
        crx10.write_cartesian_position(INTER_HOME)
        crx10.write_cartesian_position(dice_place)
        print(f"Moving down to {DICE_PLACE}")


        dice_place2 = dice_place.copy()
        dice_place2[2] -= 40   # decrease Z
        print(f"Moving down to {dice_place2}")
        crx10.write_cartesian_position(dice_place2)

        #crx10.write_joint_pose(HOME_POSE);
        crx10.schunk_gripper("open")
        crx10.write_cartesian_position(dice_place)
        crx10.write_cartesian_position(INTER_HOME)
        #crx10.write_cartesian_position(dice_place)
        dice_place[2] += 80

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

