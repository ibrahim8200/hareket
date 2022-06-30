import cv2 
import numpy as np 
import sys
import time
from numpy.core.numeric import tensordot

from pymavlink import mavutil

vid = cv2.VideoCapture(0) 
za = 0.3
x = 0
x1 = 230
x2 = 320
x3 = 410
x4 = 640
y = 0
y1 = 160
y2 = 240
y3 = 330
y4 = 480


master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

master.wait_heartbeat()


mode = 'STABILIZE'

if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)


mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
while True:

    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break


master.arducopter_arm()
master.motors_armed_wait()

frame_width = int(vid.get(3))
frame_height = int(vid.get(4))

size = (frame_width, frame_height)

result = cv2.VideoWriter('filename.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

while 1:  
    time.sleep(0.2)
    ret, frame = vid.read()                            
    frame = cv2.flip(frame , 1)                            
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 
    l_h = 120
    l_s = 0
    l_v = 0
    u_h = 179
    u_s = 255
    u_v = 255

    l_h_2 = 108 
    l_s_2 = 30
    l_v_2 = 49
    u_h_2 = 179
    u_s_2 = 255
    u_v_2 = 255

    l_h_3 = 134
    l_s_3 =  0
    l_v_3 =  0
    u_h_3 =  179
    u_s_3 =  253
    u_v_3 =  255

    l_h_4 = 130
    l_s_4 = 35
    l_v_4 = 112
    u_h_4 = 179
    u_s_4 = 255
    u_v_4 = 255

    lower1 = np.array([l_h, l_s, l_v])
    upper1 = np.array([u_h, u_s, u_v])
 

    lower2 = np.array([l_h_2, l_s_2, l_v_2])
    upper2 = np.array([u_h_2, u_s_2, u_v_2])


    lower3 = np.array([l_h_3, l_s_3, l_v_3])
    upper3 = np.array([u_h_3, u_s_3, u_v_3])

    lower4 = np.array([l_h_4, l_s_4, l_v_4])
    upper4 = np.array([u_h_4, u_s_4, u_v_4])
    

    lower_mask1 = cv2.inRange(hsv, lower1, upper1) 
    upper_mask2 = cv2.inRange(hsv, lower2, upper2) 
    lower_mask3 = cv2.inRange(hsv, lower3, upper3) 
    upper_mask4 = cv2.inRange(hsv, lower4, upper4)
    mask_Fainal = lower_mask1 + upper_mask2 + lower_mask3 + upper_mask4

    lower_mask1 = cv2.bitwise_and(frame, frame, mask=lower_mask1)
    upper_mask2 = cv2.bitwise_and(frame, frame, mask=upper_mask2)
    lower_mask3 = cv2.bitwise_and(frame, frame, mask=lower_mask3)
    upper_mask4 = cv2.bitwise_and(frame, frame, mask=upper_mask4)
 

    contours, _ = cv2.findContours(mask_Fainal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    detections = []


    for cnt in contours:           
        area = cv2.contourArea(cnt)
        
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True) 
        x = approx.ravel()[0] 
        y = approx.ravel()[1] 

        if area > 400:
            x1, y1, w, h = cv2.boundingRect(cnt)
            detections.append([x1, y1, w, h]) 

            if 6 <= len(approx) <= 9:


                M = cv2.moments(cnt) 
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                cv2.circle(frame, (cx, cy), j, (0, 255, 0), 2)


        if x < cx < x1 and y < cy < y1:  # ust sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if x1 < cx < x2 and y < cy < y1:  # ust orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                -100,
                350,
                0,
                0)
            time.sleep(za)

        if x2 < cx < x3 and y < cy < y1:  # ust orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                100,
                350,
                0,
                0)
            time.sleep(za)
        if x3 < cx < x4 and y < cy < y1:  # ust sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                350,
                0,
                0)
            time.sleep(za)
        if x < cx < x1 and y1 < cy < y2:  # orat sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                350,
                0,
                0)
            time.sleep(za)

        if x1 < cx < x2 and y1 < cy < y2:  # orat orta
            master.mav.manual_control_send(
                master.target_system,
                350,
                -100,
                350,
                0,
                0)
            time.sleep(za)

        if x2 < cx < x3 and y1 < cy< y2:
            master.mav.manual_control_send(
                master.target_system,
                400,
                100,
                350,
                0,
                0)
            time.sleep(za)

        if x3 < cx < x4 and y1 < cy < y2:  # orat sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                350,
                0,
                0)
            time.sleep(za)


        if x < cx < x1 and y2 < cy < y3:  # orat sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                350,
                0,
                0)
            time.sleep(za)

        if x1 < cx < x2 and y2 < cy < y3:  # orat orta
            master.mav.manual_control_send(
                master.target_system,
                300,
                100,
                350,
                0,
                0)
            time.sleep(za)

        if x2 < cx < x3 and y2 < cy< y3:
            master.mav.manual_control_send(
                master.target_system,
                400,
                0,
                350,
                0,
                0)
            time.sleep(za)

        if x3 < cx < x4 and y2 < cy < y3:  # orat sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                350,
                0,
                0)
            time.sleep(za)

        if x < cx < x1 and y3 < cy < y4:  # asga sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                500,
                0,
                0)
            time.sleep(za)
        if x1 < cx < x2 and y3 < cy < y4:  # asga orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                100,
                500,
                0,
                0)
            time.sleep(za)

        if x2 < cx < x3 and y3 < cy < y4:  # asga sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                100,
                500,
                0,
                0)
            time.sleep(za)
        if x3 < cx < x4 and y3 < cy < y4:  # asga sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                500,
                0,
                0)
            time.sleep(za)

        if cx and cy is None:
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                250,
                300,
                0)
            time.sleep(za)

    result.write(frame)

    cv2.imshow("Frame",frame)              
    if cv2.waitKey(1) == 27:
        break


def test():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)



vid.release()
cv2.destroyAllWindows()
result.release()
