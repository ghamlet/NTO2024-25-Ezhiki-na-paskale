import atexit
import time
import random
from pathlib import Path

import cv2
import numpy as np
# import yolopy

from arduino import Arduino
from road_utils import *


DIST_METER = 1825  # ticks to finish 1m
CAR_SPEED = 1600
THRESHOLD = 200
CAMERA_ID = '/dev/video0'
ARDUINO_PORT = '/dev/ttyUSB0'

GO = 'GO'
STOP = 'STOP'
CROSS_STRAIGHT = 'CROSS_STRAIGHT'
CROSS_RIGHT = 'CROSS_RIGHT'
CROSS_LEFT = 'CROSS_LEFT'
_CROSS_LEFT_STRAIGHT = '_CROSS_LEFT_STRAIGHT'
_CROSS_LEFT_LEFT = '_CROSS_LEFT_LEFT'
_CROSS_LEFT_STRAIGHT_AGAIN = '_CROSS_LEFT_STRAIGHT_AGAIN'

PREV_SUBSTATE = None
SUBSTATE = None

PD_UP = 0.4
PD_DOWN = 0.15
PD_H = 0.65
PD_H_INV = 1 - PD_H
X_OFFSET = 0
Y_OFFSET = 0
WIDTH_COEFF = 0

ON_CROSS = CROSS_RIGHT

START_ACTION = False

STATE = GO
PREV_STATE = None

arduino = Arduino(ARDUINO_PORT, baudrate=115_200, timeout=0.1)
#arduino = FakeArduino(debug=False)
time.sleep(2)
print("Arduino port:", arduino.port)


# cap = find_camera(fourcc="MJPG", frame_width=1280, frame_height=720)
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print('[ERROR] Cannot open camera ID:', CAMERA_ID)
    quit()

find_lines = centre_mass2

# wait for stable white balance
for i in range(30):
    ret, frame = cap.read()

#arduino.set_speed(CAR_SPEED)
last_err = 0
ped_log_state_prev = None
last_ped = 0
while True:
    start_time = time.time()
    ret, frame = cap.read()
    end_frame = time.time()
    if not ret:
        break
    
    orig_frame = frame.copy()
    frame = cv2.resize(frame, SIZE)

    bin = binarize(frame, THRESHOLD)

    wrapped = trans_perspective(bin, TRAP, RECT, SIZE)

    bin_line = bin.copy()
    
    left, right = find_lines(wrapped)
    
    # --- GO RIGHT --- #
    if STATE == CROSS_RIGHT:
        if not START_ACTION and not find_lines.left_found:
            START_ACTION = True
        
    if STATE == CROSS_RIGHT and START_ACTION:
        left = int(right - wrapped.shape[1] * 0.6)

        if detect_return_road(wrapped, find_lines.left_side_amount, find_lines.right_side_amount):
            STATE = GO
    # --- GO RIGHT END --- #

    # --- GO STRAIGHT --- #
    if STATE == CROSS_STRAIGHT:
        if not START_ACTION:
            START_ACTION = True
            SUBSTATE = 0
        
    if STATE == CROSS_STRAIGHT and START_ACTION:
        if SUBSTATE == 0:
            bottom_offset_percet = 0.3
            line_amount_percent = 0.15
        else:
            bottom_offset_percet = 0.1
            line_amount_percent = 0.3

        pixel_offset = int(bin.shape[1] * 0.3)
        idx, max_dist = cross_center_path_v4_2(bin, pixel_offset=pixel_offset, bottom_offset_percent=bottom_offset_percet,
                                               line_amount_percent=line_amount_percent, show_all_lines=False)

        left = idx
        right = idx
        cv2.line(bin_line, (idx, 0), (idx, bin_line.shape[0]), 255)

        img_h, img_w = bin.shape[:2]
        h = int(0.9 * img_h)
        w = int(0.7 * img_w)
        cv2.line(bin_line, (w, h), (img_w, h), 200) # hori
        cv2.line(bin_line, (w, 0), (w, img_h), 200) # vert
        crop = bin[h:, w:]
        crop_pixels = crop.shape[0] * crop.shape[1]
        crop_white_pixels = np.sum(crop)//255
        if crop_white_pixels == 0:
            SUBSTATE = 1

        if detect_return_road(wrapped, find_lines.left_side_amount, find_lines.right_side_amount) and not detect_stop2(wrapped):
            STATE = GO
    # --- GO STRAIGHT END --- #

    # --- GO LEFT --- #
    if STATE == CROSS_LEFT:
        STATE = _CROSS_LEFT_STRAIGHT
        meters = 0.4
        arduino.dist(int(DIST_METER*meters))
        print(f'Task: go {meters} meters ({int(DIST_METER*meters)} ticks)')

    if STATE == _CROSS_LEFT_STRAIGHT_AGAIN:
        pixel_offset = int(bin.shape[1] * 0.1)
        idx, max_dist = cross_center_path_v4_2(bin, pixel_offset=pixel_offset, line_amount_percent=0.3,
                                               bottom_offset_percent=0.1)
        idx = max(0, idx)
        left = idx
        right = idx
        cv2.line(bin_line, (idx, 0), (idx, bin_line.shape[0]), 255)

        if detect_return_road(wrapped, find_lines.left_side_amount, find_lines.right_side_amount) and not detect_stop2(wrapped):
            STATE = GO

    if STATE == _CROSS_LEFT_LEFT:
        # left = right = 0
        arduino.check()
        if arduino.waiting():
            arduino_status = arduino.read_data()
            if 'end' in arduino_status:
                STATE = _CROSS_LEFT_STRAIGHT_AGAIN
                # arduino.dist(int(DIST_METER*0.7))

    if STATE == _CROSS_LEFT_STRAIGHT:
        pixel_offset = int(bin.shape[1] * 0.3)
        idx, max_dist = cross_center_path_v4_2(bin, pixel_offset=pixel_offset)
        left = idx
        right = idx
        cv2.line(bin_line, (idx, 0), (idx, bin_line.shape[0]), 255)

        check_start = time.time()
        arduino.check()
        if arduino.waiting():
            arduino_status = arduino.read_data()
            if 'end' in arduino_status:
                STATE = _CROSS_LEFT_LEFT
                meters = 0.7
                arduino.dist(int(DIST_METER*meters))
                print(f'Task: go {meters} meters ({int(DIST_METER*meters)} ticks)')
    # --- GO LEFT END --- #

    err = 0-((left + right) // 2 - wrapped.shape[1] // 2)
    angle = int(90 + KP * err + KD * (err - last_err)) # EXPERIMENT 90 -> 85
    last_err = err
    
    if STATE == _CROSS_LEFT_LEFT:
        angle = 120

    # angle += 5
    angle = min(max(50, angle), 120)
    
    if STATE == GO and detect_stop2(wrapped):
        START_ACTION = False
        #STATE = ON_CROSS
        #STATE = random.choice([CROSS_RIGHT, CROSS_STRAIGHT, CROSS_LEFT])
        STATE = CROSS_LEFT
    
    if PREV_STATE != STATE or PREV_SUBSTATE != SUBSTATE:
        print(f'STATE: {STATE} ({SUBSTATE})')
        PREV_STATE = STATE
        PREV_SUBSTATE = SUBSTATE

    if STATE != STOP:
        arduino.set_speed(CAR_SPEED)
        arduino.set_angle(angle)
    else:
        arduino.stop()

    end_time = time.time()

    fps = 1/(end_time-start_time)
    if fps < 10:
        print(f'[WARNING] FPS is too low! ({fps:.1f} fps)')

