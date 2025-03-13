import cv2
import os


class ColorTracker:
    
    def __init__(self):
        self.main_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.minb, self.ming, self.minr = 0, 0, 0
        self.maxb, self.maxg, self.maxr = 255, 255, 255
        self.create_trackbar()


    def create_trackbar(self):
        """Создает трекбары для настройки цветовых порогов"""
        
        cv2.namedWindow("trackbar")
        cv2.createTrackbar('minb', 'trackbar', self.minb, 255, lambda x: None)
        cv2.createTrackbar('ming', 'trackbar', self.ming, 255, lambda x: None)
        cv2.createTrackbar('minr', 'trackbar', self.minr, 255, lambda x: None)
        cv2.createTrackbar('maxb', 'trackbar', self.maxb, 255, lambda x: None)
        cv2.createTrackbar('maxg', 'trackbar', self.maxg, 255, lambda x: None)
        cv2.createTrackbar('maxr', 'trackbar', self.maxr, 255, lambda x: None)


    def get_trackbar_positions(self):
        """Получает текущие значения трекбаров"""
        
        self.minb = cv2.getTrackbarPos('minb', 'trackbar')
        self.ming = cv2.getTrackbarPos('ming', 'trackbar')
        self.minr = cv2.getTrackbarPos('minr', 'trackbar')
        self.maxb = cv2.getTrackbarPos('maxb', 'trackbar')
        self.maxg = cv2.getTrackbarPos('maxg', 'trackbar')
        self.maxr = cv2.getTrackbarPos('maxr', 'trackbar')


    def process_frame(self, frame):
        """Обрабатывает кадр, применяя маску по цвету"""
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.get_trackbar_positions()

        mask = cv2.inRange(hsv, (self.minb, self.ming, self.minr), (self.maxb, self.maxg, self.maxr))
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('result', result)

        k = cv2.waitKey(1)
        if k == ord('q'):
            exit()
            
        elif k == ord('s'):
            self.save_thresholds()


    def save_thresholds(self):
        """Сохраняет значения порогов в текстовый файл"""
        
        with open(os.path.join(self.main_dir, "trackbars_save.txt"), "a") as f:
            title = input("\nEnter the description \nTo cancel, write no: ")
            
            if title.lower() != "no":
                f.write(f"{title}:  {self.minb, self.ming, self.minr}, {self.maxb, self.maxg, self.maxr}\n")
                print("Saved\n")
                


import cv2
import numpy as np
import cv2
import numpy as np

import cv2
import numpy as np
import cv2
import numpy as np

import time


STATE = ""


SIZE = (533, 300)

RECT = np.float32([[0, SIZE[1]],
                   [SIZE[0], SIZE[1]],
                   [SIZE[0], 0],
                   [0, 0]])

TRAP = np.float32([[10, 299],
                   [523, 299],
                   [440, 200],
                   [93, 200]])


def binarize(frame):
    hsv =  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (58, 50, 40), (255, 255, 255))

    return mask



def trans_perspective(binary, trap, rect, size, d=0):
    matrix_trans = cv2.getPerspectiveTransform(trap, rect)
    perspective = cv2.warpPerspective(binary, matrix_trans, size, flags=cv2.INTER_LINEAR)
    if d:
        cv2.imshow('perspective', perspective)
    return perspective


def detect_stop2(perspective, show = False):
    h, w = perspective.shape[:2]
    rows_brigthness = np.sum(perspective, axis=1)
    brightest_row_id = np.argmax(rows_brigthness)
    row_white_pixel_amount = rows_brigthness[brightest_row_id]//255
    
    if show:
        stop_frame = perspective.copy()
        cv2.line(stop_frame, (0, brightest_row_id), (w, brightest_row_id), 255)
        cv2.imshow('stop', stop_frame) 
        print(f"Stop line: {row_white_pixel_amount:3} > {int(w * 0.2)} is {row_white_pixel_amount > w * 0.2}; "\
              f"row: {brightest_row_id:3} >= {h//3} is {brightest_row_id >= h//3}")
    # если в самой яркой строке больше 30 процентов белых пикселей и строка ниже середины изображения
    if row_white_pixel_amount > w * 0.25 and brightest_row_id >= h//3: 
        return True
    return False



if __name__ == "__main__":
    #color_tracker = ColorTracker()
    
    
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("BPA2/output_video_1 (1).mp4")

    STATE = "GO"  # Начальное состояние
    
    
    

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка: не удалось получить кадр.")
            break

        
        frame = cv2.resize(frame, SIZE)
        
        bin = binarize(frame)
        
        wrapped = trans_perspective(bin, TRAP, RECT, SIZE)
        
        
        
        cv2.imshow("frame", wrapped)
    
        if detect_stop2(wrapped):
            print("stop")

        if cv2.waitKey(50) & 0xFF == ord('q'):
            break