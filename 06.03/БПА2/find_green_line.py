import cv2
import numpy as np
import time




def find_green_line_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([54, 74, 51])
    upper_green = np.array([98, 176, 166])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest_contour)

        if contour_area > min_contour_area:
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy), mask
    return None, mask

min_contour_area = 2000




cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
line_missing_time = None
message_printed = False  # Флаг для отслеживания статуса сообщения
STATE = ""

while True:
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: не удалось захватить кадр.")
        break

    cropped_frame = frame[210:, 130:360]
    
    # Поиск центра зеленой линии в обрезанной области
    center, mask = find_green_line_center(cropped_frame)

    if center:
        # Координаты центра относительно обрезанной области
        cx_cropped, cy_cropped = center
        
        # Пересчет координат на исходный кадр
        cx = cx_cropped + 130
        cy = cy_cropped + 210
        
        # Отрисовка центра и текста
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(frame, f"Center: ({cx}, {cy})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        line_missing_time = None
        message_printed = False  # Сбрасываем флаг при обнаружении линии
        
    else:
        if line_missing_time is None:
            line_missing_time = time.time()
            
            
        elif not message_printed and (time.time() - line_missing_time) > 2:
            print("Линия оказалась между осями машины.")
            message_printed = True  # Помечаем сообщение как показанное
            
            STATE = "STOP"

    print(STATE)