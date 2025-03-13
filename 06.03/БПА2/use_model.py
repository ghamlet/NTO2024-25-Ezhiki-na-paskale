import cv2
import numpy as np


# Загрузка модели YOLO
net = cv2.dnn.readNet("/home/avt_user/EYECAR/BPA2/model/yolov4-tiny.weights", "/home/avt_user/EYECAR/BPA2/model/yolov4-tiny.cfg")

# Загрузка классов объектов (например, из файла coco.names)
with open("/home/avt_user/EYECAR/BPA2/model/coco.names", "r") as f:
    classes = f.read().strip().split("\n")

# Получение имен выходных слоев
layer_names = net.getLayerNames()

# Получаем индексы выходных слоев
output_layers_indices = net.getUnconnectedOutLayers()

# Получаем имена выходных слоев
output_layers = [layer_names[i - 1] for i in output_layers_indices]

print("Output layers:", output_layers)

# Открытие видеофайла или видеопотока
video_path = "/dev/video0"  # Укажите путь к вашему видео
cap = cv2.VideoCapture(video_path, cv2.CAP_V4L2)

while True:
    # Чтение кадра
    ret, frame = cap.read()
    if not ret:
        break  # Если видео закончилось, выходим из цикла

    # Получение размеров кадра
    height, width, channels = frame.shape

    # Подготовка входного изображения для YOLO
    blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(416, 416), swapRB=True, crop=False)
    net.setInput(blob)

    # Выполнение предсказания
    detections = net.forward(output_layers)

    # Обработка результатов
    for output in detections:
        for detection in output:
            scores = detection[5:]  # Вероятности для каждого класса
            class_id = np.argmax(scores)  # ID класса с наибольшей вероятностью
            confidence = scores[class_id]  # Уверенность в обнаружении

            if confidence > 0.5:  # Фильтрация по порогу уверенности
                # Координаты bounding box
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Координаты верхнего левого угла
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                # Отрисовка bounding box
                color = (0, 255, 0)  # Зеленый цвет
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # Подпись с именем класса и уверенностью
                label = f"{classes[class_id]} {confidence:.2f}"
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                print(label)
#     # Отображение кадра с обнаруженными объектами
#     cv2.imshow("Video", frame)

#     # Выход по нажатию клавиши 'q'
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Освобождение ресурсов
# cap.release()
# cv2.destroyAllWindows()