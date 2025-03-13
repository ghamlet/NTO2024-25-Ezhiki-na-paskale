import rospy
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Пути к весам и конфигурации модели YOLO
weights_path = "yolov4-tiny-obj_best.weights"
config_path = "yolov4-tiny-obj.cfg"

# Инициализация ROS-узла
rospy.init_node("test_photo", disable_signals=True)

# Топик для получения изображений
img_topic = "/pioneer_max_camera/image_raw"

# Загрузка модели YOLO
net = cv2.dnn.readNet(config_path, weights_path)
yolo_model = cv2.dnn.DetectionModel(net)
yolo_model.setInputParams(size=(256, 256), scale=1/255, swapRB=True)

# Классы, которые может обнаружить модель
classes = ["long_crack", "many_hokes", "short_crakes", "two_holes"]

# Инициализация моста для преобразования ROS-изображений в OpenCV
bridge = CvBridge()

# Словарь для хранения уникальных объектов и их последних позиций
unique_objects = {}  # {track_id: {"class": class_name, "box": [x1, y1, x2, y2], "age": age}}
next_track_id = 1  # Счетчик для присвоения уникальных ID

# Порог IOU для сопоставления объектов между кадрами
IOU_THRESHOLD = 0.3

# Максимальное количество кадров, в течение которых объект может быть "потерян"
MAX_AGE = 5

def calculate_iou(box1, box2):
    """
    Вычисляет Intersection over Union (IOU) для двух bounding boxes.
    """
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    # Площадь пересечения
    inter_area = max(0, x2 - x1) * max(0, y2 - y1)

    # Площадь каждого bounding box
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])

    # IOU
    iou = inter_area / float(box1_area + box2_area - inter_area)
    return iou

def get_photo():
    global bridge
    global unique_objects
    global next_track_id

    while not rospy.is_shutdown():
        # Ожидание нового изображения
        img = rospy.wait_for_message(img_topic, Image)
        frame = bridge.imgmsg_to_cv2(img, "bgr8")

        # Обнаружение объектов на изображении
        class_ids, scores, boxes = yolo_model.detect(frame, 0.4, 0.1)

        # Список для хранения текущих объектов
        current_objects = []

        if boxes is not None and len(boxes) > 0:
            for box, cls, score in zip(boxes, class_ids, scores):
                x, y, w, h = box
                current_objects.append({
                    "box": [x, y, x + w, y + h],
                    "class": classes[cls],
                    "score": score
                })

        matched_tracks = set()  # Для хранения сопоставленных track_id
        new_objects = []

        for obj in current_objects:
            best_iou = 0
            best_track_id = None

            # Поиск объекта с максимальным IOU
            for track_id, data in unique_objects.items():
                iou = calculate_iou(obj["box"], data["box"])
                if iou > best_iou and iou >= IOU_THRESHOLD:
                    best_iou = iou
                    best_track_id = track_id

            # Если найден подходящий track_id, обновляем его позицию
            if best_track_id is not None:
                unique_objects[best_track_id]["box"] = obj["box"]
                unique_objects[best_track_id]["age"] = 0  # Сбрасываем счетчик "возраста"
                matched_tracks.add(best_track_id)
            else:
                # Если объект новый, добавляем его в unique_objects
                unique_objects[next_track_id] = {
                    "class": obj["class"],
                    "box": obj["box"],
                    "age": 0
                }
                print(f"Обнаружен новый объект: {obj['class']} (ID: {next_track_id})")
                next_track_id += 1

        # Увеличиваем "возраст" всех объектов и удаляем те, которые не были сопоставлены
        for track_id in list(unique_objects.keys()):
            if track_id not in matched_tracks:
                unique_objects[track_id]["age"] += 1
                if unique_objects[track_id]["age"] > MAX_AGE:
                    del unique_objects[track_id]
                    print(f"Объект {track_id} удален из-за отсутствия совпадений.")

        # Визуализация
        for track_id, data in unique_objects.items():
            x1, y1, x2, y2 = data["box"]
            class_name = data["class"]

            # Рисуем прямоугольник вокруг объекта
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (55, 0, 255), 2)
            cv2.putText(frame, f"{class_name} {track_id}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (55, 0, 255), 2)

        # Отображение кадра (опционально)
        if "DISPLAY" in os.environ:
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)
        else:
            print("Display is not available. Skipping cv2.imshow.")

if __name__ == "__main__":
    try:
        get_photo()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Выводим уникальные объекты по завершению программы
        print("\nИтоговое количество уникальных объектов:")
        for track_id, data in unique_objects.items():
            print(f"{data['class']} (ID: {track_id})")



