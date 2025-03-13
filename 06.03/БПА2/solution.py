import cv2
import numpy as np
import os
import time
from track_bars import ColorTracker

tracker = ColorTracker()

# Определение текущей директории
current_directory = os.path.dirname(os.path.abspath(__file__))

def load_models():
    print("LOAD models")

    # Пути к файлам модели YOLOv4 Tiny
    weights_path = os.path.join(current_directory, "model/yolov4-tiny.weights")
    config_path = os.path.join(current_directory, "model/yolov4-tiny.cfg")
    names_path = os.path.join(current_directory, "model/coco.names")

    # Загрузка модели YOLOv4 Tiny
    net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    yolo_model = cv2.dnn.DetectionModel(net)
    yolo_model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

    # Загрузка классов COCO
    
    # classes = ["long_crack", "many_hokes", "short_crakes", "two_holes"]
    with open(names_path, 'r') as f:
        classes = f.read().strip().split('\n')

    models = [yolo_model, classes]
    return models


def get_road_health(image, models):
    model, classes = models

    # Детекция объектов на изображении
    class_ids, scores, boxes = model.detect(image, confThreshold=0.5, nmsThreshold=0.4)

    stop_sign_detected = False

    if boxes is not None and len(boxes) > 0:
        for cls, score, box in zip(class_ids, scores, boxes):

            # if cls == 11:  # 11 - индекс "stop sign" в COCO
                (x, y, w, h) = box
                stop_sign_detected = True

                # Отрисовка bounding boxes и подписей
                cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), (55, 0, 255), 2)
                cv2.putText(image, f"{classes[cls]} {int(score * 100)}%", (int(x + 5), int(y + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)

    return stop_sign_detected, image


def main():
    # Загрузка модели
    models = load_models()

    # Загрузка видео (0 для веб-камеры или путь к видеофайлу)
    video_path = 0
    cap = cv2.VideoCapture(video_path)

    # Флаг для отслеживания состояния знака
    stop_sign_visible = False
    last_detection_time = None

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        tracker.process_frame(frame)
        # Оценка "здоровья дороги"
        stop_sign_detected, processed_frame = get_road_health(frame, models)

        # Если знак обнаружен
        if stop_sign_detected:
            if not stop_sign_visible:
                print("Stop sign detected!")
                stop_sign_visible = True
            last_detection_time = time.time()  # Обновляем время последнего обнаружения

        # Если знак не обнаружен
        else:
            if stop_sign_visible:
                # Проверяем, прошло ли более 3 секунд с момента последнего обнаружения
                if last_detection_time and (time.time() - last_detection_time) > 3:
                    print("Stop sign disappeared!")
                    stop_sign_visible = False
                    last_detection_time = None

        # Отображение результата
        cv2.imshow("Stop Sign Detection", processed_frame)

        # Выход по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Освобождение ресурсов
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()