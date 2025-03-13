import cv2
import yolopy

CAMERA_ID = '/dev/video0'


# путь до кватнтованной модели
model_file = '/home/avt_user/EYECAR/BPA2/yolov4-tiny-obj_best_uint8.tmfile'
# загружаем модель. 
# use_uint8 должно быть True, если ваша модель квантована. 
# use_timvx должно быть True, если вы запускаете модель на NPU.
# cls_num должно быть равно количеству классов, на которых вы обучали сеть
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True, cls_num=4)
model.set_anchors([18, 33, 33, 48, 25, 71, 58, 76, 40, 113, 87, 140])  # задаем 


# открываем видеокамеру
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


while True:
    # получаем кадр
    ret, frame = cap.read()
    if not ret:
        break
    
    # прогон изображения через нейронную сеть
    # classes хранит список из id классов всех сдетектированных объектов
    # scores уверенность в точности распознавания объектов
    # boxes - список рамок для всех объектов 
    classes, scores, boxes = model.detect(frame)

    # печатаем id классов объектов
    print(classes)

