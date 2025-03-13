import cv2

# Создаем объект VideoCapture
# 0 — это индекс камеры (обычно это встроенная веб-камера)
cap = cv2.VideoCapture(0)
SIZE = (800, 600)

# Проверяем, удалось ли открыть камеру
if not cap.isOpened():
    print("Ошибка: не удалось открыть камеру.")
    exit()


# Бесконечный цикл для захвата кадров
while True:
    # Захватываем кадр
    ret, frame = cap.read()

    # Если кадр не удалось захватить, выходим из цикла
    if not ret:
        print("Ошибка: не удалось захватить кадр.")
        break

    frame = cv2.resize(frame, SIZE)

        
    # Отображаем кадр в окне
    cv2.imshow('Video', frame)

    # Выход из цикла по нажатию клавиши 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

