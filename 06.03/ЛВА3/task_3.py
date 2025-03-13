from mcx import MCX
import time
import cv2
import numpy as np
import os

def capture_sample(robot, sample_number):
    """
    Функция для захвата пробы с указанным номером.
    """
    # Координаты для захвата пробы
    sample_positions = {
        
        1: (450, -400, 105),
        2: (550, -400, 105),
        3: (650, -400, 105),
        4: (650, -300, 105),
        5: (550, -300, 105),
        6: (650, -200, 105),
        7: (650, 260, 105),
        8: (650, 360, 105),
        9: (550, 360, 105),
        10: (650, 460, 105),
        11: (550, 460, 105),
        12: (450, 460, 105),
    
    }

    if sample_number not in sample_positions:
        print("Некорректный номер пробы")
        return False

    x, y, z = sample_positions[sample_number]

    # Перемещение к пробе
    robot.move("Robot4_1", x, y, 200, 0, 0)
    time.sleep(0.5)
    robot.move("Robot4_1", x, y, z, 0, 0)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1)
    
    # Захват пробы
    robot.move("Robot4_1", x, y, z, 0, 1)  # Активировать захват
    time.sleep(1)
    robot.move("Robot4_1", x, y, 220, 0, 1)  # перенести захват вверх
    time.sleep(1)

    return True

def transport_to_camera(robot):
    """
    Функция для транспортировки пробы к камере.
    """
    # Координаты камеры
    camera_position = (670, -511, 311.5)

    # Перемещение к камере
    robot.move("Robot4_1", *camera_position, 0, 1)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1)

    return True

def record_video(robot, sample_number, output_folder):
    """
    Функция для записи видео с поворотом пробы на 360°.
    """
    # Создаём папку для сохранения видео, если её нет
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Настройки записи видео
    video_filename = os.path.join(output_folder, f"sample_{sample_number}.avi")
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Кодек для записи видео
    fps = 10  # Частота кадров
    frame_size = (640, 480)  # Размер кадра (зависит от разрешения камеры)

    # Инициализация VideoWriter
    out = cv2.VideoWriter(video_filename, fourcc, fps, frame_size)

    # Углы для поворота (360° с шагом 45°)
    angles = [-135, -90, -45, 0, 45, 90, 135, 180]

    for angle in angles:
        # Поворот кисти манипулятора на заданный угол
        robot.move("Robot4_1", 670, -511, 311.5, angle, 1)
        while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
            time.sleep(1)

        # Получение изображения с камеры
        image_byte = robot.getCamera1Image()
        if image_byte:
            image_np = np.frombuffer(image_byte, np.uint8)
            image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            out.write(image_np)  # Запись кадра в видео
        else:
            print(f"Ошибка получения изображения под углом {angle}°")
            return False

    # Завершение записи видео
    out.release()
    print(f"Видео для пробы {sample_number} сохранено в {video_filename}")
    return True

def return_sample(robot, sample_number):
    """
    Функция для возврата пробы на место.
    """
    # Координаты для возврата пробы
    sample_positions = {
        1: (450, -400, 105),
        2: (550, -400, 105),
        3: (650, -400, 105),
        4: (650, -300, 105),
        5: (550, -300, 105),
        6: (650, -200, 105),
        7: (650, 260, 105),
        8: (650, 360, 105),
        9: (550, 360, 105),
        10: (650, 460, 105),
        11: (550, 460, 105),
        12: (450, 460, 105),
    }

    if sample_number not in sample_positions:
        print("Некорректный номер пробы")
        return False

    x, y, z = sample_positions[sample_number]

    robot.move("Robot4_1", x, y, 220, 0, 1)
    # Перемещение к месту возврата
    robot.move("Robot4_1", x, y, z, 0, 1)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1)

    # Отпускание пробы
    robot.move("Robot4_1", x, y, z, 0, 0)  # Деактивировать захват
    time.sleep(1)

    robot.move("Robot4_1", 489, -131.1, 424, 0, 0) #стартовая позиция
    return True

def main():
    robot = MCX()
    output_folder = "videos"  # Папка для сохранения видео

    # Запрос 4 номеров проб
    sample_numbers = []
    for i in range(4):
        sample_number = int(input(f"Введите номер пробы {i + 1} (1-12): "))
        if sample_number < 1 or sample_number > 12:
            print("Некорректный номер пробы. Введите число от 1 до 12.")
            return
        sample_numbers.append(sample_number)

    # Обработка каждой пробы
    for sample_number in sample_numbers:
        # Захват пробы
        if not capture_sample(robot, sample_number):
            print(f"Ошибка захвата пробы {sample_number}")
            continue

        # Транспортировка к камере
        if not transport_to_camera(robot):
            print(f"Ошибка транспортировки пробы {sample_number} к камере")
            continue

        # Запись видео с поворотом на 360°
        if not record_video(robot, sample_number, output_folder):
            print(f"Ошибка записи видео для пробы {sample_number}")
            continue

        # Возврат пробы на место
        if not return_sample(robot, sample_number):
            print(f"Ошибка возврата пробы {sample_number}")
            continue

        print(f"Проба {sample_number} успешно обработана.")

    print("Задание выполнено успешно. Видео сохранены в папке 'videos'.")

if __name__ == "__main__":
    main()