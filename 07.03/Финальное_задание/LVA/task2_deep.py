from mcx import MCX
import time
import cv2
import numpy as np

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
    robot.move("Robot4_1", x, y, 220, 0, 0)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1)

    robot.move("Robot4_1", x, y, z, 0, 0)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1)

    # Захват пробы
    robot.move("Robot4_1", x, y, z, 0, 1)  # Активировать захват
    time.sleep(1)

    robot.move("Robot4_1", x, y, 200, 0, 1)  # Активировать захват
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

def capture_images(robot):
    """
    Функция для получения 8 изображений пробы под разными углами.
    """
    angles = [0, 45, 90, 135, 180, -135, -90, -45]  # Углы для поворота (в пределах [-180°, 180°])
    images = []

    for angle in angles:
        # Поворот кисти манипулятора на заданный угол
        robot.move("Robot4_1", 670, -511, 311.5, angle, 1)
        while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
            time.sleep(0.1)

        # Получение изображения с камеры
        image_byte = robot.getCamera1Image()
        if image_byte:
            image_np = np.frombuffer(image_byte, np.uint8)
            image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            images.append(image_np)

            # Отображение изображения в отдельном окне
            window_name = f"Camera Image {angle}°"
            cv2.imshow(window_name, image_np)
            cv2.waitKey(500)  # Кратковременное отображение изображения
        else:
            print(f"Ошибка получения изображения под углом {angle}°")
            return False

    # Ожидание нажатия клавиши для закрытия окон
    print("Нажмите любую клавишу, чтобы закрыть окна с изображениями.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return True

def return_sample(robot, sample_number):
    """
    Функция для возврата пробы на место.
    """
    # Координаты для возврата пробы
    sample_positions = {
        {
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
    }

    if sample_number not in sample_positions:
        print("Некорректный номер пробы")
        return False

    x, y, z = sample_positions[sample_number]

    
    # Перемещение к месту возврата
    robot.move("Robot4_1", x, y, z, 0, 1)
    while robot.getManipulatorStatus() == 1:  # Ожидание завершения движения
        time.sleep(0.1) 
    
    robot.move("Robot4_1", x, y, 200, 0, 1)  # Деактивировать захват
    time.sleep(0.5)
    # Отпускание пробы
    robot.move("Robot4_1", x, y, z, 0, 0)  # Деактивировать захват
    time.sleep(1)

    robot.move("Robot4_1", 489, -131.1, 424, 0, 0) #стартовая позиция
    
    return True

def main():
    robot = MCX()
    successful_runs = 0

    while successful_runs < 1:  # Достаточно одного успешного выполнения
        # Запрос номера пробы
        sample_number = int(input("Введите номер пробы (1-12): "))
        if sample_number < 1 or sample_number > 12:
            print("Некорректный номер пробы. Введите число от 1 до 12.")
            continue

        # Захват пробы
        if not capture_sample(robot, sample_number):
            print("Ошибка захвата пробы")
            continue

        # Транспортировка к камере
        if not transport_to_camera(robot):
            print("Ошибка транспортировки к камере")
            continue

        # Осмотр пробы с разных ракурсов
        if not capture_images(robot):
            print("Ошибка получения изображений")
            continue

        # Возврат пробы на место
        if not return_sample(robot, sample_number):
            print("Ошибка возврата пробы")
            continue

        # Успешное выполнение
        print(f"Испытание успешно завершено")
        successful_runs += 1

    print("Задание выполнено успешно.")

if __name__ == "__main__":
    main()