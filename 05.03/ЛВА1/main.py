from mcx import MCX
import time
import cv2
import numpy as np

def capture_sample(robot, sample_number):
    """
    Функция для захвата пробы с указанным номером.        """
    # Пример координат для захвата пробы (зависит от расположения проб)
    sample_positions = {
        1: (450, -400, 95),
        2: (550,-400, 95),
        3: (650, -400, 95),
        4: (650,-300, 95),
        5: (550, -300, 95),
        6: (650,-200,95 ),
        7: (650, 260,95),
        8: (650,360,95),
        9: (550, 360,95),
        10: (650,460,95),
        11: (550, 460,95),
        12: (450,460,95),

        # Добавьте координаты для всех 12 проб
    }

    if sample_number not in sample_positions:
        print("Некорректный номер пробы")
        return False

    x, y, z = sample_positions[sample_number]
    
    robot.move("Robot1_1", 489, -131.1, 424, 0, 0)
    
    # Перемещение к пробе
    robot.move("Robot1_1", x, y, z, 0, 0)
    time.sleep(2)  # Ожидание завершения движения

    # Захват пробы
    robot.move("Robot1_1", x, y, z, 0, 1)  # Активировать захват
    time.sleep(1)

    return True

def transport_to_camera(robot):
    """
    Функция для транспортировки пробы к камере.
    """
    # Координаты камеры
    camera_position = (670, -511, 311.5)

    # Перемещение к камере
    robot.move("Robot1_1", *camera_position, 0, 1)
    time.sleep(2)

    # Получение изображения с камеры
    image_byte = robot.getCamera1Image()
    if image_byte:
        image_np = np.frombuffer(image_byte, np.uint8)
        image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
        cv2.imshow("Camera Image", image_np)
        cv2.waitKey(1)
        time.sleep(5)  # Ожидание для просмотра изображения
        cv2.destroyAllWindows()
    else:
        print("Ошибка получения изображения с камеры")
        return False

    return True

def return_sample(robot, sample_number):
    """
    Функция для возврата пробы на место.
    """
    # Пример координат для возврата пробы
    sample_positions = {
         1: (450, -400, 95),
        2: (550,-400, 95),
        3: (650, -400,95),
        4: (650,-300, 95),
        5: (550, -300, 95),
        6: (650,-200, 95),
        7: (650, 260,95),
        8: (650,360,95),
        9: (550, 360,95),
        10: (650,460,95),
        11: (550, 460,95),
        12: (450,460,95),
        # Добавьте координаты для всех 12 проб
    }

    if sample_number not in sample_positions:
        print("Некорректный номер пробы")
        return False

    x, y, z = sample_positions[sample_number]

    # Перемещение к месту возврата
    robot.move("Robot1_1", x, y, z, 0, 1)
    time.sleep(2)

    # Отпускание пробы
    robot.move("Robot1_1", x, y, z, 0, 0)  # Деактивировать захват
    time.sleep(1)

    return True

def main():
    robot = MCX()
    successful_runs = 0

    while successful_runs < 1:
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

        # Возврат пробы на место
        if not return_sample(robot, sample_number):
            print("Ошибка возврата пробы")
            continue

        # Успешное выполнение
        print(f"Испытание {successful_runs + 1} успешно завершено")
        successful_runs += 1

    print("Задание выполнено успешно 2 раза подряд.")

if __name__ == "__main__":
    main()