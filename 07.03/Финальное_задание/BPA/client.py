import cv2
import json
import socket

# ==============================================
# 1. НАСТРОЙКА КАРТЫ
# ==============================================
map_image_path = "photo_2025-03-06_02-04-09.png"
image = cv2.imread(map_image_path)
if image is None:
    raise FileNotFoundError(f"Карта {map_image_path} не найдена!")

drone_data = [
        {
            "count": "2",
            "ids": "3",
            "classes": "two_hokes",
            "drone_x": 3.3,  # реальные координаты дрона (метры)
            "drone_y": 4.5,
            "z": 100,
            "image": ""  # Путь к изображению с камеры дрона
        },

        {
            "count": "3",
            "ids": "3",
            "classes": "two_holes",
            "drone_x": 3.3,  # реальные координаты дрона (метры)
            "drone_y": 4.5,
            "z": 100,
            "image": ""  # Путь к изображению с камеры дрона
        }
    ]
map_height, map_width, channels = image.shape
min_x, max_x = 0, 8  # Границы карты в реальных координатах (метры)
min_y, max_y = 0, 5

# Коэффициенты масштабирования
scale_x = map_width / (max_x - min_x)
scale_y = map_height / (max_y - min_y)

# ==============================================
# 2. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ==============================================
def real_to_pixel(x, y):
    """Преобразует реальные координаты (x, y) в пиксели карты."""
    pixel_x = int((x - min_x) * scale_x)
    pixel_y = int((y - min_y) * scale_y)
    return (pixel_x, pixel_y)

# ==============================================
# 3. СЕРВЕР И ОБРАБОТКА ДАННЫХ
# ==============================================
def process_drone_data(drone_data):
    """Обрабатывает данные дрона и отображает их на карте."""
    map_image = cv2.imread(map_image_path)
    if map_image is None:
        raise FileNotFoundError(f"Карта {map_image_path} не найдена!")

    for data in drone_data:
        # Координаты дрона
        drone_pixel_x, drone_pixel_y = real_to_pixel(data["drone_x"], max_y - data["drone_y"])
        
        # Отрисовка зеленой точки (дрон)
        cv2.circle(map_image, (drone_pixel_x, drone_pixel_y), 6, (0, 255, 0), -1)

        # Формирование текста для отображения
        text = f"Count: {data['count']}, IDs: {data['ids']}, Classes: {data['classes']}"
        
        # Позиция текста (смещение от точки)
        text_x, text_y = drone_pixel_x + 10, drone_pixel_y - 10
        
        # Отрисовка текста
        cv2.putText(map_image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Сохранение и вывод результата
    cv2.imwrite("result_map.png", map_image)
    cv2.imshow("Результат", map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def send_data(data, host='172.16.65.131', port=65432):
    """Отправляет данные на сервер."""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(json.dumps(data).encode('utf-8'))
        print("Данные отправлены на сервер.")
        
        response = s.recv(1024) # Чтение данных (максимум 1024 байта)

        if response: 
                response_data = json.loads(response.decode('utf-8')) 
                print("Получен ответ от сервера:", response_data)

                process_drone_data(response_data)

send_data(drone_data)
