import cv2

# ==============================================
# 1. НАСТРОЙКА КАРТЫ
# ==============================================
# Параметры карты (задайте свои значения)
map_image_path = "BPA4/photo_2025-03-06_02-04-09.png" 
image = cv2.imread(map_image_path)        # Путь к изображению карты
map_height, map_width, channels = image.shape  # Размер карты в пикселях
min_x, max_x = 0, 8              # Границы карты в реальных координатах (метры)
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

def detect_damage(image_path):
    """Обнаруживает повреждение на изображении и возвращает его координаты в кадре."""
    frame = cv2.imread(image_path)
    if frame is None:
        return None

    # Предобработка изображения
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Поиск контуров
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Берем самый большой контур
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # X-координата центра повреждения
            cy = int(M["m01"] / M["m00"])  # Y-координата центра повреждения
            return (cx, cy)
    return None

# ==============================================
# 3. ДАННЫЕ ДРОНА (ЗАМЕНИТЕ НА СВОИ)
# ==============================================
# Пример данных: изображения и координаты дрона
drone_data = [
    {
        "image": "frame_001.jpg",
        "drone_x": 1.3,  # реальные координаты дрона (метры)
        "drone_y": 4.5,
        "z": 100
    },
    # {
    #     "image": "frame_002.jpg",
    #     "drone_x": 60.2,
    #     "drone_y": 30.8,
    #     "z": 15
    # }
]

# ==============================================
# 4. ОБРАБОТКА ДАННЫХ И ВИЗУАЛИЗАЦИЯ
# ==============================================
# Загрузка карты
map_image = cv2.imread(map_image_path)
if map_image is None:
    raise FileNotFoundError(f"Карта {map_image_path} не найдена!")

# Обработка каждого кадра
damage_points = []
for data in drone_data:
    # Координаты дрона
    drone_pixel_x, drone_pixel_y = real_to_pixel(data["drone_x"], max_y-data["drone_y"])
    cv2.circle(map_image, (drone_pixel_x, drone_pixel_y), 6, (0, 255, 0), -1)  # Зеленая точка - дрон

    # Детектирование повреждения
    damage_center = detect_damage(data["image"])
    if damage_center is None:
        continue

    # Расчет смещения повреждения (пример для кадра 1920x1080)
    frame_center_x, frame_center_y = 960, 540  # Центр кадра дрона
    pixel_scale = 0.01  # 1 пиксель = 0.01 метра (зависит от высоты и камеры)
    
    dx = (damage_center[0] - frame_center_x) * pixel_scale
    dy = (damage_center[1] - frame_center_y) * pixel_scale

    # Реальные координаты повреждения
    damage_x = data["drone_x"] + dx
    damage_y = data["drone_y"] + dy

    # Преобразование в пиксели карты
    damage_pixel_x, damage_pixel_y = real_to_pixel(damage_x, damage_y)
    damage_points.append((damage_pixel_x, damage_pixel_y))

# Отрисовка повреждений
for (x, y) in damage_points:
    if 0 <= x < map_width and 0 <= y < map_height:
        cv2.circle(map_image, (x, y), 8, (0, 0, 255), -1)  # Красная точка - повреждение

# Сохранение и вывод результата
cv2.imwrite("result_map.png", map_image)
cv2.imshow("Результат", map_image)
cv2.waitKey(0)
cv2.destroyAllWindows()