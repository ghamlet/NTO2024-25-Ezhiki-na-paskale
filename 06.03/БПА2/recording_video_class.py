import cv2
import time
import os

# Инициализация камеры
cap = cv2.VideoCapture(0)


if not cap.isOpened():
    raise RuntimeError("Не удалось подключиться к камере")

# Определение параметров видео
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_size = (frame_width, frame_height)

# Измерение FPS
print("Измерение FPS...")
start_test = time.time()
test_frames = 0
while time.time() - start_test < 5:
    ret, _ = cap.read()
    if ret:
        test_frames += 1
avg_fps = test_frames / 5
print(f"Средний FPS: {avg_fps:.1f}")



# Поиск рабочего кодека
codec = None
codecs = ['mp4v', 'avc1', 'X264', 'XVID', 'MJPG']
for c in codecs:
    try:
        fourcc = cv2.VideoWriter_fourcc(*c)
        writer = cv2.VideoWriter('test.mp4', fourcc, avg_fps, frame_size)
        if writer.isOpened():
            writer.release()
            os.remove('test.mp4')
            codec = c
            break
    except: pass


if not codec:
    cap.release()
    raise RuntimeError("Не найден подходящий кодек")

# Генерация уникального имени файла
output_file = "output_video.mp4"
counter = 1
while os.path.exists(output_file):
    output_file = f"output_video_{counter}.mp4"
    counter += 1

# Инициализация VideoWriter
fourcc = cv2.VideoWriter_fourcc(*codec)
out = cv2.VideoWriter(output_file, fourcc, avg_fps, frame_size)

# Переменные для отслеживания состояния
frame_count = 0
start_time = time.time()
show_preview = True
preview_text_color = (0, 255, 0)

# Основной цикл записи

print(f"Начата запись в файл: {output_file}")
print("Нажмите Q для остановки записи...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Запись кадра
    out.write(frame)
    frame_count += 1

    # Расчет и отображение FPS
    current_fps = frame_count / (time.time() - start_time)
    
    if show_preview:
        cv2.putText(frame, f"FPS: {current_fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, preview_text_color, 2)
        cv2.putText(frame, f"{frame_width}x{frame_height}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, preview_text_color, 2)
        cv2.imshow("Video Recording", frame)

    # Выход по клавише Q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

