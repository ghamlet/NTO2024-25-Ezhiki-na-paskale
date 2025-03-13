import cv2
import socket
import pickle
import struct

# Адрес и порт СЕРВЕРА, к которому подключаемся
HOST = '172.16.65.131'  # IP малинки 
PORT = 8080

data = b""
payload_size = struct.calcsize(">L")


# Создаем клиентский сокет
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))  # Подключаемся к серверу
print('Connected to server')

while True:
    try:
        # Получаем размер сообщения
        while len(data) < payload_size:
            data += s.recv(4096)
        
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        # Получаем данные кадра
        while len(data) < msg_size:
            data += s.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]

        # Декодируем кадр
        frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        # Обработка кадра (ваша логика)
        cv2.imshow("Client Frame", frame)
        
        if cv2.waitKey(1) == ord("q"):
            cv2.destroyAllWindows()
            break

    except Exception as e:
        print(f"Error: {e}")
        break

s.close()