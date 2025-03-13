"""
UDPClient - класс для получения видеопотока от UDP-сервера.

Этот класс реализует клиентскую часть для получения видеокадров, отправленных сервером по протоколу UDP.
Кадры кодируются в формате JPEG, преобразуются в base64 и передаются по сети. Клиент декодирует полученные
данные и возвращает кадры для дальнейшей обработки или отображения.

Пример использования:

    client = UDPClient(host_ip="127.0.0.1", port=9999)
    while True:
        frame = client.receive_frame()
        cv2.imshow("Received Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
"""

import cv2
import socket
import numpy as np
import base64



class UDPClient:
    """
    Класс для работы с UDP-клиентом, получающим видеопоток от сервера.
    """
    


    def __init__(self, host_ip = "0.0.0.0", port = 9999, buffer_size = 65536):
        """
        Инициализация UDP-клиента.

        Параметры:
            host_ip (str): IP-адрес сервера - тот, кто передаёт изображения (по умолчанию "0.0.0.0").
            port (int): Порт сервера (по умолчанию 9999).
            buffer_size (int): Размер буфера для приема данных (по умолчанию 65536 байт).
        """
        
        self.host_ip = host_ip             # IP-адрес сервера
        self.port = port                    # Порт сервера
        self.buffer_size = buffer_size         # Размер буфера для приема данных

        
        # Создание UDP-сокета
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Установка размера буфера для приема данных
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        
        # Отправка начального сообщения для подключения к серверу
        self.client_socket.sendto(b"connect", (self.host_ip, self.port))
        print(f"Connected to server at {self.host_ip}:{self.port}")


    def receive_frame(self):
        """
        Получение и декодирование видеокадра от сервера.

        Возвращает:
            frame (numpy.ndarray): Декодированный кадр в формате numpy массива.
        """
        
        print("get")
        packet, _ = self.client_socket.recvfrom(self.buffer_size)     # Получение пакета данных от сервера

        data = base64.b64decode(packet)   # Декодирование данных из base64
        npdata = np.frombuffer(data, dtype=np.uint8)     # Преобразование данных в массив numpy
        frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)      # Декодирование массива в изображение 

        return frame  # Возврат декодированного кадра