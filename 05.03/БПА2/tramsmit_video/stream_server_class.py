"""
UDPStreamer - класс для отправки видеопотока по UDP.

Этот класс реализует серверную часть для отправки видеокадров клиенту по протоколу UDP.
Кадры сжимаются в формате JPEG, кодируются в base64 и передаются по сети. Сервер ожидает
подключения клиента и отправляет ему видеопоток.

Пример использования:
    streamer = UDPStreamer(host_ip="0.0.0.0", port=9999)
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        streamer.send_frame(frame)
        
"""

import cv2
import socket
import base64


class UDPStreamer:
    """
    Класс для работы с UDP-сервером, отправляющим видеопоток клиенту.

    Атрибуты:
        host_ip (str): IP-адрес сервера (по умолчанию "0.0.0.0").
        port (int): Порт сервера (по умолчанию 9999).
        buffer_size (int): Размер буфера для приема данных (по умолчанию 65536 байт).
        socket_address (tuple): Кортеж (IP, порт) для привязки сокета.
        server_socket (socket.socket): UDP-сокет для обмена данными с клиентом.
        client_addr (tuple): Адрес клиента (IP, порт), подключившегося к серверу.
    """
    

    def __init__(self, host_ip = "0.0.0.0", port = 9999, buffer_size = 65536):
        """
        Инициализация UDP-сервера.

        Параметры:
            host_ip (str): IP-адрес сервера - тот, кто передаёт изображения (по умолчанию "0.0.0.0").
            port (int): Порт сервера (по умолчанию 9999).
            buffer_size (int): Размер буфера для приема данных (по умолчанию 65536 байт).
        """
        
        
        self.host_ip = host_ip
        self.port = port
        self.buffer_size = buffer_size
        
        self.socket_address = (self.host_ip, self.port)  # Адрес сокета (IP, порт)

        # Создание UDP-сокета
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Установка размера буфера для приема данных
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
        # Привязка сокета к указанному адресу
        self.server_socket.bind(self.socket_address)
        print("Listening at: ", self.socket_address)

        # Ожидание подключения клиента
        self.data, self.client_addr = self.server_socket.recvfrom(self.buffer_size)
        print("Got connection from: ", self.client_addr)


    def send_frame(self, frame):
        """
        Отправка видеокадра клиенту.

        Параметры:
            frame (numpy.ndarray): Кадр видео в формате numpy массива.
        """
        
        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 40])    # Сжатие кадра в формат JPEG с качеством 70
        message = base64.b64encode(buffer)  # Кодирование сжатого кадра в base64
        self.server_socket.sendto(message, self.client_addr)        # Отправка закодированного кадра клиенту
