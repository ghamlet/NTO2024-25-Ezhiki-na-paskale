import cv2
import json
import socket


def start_server(host='172.16.65.131', port=65432):


    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Сервер запущен на {host}:{port}")
        conn, addr = s.accept()
        with conn:
            print('Подключен клиент:', addr)
            data = conn.recv(1024)
            if data:
                drone_data = json.loads(data.decode('utf-8'))
                print("Получены данные:", drone_data)
                
                response = json.dumps(drone_data).encode('utf-8')
                conn.sendall(response)
                print("Данные отправлены клиенту.")



start_server()