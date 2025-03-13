
import cv2
import socket
import pickle
import struct

# Server configuration
HOST = '172.16.65.131'  # 192.168.4.1
PORT = 8080

# Create server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
print(f"Server listening on {HOST}:{PORT}")


# Accept client connection
client_socket, addr = server_socket.accept()
print(f"Connected to client: {addr}")

# Initialize camera
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)  # 0 for default camera
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

try:
    while True:
        # Read frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        
        # Compress frame to JPEG (adjust quality as needed)
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 30])
        if not ret:
            continue
        
        # Serialize frame data
        data = pickle.dumps(buffer)
        
        # Pack message size (4 bytes, big-endian)
        message_size = struct.pack(">L", len(data))
        
        # Send message size followed by frame data
        client_socket.sendall(message_size)
        client_socket.sendall(data)
        

except Exception as e:
    print(f"Error: {e}")
    
finally:
    cap.release()
    cv2.destroyAllWindows()
    client_socket.close()
    server_socket.close()
    print("Connection closed and resources released.")
