import cv2
from stream_server_class import UDPStreamer

video_server = UDPStreamer(host_ip="172.16.65.131", port=9999)

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)


SIZE  = (533, 300)
while True:
    ret, frame = cap.read()
    
    if not ret:
        break
    
    frame = cv2.resize(frame, SIZE)
    video_server.send_frame(frame)
    print("send frame")
    
    