from udp_client_class import UDPClient

import cv2

client = UDPClient(host_ip="172.16.65.131", port=9999)



while True:
    try:
        frame = client.receive_frame()
        
        cv2.imshow("Receiving Frame", frame)


        
        # Выход по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    except:
        pass