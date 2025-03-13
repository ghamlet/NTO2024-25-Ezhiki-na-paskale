from mcx import MCX
import cv2
import numpy as np

if __name__ == "__main__":
    robot = MCX()

    while True:
        image_byte = robot.getCamera1Image() #get image from tc3
        if image_byte:
            image_np = np.frombuffer(image_byte, np.uint8)
            image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
            cv2.imshow("Display window", image_np)
            cv2.waitKey(1)
