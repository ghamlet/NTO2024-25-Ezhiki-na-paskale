import rospy
import cv2
import os
from rospy import Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Response, Flask, render_template
from threading import Thread
from time import sleep

import numpy as np
import socket
from stream_server_class import UDPStreamer

#video_server = UDPStreamer(host_ip="172.16.65.104", port=9999)



weights_path = "yolov4-tiny-obj_best.weights"  
config_path = "yolov4-tiny-obj.cfg"
rospy.init_node("test_photo", disable_signals=True)
img_topic = "/pioneer_max_camera/image_raw"
rospy.Subscriber(img_topic, Image)
net = cv2.dnn.readNet(config_path, weights_path)
yolo_model = cv2.dnn.DetectionModel(net)
yolo_model.setInputParams(size=(256, 256), scale=1/255, swapRB=True)
classes = ["long_crack", "many_hokes", "short_crakes", "two_holes"]

bridge = CvBridge()
img = None


def get_photo():

    global bridge
    
    while True:
    
      img = rospy.wait_for_message(img_topic, Image)
#      print("convert image")
      frame= bridge.imgmsg_to_cv2(img,"bgr8")
      class_ids, scores, boxes = yolo_model.detect(frame, 0.4, 0.1)
        # print(scores)
        # print(class_ids)

      if boxes is not None and len(boxes) > 0:
        for cls, score,  box in zip(class_ids, scores, boxes):
            (x, y, w, h) = box
            print(x,y,w,h,classes[cls])



            cv2.rectangle(frame, (int(x), int(y)), (int(x +w), int(y+h)), (55, 0, 255), 2)
     # video_server.send_frame(frame)


def callback(data):
    global bridge
    global img
    
    try:
        frame=bridge.imgmsg_to_cv2(data,"bgr8")
        frame=cv2.resize(frame,(640,480))
        result, frame_to_send = cv2.imencode('.jpg', frame, encode_param)
    except Exception as e:
        rospy.loginfo(str(e))

get_photo()


