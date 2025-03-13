from websocket import create_connection
import json
import threading
import time
import base64

class MCX():
    def __init__(self):
        """
        Сlass is a simplified mechanism for controlling a robot arm.
        """
        self.__ws_get = create_connection("ws://localhost:8766")
        self.__ws_post = create_connection("ws://localhost:8765")
        self.__data = {}
        self.__image1 = ""
        self.__image2 = ""
        self.__motor = []
        self.__load = []
        self.__status = []
        self.__temperature = []
        self.__warning = []
        self.__count = []
        self.__getData()

    def __getData(self):
        def callback():
            while True:
                try:
                    self.__data = self.__ws_get.recv()
                    json_data = json.loads(self.__data)
                    if "Robot" in json_data['name']:
                        self.__motor = [json_data['m1'], json_data['m2'], json_data['m3'], json_data['m4'], json_data['m5'], json_data['m6']]
                        self.__load = [json_data['l1'], json_data['l2'], json_data['l3'], json_data['l4'], json_data['l5'], json_data['l6']]
                        self.__temperature = [json_data['t1'], json_data['t2'], json_data['t3'], json_data['t4'], json_data['t5'], json_data['t6']]
                        self.__status = [json_data['s']]
                        self.__warning = [json_data['i']]
                        self.__count = [json_data['n']]

                    if "SmartCamera" in json_data['name']:
                        if int(json_data['name'].split("_")[1]) == 1:
                            self.__image1 = base64.b64decode(json_data['image_data'])
                        if int(json_data['name'].split("_")[1]) == 2:
                            self.__image2 = json_data['image_data']

                    self.__connection = True

                    time.sleep(0.025)
                except:
                    self.__connection = False
        
        threading.Thread(target=callback, daemon=True).start()

    def move(self, name, x, y, z, t, v):
        """
        Move to the robot ARM
            Args:
                name(str): name of the robot ARM is based on the team number (Robot"team id"_"number robot")
                x(float): x coordinate, mm
                y(float): y coordinate, mm
                z(float): z coordinate, mm
                t(float): gripper rotate, grad
                v(float): gripper state, 0 or 1

            Description:
                If there is a vacuum system, the suction cup is disactivated -> 0.
                If there is a vacuum system, the suction cup is activated -> 1.

                In the presence of a gripping device, the grip is unclenches -> 0.
                In the presence of a gripping device, the grip is compressed -> 1.

            Returns:
                bool: True if operation is completed, False if failed
            
        """
        msg = {
            'robot_name': name,
            'N': '0',
            'X': x,
            'Y': y,
            'T': t,
            'G': z,
            'V': v,
            'L0': '0',
            'L1': '0',
            'L2': '0',
            'L3': '0',
            'L4': '0',
            'P': '0',
            'Text': '' 
        }

        try:
            self.__ws_post.send(json.dumps(msg))
            return True
        except:
            return False
        
    def getManipulatorMotor(self):
        """
        Getting data about the robot ARM motors
            Returns:
                list(str): x, y, z, rx, ry, rz 
            
        """
        return self.__motor 
    
    def getManipulatorLoad(self):
        """
        Getting data about load the robot ARM motors
            Returns:
                list(str): motor1, motor2, motor3, motor4, motor5, motor6 
            
        """
        return self.__load 
    
    def getManipulatorTemperature(self):
        """
        Getting data about temperature the robot ARM motors
            Returns:
                list(str): motor1, motor2, motor3, motor4, motor5, motor6 
            
        """
        return self.__temperature 

    def getManipulatorStatus(self):
        """
        Getting status move the robot ARM
            Returns:
                list(str): 0, 1

            Description: 
                if "0" - robot ARM it stands still
                if "1" - robot ARM is move
        """
        return int(self.__status[0])
    
    def getManipulatorCount(self):
        """
        Getting count message the robot ARM
            Returns:
                list(str): the number of messages received on the robot ARM
        """
        return int(self.__count[0])
    
    def getManipulatorWarning(self):
        """
        Getting warning the robot ARM
            Returns:
                list(str): the number of warning on the robot ARM
        """
        return int(self.__warning[0]) 

    def getManipulatorWarningStr(self):
        """
        Getting warning the robot ARM
            Returns:
                list(str): A verbal description of the warning on the robot arm
        """
        try:
            if self.__warning:
                match (int(self.__warning[0])):
                    case(1):
                        return "RadiusIn error"
                    case(2):
                        return "Area error"
                    case(3):
                        return "Angle error"
                    case(4):
                        return "Z error"
            else:
                return False
        except:
            return self.__warning        
    
    def getCamera1Image(self):
        """
        Getting image data from camera №1
            Returns:
                byte: image bytes 
            
        """
        return self.__image1
    
    def getCamera2Image(self):
        """
        Getting image data from camera №2
            Returns:
                byte: image bytes 
            
        """
        return self.__image2