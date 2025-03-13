from mcx import MCX
import cv2
import numpy as np
import time

def readWarning(robot):
    if robot.getManipulatorWarning() == 0:
        return True
    else:
        return False
    
def getData(robot):
    if not robot.getManipulatorMotor() == []:
        print(f"\nMotor: {robot.getManipulatorMotor()}")
        print(f"Temperature: {robot.getManipulatorTemperature()}")
        print(f"Load: {robot.getManipulatorLoad()}\n")
        return True
    else:
        print("Restart Server.py")
        return False
    
def motion_programm(robot, step):
    match (step):
        case(0):
            robot.move("Robot1_1", 489, -131.1, 424, 0, 0) #start point
            return True

        case(1):
            robot.move("Robot1_1", 494, 272, 250, 0, 0) #A point
            return True

        case(2):
            robot.move("Robot1_1", 494, 272, 95, 0, 0) #A point
            return True
        
        case(3):
            robot.move("Robot1_1", 494, 272, 95, 0, 1) #A point
            return True

        case(4):
            robot.move("Robot1_1", 494, 272, 250, 0, 1) #A point
            return True

        case(5):
            robot.move("Robot1_1", 470, -400, 310, 0, 1) #camera point
            return True

        case(6):
            robot.move("Robot1_1", 470, -400, 310, 45, 1) #A point
            return True

        case(7):
            time.sleep(5)
            robot.move("Robot1_1", 555, -310, 250, 0, 1) #B point
            return True

        case(8):
            robot.move("Robot1_1", 555, -310, 95, 0, 1) #B point
            return True

        case(9):
            robot.move("Robot1_1", 900, -300, 250, 0, 1) #out point
            return True

        case(10):
            robot.move("Robot1_1", 555, -310, 95, 0, 0) #B point
            return True

        case(11):
            robot.move("Robot1_1", 555, -310, 250, 0, 0) #B point
            return True

        case(12):
            robot.move("Robot1_1", 489, -135.5, 349, 0, 0) #Start point
            return True

        case(13):
            print("Programm done!")
            exit()

def main():
    print("start")
    robot = MCX() 
    time.sleep(1)
    programm_start_bool = True
    current_count = 0
    start_count = robot.getManipulatorCount()

    while programm_start_bool:
        current_count = robot.getManipulatorCount()
        if robot.getManipulatorStatus() == 0:
            step = current_count - start_count
            print(f"Step {step}")
            motion_programm(robot, step)
            time.sleep(0.5) #time for send message


        if robot.getManipulatorStatus() == 1:
            print("Robot move")

        if readWarning(robot):
            print("No error")
            getData(robot)
        else:
            print(robot.getManipulatorWarningStr())
            print(f"In step {step}")
            exit()
                                
        time.sleep(0.01) #Mandatory use to reduce CPU load

if __name__ == "__main__":
    main()