
# #-------------------General Modules
import socket
import numpy as np
import os
import math
import time


# #-------------------Math and Robot Modules
from scipy.spatial.transform import Rotation as R
ip = "192.168.11.11"
port = int("10003")


#---------------------------------------------------------------------------------------------------------
#Modules to Make Motion  of the Robot with respect to the Robot



def connect(ip = "192.168.11.11", port = 10003):
    """Connect to the robot

    Args:
        ip ([String]): [192.168.0.10]
        port ([Int]): [10003]

    Returns:
        [Object]: [soccet communication s]
    """
    try:
        s = socket.socket()
        s.connect((ip, int(port)))
        print('robot connected succesfully to')
        print('ip - {}'.format(ip), '\n', 'port - {}'.format(port))
        return s
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")








def movej(s, angles, port=10003, robotID=0):
    """[Move the robot using Joint Values]

    Args:
        s ([Object]): [Robot TCP Connection]
        angles ([List of Floats]): [Joint angles]
        port (int, optional): [Port you got connected to]. Defaults to 10003.
        robotID (int, optional): [The unique identifier of the robot]. Defaults to 0.
    """
    l = angles
    try:
        
        
        data = "MoveJ,0,{},{},{},{},{},{},;".format(
            l[0], l[1], l[2], l[3], l[4], l[5])
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        
        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")


import os

def movel(s, pose, port=10003, robotID=0):
    """Move the robot using cartesain EE pose wrt base

    Args:
        s ([object]): [soccet communication]
        pose ([list of floats]): [x,y,z,rx,ry,rz]
        port (int, optional): [port number of interger type]. Defaults to 10003.
        robotID (int, optional): [The unique identifier for the robot]. Defaults to 0.
    """
    try:
        
        
        x, y, z, rx, ry, rz = tuple(pose)
        data = str("MoveL,{},{},{},{},{},{},{},;\n".format(robotID, x, y, z, rx, ry, rz))
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        # print(cut)
        time.sleep(0.01)
        
        if(cut[1] == "OK"):
          while 1:
              s.send(b"ReadRobotState,0,;")
              time.sleep(0.01)
              result = s.recv(port).decode()
              cut = result.split(',')
              if(cut[1] == "OK" and cut[2] == "0" ):
                # time.sleep(0.1)
                break
        elif(cut[1] == "Fail"):
          print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")







def getj(s,port = 10003, robotID = 0):
    """Get the cartesain pose of the Robot

    Args:
        s ([Object]): [Soccet communication]
        port (int, optional): [Unique Port number]. Defaults to 10003.
        robotID (int, optional): [The unique identifier of the Robot]. Defaults to 0.

    Returns:
        [list of Floats]: [Returns the EE position of the robot if successful]
        [String]: [if the command is unsuccesful]
        
    """
    try:
        
    
        data = str("ReadActPos,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            print(os.getcwd())

            return list(map(float,cut[0 + 2: 6 + 2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 

def getl(s,port = 10003,robotID = 0):
    """Get the joint angle value of the Robot

    Args:
        s([Onject]): [TCP socket connection]
        port (int, optional): [The unique port number]. Defaults to 10003.
        robotID (int, optional): [The unique identifier for the robot]. Defaults to 0.

    Returns:
        [list of Floats]: [Returns the joint angles of the robot if successful]
        [String]: [if the command is unsuccesful]
    """
    try:
        data = str("ReadActPos,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return list(map(float,cut[6 + 2:12 + 2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 
        
        
def startservo(s,servotime, lookaheadtime,port = 10003,robotID = 0):
    try:
        data = str("StartServo,{},{},{},;\n".format(robotID,servotime, lookaheadtime))
        print(data)
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        print(result)
        cut=result.split(',')
        
        if(cut[1]=="OK"):
            print('start servo exexuted')
            return 
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant start the servo") 
        


def pushservo(s,angs,port = 10003,robotID = 0):
    try:
        l = angs
        data = str("PushServoJ,{},{},{},{},{},{},{},;\n".format(robotID,l[0], l[1], l[2], l[3], l[4], l[5]))
        print(data)
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            print('push servo exexuted')
            return 
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant start the servo")     
        
        
        
def home1(s):
    """Home Position one

    Args:
        s (object): Socket
    """
    try:
        movej(s,[0,0,-90,90,-90,0])
    except:
        print("cant go home1")
             
def home2(s):
    """Home Position two

    Args:
        s (object): Socket
    """
    
    try:
        movej(s,[27,0,-90,90,45,0])
    except Exception as e:
        print("cant go home2") 
        
        
