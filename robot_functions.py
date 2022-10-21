
import numpy as np
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET
import os

def robot_orientation_correction(robot_pose):
#Calibration_Values
    current_dir = os.getcwd()
    atracsys_script = os.path.abspath(os.path.join(current_dir,os.pardir))
    atracsys = os.path.abspath(os.path.join(atracsys_script,os.pardir))
    navigation = os.path.abspath(os.path.join(atracsys,os.pardir))
    
    
    xml_path = navigation + "\RobotAPI\Elfin_Robot\ElfinSSRTesting\configs\configs.xml"
    
    config_xml = ET.parse(xml_path)
    root = config_xml.getroot()
    

    #print(el_data)
    orientation_value = root[0][10].text
    
    orientation_tcp = np.fromstring(orientation_value,dtype=float , count= -1, sep=',')
    
    #Converting Orientation euler values to Rotation matrix
    orientation_obj = R.from_euler('xyz',orientation_tcp,degrees=True)
    orientation_matrix = orientation_obj.as_matrix()
    
    #Converting robot getl euler values to Rotation matrix
    robot_euler = robot_pose[-3:]
    robo_euler_obj = R.from_euler('xyz',robot_euler,degrees = True)
    robot_rotmat = robo_euler_obj.as_matrix()
    
    #Applying the orientation to robot getl
    final_rotmat = robot_rotmat @ np.linalg.inv(orientation_matrix)
    
    #Euler angles with orientation
    final_rt_crt = R.from_matrix(final_rotmat).as_euler('xyz',degrees = True)
    
    robot_pose_new = robot_pose[:3]
    for i in range(len(final_rt_crt)):
        robot_pose_new.append(final_rt_crt[i])
        
    return robot_pose_new