# Version:A1.0_20220908
from collections import deque
from json import tool
import logging
import atracsys.ftk as tracker
import os
import time
from multiprocessing import Process
import json
#from ElfinRobotMovement import getl , connect
from scipy.spatial.transform import Rotation as R
from os import walk
from flask import Flask
from flask_restful import Resource, Api, reqparse
import pandas as pd
from robot_functions import *
from log_function import setup_logger
from functions import connectionCheck, geometric_median, unit_vector,angle_between,guard_detect,marker_loaded,rotate_vec,ransac,rot2tf,getdivot_x_axis,distance,marker_loaded_one
import urllib3
http = urllib3.PoolManager(maxsize=10)
# from Final_registration import calibrate_tool,collect
import numpy as np
from numpy.linalg import norm
from multiprocessing import Process,Pipe
from flask import request
#logging
from logging import *
# from functions import Get_camera_quats
app = Flask(__name__)
api = Api(app)
# test
import requests
from queue import Queue
from threading import Thread
from filter_func import *
from align_needle_x import align_astura, align_baseplate, align_wedge, align_x
from  sksurgeryspherefitting.algorithms.sphere_fitting import fit_sphere_least_squares

import configparser
np.set_printoptions(suppress=True)

global initialise_status, flag,procs,procs_piv

global connectStatus,initi_delay,guard_ref
global Killflag

connectStatus =0 


init_delay= configparser.ConfigParser()
init_delay.read('Config\\NavigationConfig.ini')
initialisation_delay= init_delay.get("Initialise","delay")
initi_delay = float(initialisation_delay)
global guard_values_stack
guard_values_stack = []
markers = []
geo_status = []
sd = 0
count = 0
status = 0
initialise_status = 0
count_alive_static = 0
count_alive_pivoting = 0
count_alive_recalibration = 0
count_alive_orientation =0
flag = 0
maxPoints = 0
current_guard = []
current_guard_cam = []

procs = []
procs_piv = []
procs_recal=[]
procs_orientation = []
get_cam_json = {"QueryType": "TestData", "IsCameraDataObtained": False, "RegisteredMarkersList": [], "XPointsList":[], "FiducialDataList": []}
register = []
dir=os.getcwd()
log_path= os.path.join(dir,'Log')
#logging
if os.path.exists(log_path)==False:
    
    direc="Log"
    folder=os.path.join(dir,direc)
    fld=os.mkdir(folder)
camera_data_log = setup_logger('CameraData','Log\\CameraData.log')
tool_registration_log = setup_logger('ToolRegistration','Log\\ToolRegistration.log')
guard_log = setup_logger('GuardValues','Log\\GuardValues.log')
guard_values = setup_logger('GuardData','Log\\GuardData.log')
q = Queue()
qout = Queue()
qin = Queue()
qiout= Queue()
def collection():
    global q,qout,register,cam
    markerdata = cam.GetCurrentMarkerData()
    args = q.get()
  
    print(f'camera status {register.camera.status}')
    reg_counter,sd = register.collect(args[0],args[1],args[2],args[3])
    qout.put([reg_counter,sd])
    
def pivoting_collect():
    global q,qout,register,cam
    markerdata = cam.GetCurrentMarkerData()
    args = q.get()
    print(f'camera status {register.camera.status}')
    count,cost,status_piv = register.calibrate_tool(args[0],args[1],args[2],args[3],args[4])
    qout.put([count,cost,status_piv])

def recal_collect():
    global q,qout,cam
    args = q.get()
    reg_count,status_flag,geo_name = marker_recal(args[0],args[1])
    qout.put([reg_count,status_flag,geo_name])
    
def orientation_collect():
    global q,qout
    args = q.get()
    status_orientation = orientation_correction(args[0],args[1],args[2],args[3])
    qout.put([status_orientation])
    

class CameraData:
    RegisteredMarkersList = []
    XPointsList = []
    QueryType = 'TestData'
    Version = '2.0'
    ConnectionStatus = ""
    InitialisationStatus=""
    CameraTilt=-0.0
    IsCameraDataObtained = False
    FiducialDataList = []
    
    
    def __init__(self):
        self.RegisteredMarkersList = []
        self.XPointsList = []
        self.QueryType = 'TestData'
        self.Version = 2.0
        self.IsCameraDataObtained = False  
        self.FiducialDataList = [] 
        self.guardMarker = []
        
class Point:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
class Scale:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0  
class Angle:
    val = 0
    def __init__(self):
        self.val = 0 
class Rotation:
    w = 0
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0
class Transform:
    point = Point()
    point1 = Point()
    point2 = Point()
    point3 = Point()
    point4 = Point()
    rotation = Rotation()
    scale = Scale()
    angle = Angle()

    def __init__(self):
        self.point = Point()
        self.point1 = Point()
        self.point2 = Point()
        self.point3 = Point()
        self.point4 = Point()
        self.rotation = Rotation()
        self.scale = Scale()
        self.angle = Angle()
class RegisteredMarker:
    Top = Transform()
    Bottom = Transform()
    MarkerName = ''
    ErrorStatus = ''
    ErrorValue = 0.0
    MarkerBallsList =[] 
    def __init__(self):
        self.Top = Transform()
        self.Bottom = Transform()
        self.MarkerName = ''
        self.ErrorStatus = ''
        self.ErrorValue = 0.0
        self.MarkerBallsList = []

#CameraData JSON format
def GetJSONFormatData(cameraData):
    cameraJson = {
        "QueryType": cameraData.QueryType,
        "Version": 'A1.0',
        "IsCameraDataObtained": True,
        "ConnectionStatus": cameraData.ConnectionStatus,
        "InitialisationStatus": cameraData.InitialisationStatus,
        "CameraTilt":cameraData.CameraTilt
        
        
        
    }

    cameraJson["RegisteredMarkersList"] = []

    for i in cameraData.RegisteredMarkersList:
        top_transformJson = {
            "point": {
                "x" : i.Top.point.x,
                "y" : i.Top.point.y,
                "z" : i.Top.point.z
            },
            "point1": {
                "x" : i.Top.point1.x,
                "y" : i.Top.point1.y,
                "z" : i.Top.point1.z
            },
            "point2": {
                "x" : i.Top.point2.x,
                "y" : i.Top.point2.y,
                "z" : i.Top.point2.z
            },
            "point3": {
                "x" : i.Top.point3.x,
                "y" : i.Top.point3.y,
                "z" : i.Top.point3.z
            },
            "point4": {
                "x" : i.Top.point4.x,
                "y" : i.Top.point4.y,
                "z" : i.Top.point4.z
            },

            "scale": {
                "x" : i.Top.scale.x,
                "y": i.Top.scale.y,
                "z": i.Top.scale.z,
            },

            "rotation": {
                "x": i.Top.rotation.x,
                "y": i.Top.rotation.y,
                "z": i.Top.rotation.z,
                "w": i.Top.rotation.w
            },
            "Angle":{
                "ang" : i.Top.angle.value
            }
        }
        marker_Json = {
            "MarkerName": i.MarkerName,
            "ErrorStatus": i.ErrorStatus,
            "ErrorValue": i.ErrorValue
        }
        marker_Json["Top"] = top_transformJson
        marker_Json["Bottom"] = top_transformJson
        marker_Json["MarkerBallsList"] = []
        cameraJson["RegisteredMarkersList"].append(marker_Json)

    cameraJson["XPointsList"] = []
    cameraJson["FiducialDataList"] = []
    
    for fiducial_pos in cameraData.FiducialDataList:
        cameraJson["FiducialDataList"].append(fiducial_pos)
    # cameraJson["guardMarker"] = cameraData.guardMarker
    return json.dumps(cameraJson)

#CameraConnection
class CameraConnection:
    def __init__(self):
        self.tracking_system = tracker.TrackingSystem()
        self.frame = tracker.FrameData()
        self.geo_path = ''
        self.geo_status = []
        self.status = 0
        self.errorMessage = []
        self.pivot_geometry = tracker.Geometry()
        self.camData = json.dumps(get_cam_json)
        self.counter = 0
        self.vel_std = 0
        self.vel_mean = 0
        self.allMarkerData = CameraData()


    def ConnectCamera(self):
        self.errorMessage.clear()
        status = 0
        print("Camera connection started")
        camera_data_log.info("Camera Connection Started")
        
       
        if self.tracking_system.initialise() != tracker.Status.Ok:
            self.errorMessage.append("Error, can't initialise the atracsys SDK api.")
            camera_data_log.error("Error, can't initialise atracsys SDK api")
        
        
       
        if self.tracking_system.enumerate_devices() != tracker.Status.Ok:
            self.errorMessage.append("Error, can't enumerate devices.")
            camera_data_log.error("Error, can't enumerate devices.")
       
        if self.tracking_system.set_int_option('Symmetrise coordinates',1) == tracker.Status.Ok:
            print('Coordinates in center')
        else: 
            print('Coordinates in the left camera')

        if self.tracking_system.create_frame(False, 10, 20, 20, 10) != tracker.Status.Ok:
            self.errorMessage.append("Error, can't create frame object.") 
            camera_data_log.error("Error, can't create frame object.")
            self.status = 0
        
        
            
        
        else:

            self.status = 1
            status=self.status            

        # if self.tracking_system.set_int_option('Active Wireless Pairing Enable', 0) != tracker.Status.Ok:
        #     self.errorMessage.append("Error, can't enable wireless marker pairing.")
        #     IsCameraConnected = False  
        print("Camera detected successfully")
        
        camera_data_log.info("Camera detected successfully")

        return status

    def LoadGeometryFiless(self,geo,path):
        self.geo_status.clear()
        self.geo_path = path
        for geometry in geo:    

            if self.tracking_system.set_geometry(path+"\\"+ geometry) != tracker.Status.Ok:
                self.errorMessage.append("Error, can't load geometry file.")
                camera_data_log.error("Error, can't load geometry file.")
                self.geo_status.append([geometry,0])
            else:
                self.geo_status.append([geometry,1])
                camera_data_log.info("Geometry Files Loaded")
        # if self.tracking_system.set_int_option("Strobe mode", 2) != tracker.Status.Ok:
        #     self.errorMessage.append("Error, disable strobe tracking_system")

    def LoadGeometryFiless_in(self,geometry,path,type):
        
        self.geo_path = path
       

        if type == "Load":
            
            if self.tracking_system.unset_geometry(path+"\\"+geometry) != tracker.Status.Ok:
                #print("error can't unset")
                camera_data_log.error("Error, can't unset geometry file.")
                print("Error,Can't Unset Geometry File")
                l_stat=0
                
            

            if self.tracking_system.set_geometry(path+"\\"+geometry) != tracker.Status.Ok:
                self.errorMessage.append("Error, can't load geometry file.")
                print("Can't Set Geometry File")
                camera_data_log.error("Error, can't load geometry file.")
                self.geo_status.append([geometry,0])
                l_stat=0
            else:
                self.geo_status.append([geometry,1])
                camera_data_log.info("Geometry files loaded")
                print("Geometry Files Reloaded")
                l_stat=1
        elif type == "Unload":
            
            if self.tracking_system.unset_geometry(path+"\\"+geometry) != tracker.Status.Ok:
                #print("error can't unset")
                camera_data_log.error("Error, can't unset geometry file.")
                print("Error,Can't Unset Geometry File")
                l_stat=0
            else:
                l_stat =1
                self.geo_status.remove([geometry,1])
                camera_data_log.info("Geometry file unloaded")
                print("Geometry File Unloaded")
            
        # if self.tracking_system.set_int_option("Strobe mode", 2) != tracker.Status.Ok:
        #     self.errorMessage.append("Error, disable strobe tracking_system")
        
        return l_stat
    
    def ReloadGeometry(self,geometry_id):
        load_stat = 1
        pivot = tracker.Geometry()

        print(self.geo_path+"\\"+ 'geometry'+str(geometry_id)+'.ini')
        # self.geo_path+"\\"+ 'geometry'+str(geometry_id)+'.ini'
        # if self.tracking_system.unset_geometry(geometry_id) != tracker.Status.Ok:
        #     print("Error, can't unset geometry file.")
        #     load_stat = 0
        if self.tracking_system.unset_geometry(geometry_id) != tracker.Status.Ok:
            print("Error, can't unset geometry file.")
            camera_data_log.info("Error, can't unset geometry file.")
            load_stat = 0
        if self.tracking_system.set_geometry(self.geo_path+'\\'+'geometry'+str(geometry_id)+'.ini')!=tracker.Status.Ok:
            load_stat = 0
            print("Error, can't set geometry file.")
            camera_data_log.info("Error, can't set geometry file.")
        
        # if tracker.load_geometry(self.geo_path+'\\'+'geometry'+str(geometry_id)+'.ini', pivot) != tracker.Status.Ok:      
        #     self.errorMessage.append("Error, can't reload geometry file.")
        #     print("Error, can't reload geometry file.")
        #     logging.error("Error, can't reload geometry file.")
        #     load_stat = 0
        self.pivot_geometry = pivot


        return load_stat

    def GetCurrentMarkerData(self):
        
        iscameraconnected =connectStatus

        self.allMarkerData = CameraData()
        connectioncheck = 0
        if iscameraconnected == 1:
            t = self.tracking_system.get_last_frame(100, self.frame)
            self.allMarkerData = CameraData()
           
            ac_status,data = self.tracking_system.get_accelerometer_data()
            if ac_status == tracker.Status.Ok:
                ax=data[0][0].x 
                ay=data[0][0].y 
                az=data[0][0].z
                phi = np.arctan(az/ay)
                camera_tilt = np.rad2deg(phi) -15
                # camera_tilt_str = "%.2f" %camera_tilt
                self.allMarkerData.CameraTilt = camera_tilt.round(2)
                #camera_data_log.info("Camera Tilt: %f",camera_tilt)    
            else:
                self.allMarkerData.CameraTilt = -0.00
            
            
            if  t == tracker.Status.Ok:
                if len(self.frame.markers) !=0 or connectioncheck>5:
                    pass
                    
                # print('frame.fiducials',type(frame.fiducials))
                if len(self.frame.fiducials) !=0:
                    for fid in self.frame.fiducials:
                        # print(fid.position)
                        self.allMarkerData.FiducialDataList.append(fid.position)
                
                self.allMarkerData.guardMarker = current_guard
                registeredMarker = RegisteredMarker()
                registeredMarker.MarkerName = 'GuardMarker'
                registeredMarker.MarkerBallsList = []
                registeredMarker.ErrorValue = -1
                registeredMarker.ErrorStatus = 'Enabled'

                translation = Point()
                if len(current_guard) == 0:
                    translation.x = 0
                    translation.y = 0
                    translation.z = 0
                else: 
                    translation.x = current_guard_cam[0]
                    translation.y = current_guard_cam[1]
                    translation.z = current_guard_cam[2]
                angulation = Angle()
                angulation.value = -1
                top = Transform()
                top.point = translation
                top.angle = angulation
                registeredMarker.Top = top
                registeredMarker.Bottom = top              
                self.allMarkerData.RegisteredMarkersList.append(registeredMarker)  
                self.allMarkerData.ConnectionStatus= str(int(connectStatus))
                self.allMarkerData.InitialisationStatus = str(initialise_status)
                
                
                if len(current_guard_cam) != 0:
                    guard_values_stack.append(current_guard_cam)
                while len(guard_values_stack) == 100:
                    median_guard_value = np.median(guard_values_stack,axis=0)
                    guard_values.info(median_guard_value)
                    
                
                for x in self.frame.markers:
                    
                    registeredMarker = RegisteredMarker()
                    registeredMarker.MarkerName = str(x.geometry_id)
                    registeredMarker.MarkerBallsList = []
                    registeredMarker.ErrorValue = x.registration_error
                    registeredMarker.ErrorStatus = 'Enabled'
                

                    translation = Point()
                    translation.x = x.position[0]
                    translation.y = x.position[1]
                    translation.z = x.position[2]
                    
    #               changed the fiducail index from 0,1,2,3 to 4,5,6,7
                    translation1 = Point()
                    translation2 = Point()
                    translation3 = Point()
                    translation4 = Point()
                    
                    translation1.x = x.corresponding_fiducial(0).position[0]
                    translation1.y = x.corresponding_fiducial(0).position[1]
                    translation1.z = x.corresponding_fiducial(0).position[2]
                    

                    translation2.x = x.corresponding_fiducial(1).position[0]
                    translation2.y = x.corresponding_fiducial(1).position[1]
                    translation2.z = x.corresponding_fiducial(1).position[2]

                    translation3.x = x.corresponding_fiducial(2).position[0]
                    translation3.y = x.corresponding_fiducial(2).position[1]
                    translation3.z = x.corresponding_fiducial(2).position[2]

                    translation4.x = x.corresponding_fiducial(3).position[0]
                    translation4.y = x.corresponding_fiducial(3).position[1]
                    translation4.z = x.corresponding_fiducial(3).position[2]
                    m1 = np.array([translation1.x,translation1.y,translation1.z])
                    m2 = np.array([translation2.x,translation2.y,translation2.z])
                    m3 = np.array([translation3.x,translation3.y,translation3.z])
                    v1 = unit_vector(m1-m2)
                    v2 = unit_vector(m3-m2)
                    v_norm = np.cross(v1,v2)
                    angulation = Angle()
                    temp = np.nan_to_num(angle_between(v_norm,np.array([0,0,1])))
                    if temp > 90:
                        temp = 180-temp
                    angulation.value = temp


                    scale = Scale()
                    scale.x = 1
                    scale.y = 2
                    scale.z = 3

                    rotMat = R.from_matrix(x.rotation)
                    qs = rotMat.as_quat()
    #                 print(qs)
                    rotation = Rotation()
                    rotation.x = qs[0]
                    rotation.y = qs[1]
                    rotation.z = qs[2]
                    rotation.w = qs[3]*-1

                    top = Transform()
                    top.rotation = rotation
                    top.scale = scale
                    top.point = translation
                    top.point1 = translation1
                    top.point2 = translation2
                    top.point3 = translation3
                    top.point4 = translation4
                    top.angle = angulation

                    registeredMarker.Top = top
                    registeredMarker.Bottom = top

                    self.allMarkerData.RegisteredMarkersList.append(registeredMarker)  
                connectioncheck = connectioncheck+1
                #camera_data_log.debug(f"Camera Data: {self.camData}")
            else :
                print("Not able to get frame from camera")
                camera_data_log.warning("Not able to get frame from camera")
                print(f'status of connection {self.status}')
        else:
            print("Check Connection")
            camera_data_log.warning("Check Connection")
            
            self.allMarkerData.ConnectionStatus = str(int(connectStatus))
            self.allMarkerData.InitialisationStatus = str(initialise_status)
            
        self.camData = GetJSONFormatData(self.allMarkerData)
        return self.allMarkerData

    def Get_camera_quats(self,geometry):
        
        RegisteredMarkerCount = 0
        data = {}
        # self.GetCurrentMarkerData()
        # print(self.camData)
        try:
            # 
            json_dict = json.loads(self.camData)
            # print(f'printing self.cam data {json_dict}')
            RegisteredMarkerCount =  len(json_dict['RegisteredMarkersList'])
        except:
            print('json error')
            tool_registration_log.error('JSON Error')
            print(f'printing self.cam data {json_dict}')
        

        if  RegisteredMarkerCount != 0: 
            for i in range(RegisteredMarkerCount):
                for Markers in geometry: 
                    if json_dict['RegisteredMarkersList'][i]["MarkerName"] == Markers:
                        Marker0 = {}
                        Marker0 = json_dict['RegisteredMarkersList'][i]
                        rot = Marker0['Top']['rotation']
                        pos = Marker0['Top']['point']
                        pos1 = Marker0['Top']['point1']
                        pos2 = Marker0['Top']['point2']
                        pos3 = Marker0['Top']['point3']
                        pos4 = Marker0['Top']['point4']
                        anglee= Marker0['Top']['Angle']
                        err_fre = Marker0['ErrorValue']
                        position = [pos['x'],pos['y'],pos['z'] ] 
                        position1 =[pos1['x'],pos1['y'],pos1['z'] ]
                        position2 =[pos2['x'],pos2['y'],pos2['z'] ]
                        position3 =[pos3['x'],pos3['y'],pos3['z'] ]
                        position4 =[pos4['x'],pos4['y'],pos4['z'] ]
                        quat = [ rot['x'],rot['y'],rot['z'],rot['w'] ]
                        ag=anglee['ang']
                        #position 1 and position2 are random fidicial data of all the retro balls

                        data[Markers] = (quat,position,position1,position2,position3,position4,ag,err_fre) 
                        

        else:
            print("Marker not visible")
            tool_registration_log.warning("Marker Not Visible")
            
        
        return data

    def data_fetch(self,method,tool_marker,reference_marker,delay,maxPoints):
        if method == 'baseplate' :
            vel_thresh = 0.1
            sample_size = 500
            lenFlag = True
            print('Please enter the aproximate length or place the tool in pivot point')

        elif method == 'static' :
            vel_thresh = 1.5
            sample_size = maxPoints
            lenFlag = True
        elif method == 'baseplateverification':
            vel_thresh = 0.1
            sample_size = 500
            lenFlag = True
            
        else:
            vel_thresh = 1.5
            sample_size = maxPoints
            lenFlag = True

        needle_marker  = tool_marker
        geometry = [needle_marker,reference_marker]
        needle_marker_pos = []
        needle_marker_quat = []
        reference_marker_pos = []
        reference_marker_quat = []
        fiducial1 = np.zeros(3)
        fiducial2 = np.zeros(3)
        fiducial3 = np.zeros(3)
        fiducial4 = np.zeros(3)

        # from get_json import Get_camera_quats
        #######################################
        global reg_counter 
        reg_counter = 0
        end_time = time.time() + 40
        ref_pos_list = []
        ref_quat_list = []
        needle_pos_list = []
        needle_quat_list = []

        fiducial1 = []
        fiducial2 = []
        fiducial3 = []
        fiducial4 = []

        tool = []
        vel_list = []
        
        tool_pos = []
        ref_pos = []
        tool_quat = []
        ref_quat = []
        fid1 = []
        fid2 = []
        fid3 = []
        fid4 = []

        length = 0
        err = 2
        time.sleep(delay)
        start_time = time.time()
        while reg_counter < sample_size: 
                       
            if time.time() < end_time  :   
                self.GetCurrentMarkerData()
                camera_data = self.Get_camera_quats(geometry)
                # print(camera_data)

                if reference_marker in camera_data :
                    reference_marker_quat =  camera_data[reference_marker][0]
                    reference_marker_pos = camera_data[reference_marker][1]
                    
                    # print(' pos ',reference_marker_pos,"  quat ",reference_marker_quat)

                if needle_marker in camera_data :
                    needle_marker_quat =  camera_data[needle_marker][0]
                    needle_marker_pos = camera_data[needle_marker][1]


                if (needle_marker in camera_data) and (reference_marker in camera_data) :
                    disp = norm(np.array([reference_marker_pos]) - np.array([needle_marker_pos]))
                    # print(disp)
                    tool.append(needle_marker_pos)
                    if len(tool) == 2:   
                        velocity = norm(np.diff(np.array(tool),axis = 0),axis = 1)
                        tool.reverse()
                        tool.pop()
                        vel_list.append(velocity)
                        if ((disp < length+err) or lenFlag) and ((disp > length -err) or lenFlag) and velocity < vel_thresh:
                            needle_pos_list.append(needle_marker_pos)
                            needle_quat_list.append(needle_marker_quat)
                            ref_pos_list.append(reference_marker_pos)
                            ref_quat_list.append(reference_marker_quat)
                            fiducial1.append(camera_data[needle_marker][2])
                            fiducial2.append(camera_data[needle_marker][3])
                            fiducial3.append(camera_data[needle_marker][4])
                            fiducial4.append(camera_data[needle_marker][5])
                            # print('tool_marker',needle_marker_pos)
                    
                
                            reg_counter = reg_counter + 1
                            self.counter = reg_counter
                            print(f'count {reg_counter}')
            else:
                print('Time out')
                tool_registration_log.info("Time Out")
                break
        end_time_v = time.time()
        end_time_vv = end_time_v - start_time
        if reg_counter == 0:
            t_per_samples = 10000
        else:
            t_per_samples = end_time_vv / reg_counter        
        velocity_array = np.array(vel_list)
        self.vel_std = (np.std(velocity_array,axis = 0)+ np.mean(velocity_array,axis = 0)) / t_per_samples
        self.vel_mean = np.mean(velocity_array,axis = 0)
       
        tool_pos = np.array(needle_pos_list)
        ref_pos = np.array(ref_pos_list) 
        tool_quat = np.array(needle_quat_list)
        ref_quat = np.array(ref_quat_list)
        fid1 = np.array(fiducial1)
        fid2 = np.array(fiducial2)
        fid3 = np.array(fiducial3)
        fid4 = np.array(fiducial4)
        st = np.std(norm(tool_pos,axis=1))
         
        return(tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter)


    def __del__(self):
        
        print('Inside destructor')
        print('Object destroyed')
        
        

dirname=os.getcwd()
filename=os.path.join(dirname,'env\GeometryFilePath.txt')
fd=os.open(filename, os.O_RDONLY)
red=os.read(fd,10000)
collect_path= red.decode('utf-8')

#Registration Class
class Registration:
    def __init__(self,cam,path):
        self.counter = 0
        self.reference = "" 
        self.tool = ""
        self.maxPoints = 100
        self.delay = 0
        self.method = 'static' 
        self.path = path 
        self.camera = cam
        self.guard = []
    def guard_detect(self,tool_tip,reference_marker,delay):
        
        global guard_ref
        guard_pos = []
        ref_pos = []
        global dist_arr
        dist_arr = []
        
        grd_ref = np.zeros(3)
        dirname=os.getcwd()
        filename=os.path.join(dirname,'env\port.txt')
        fd=os.open(filename, os.O_RDONLY)
        red=os.read(fd,20)
        port= red.decode('utf-8')
        print(port)
        
        r = http.request('GET','http://'+port+':8081/GetCameraData')
        camera_data_log.info(r)
        json_dict = json.loads(r.data)
        # print(json_dict)
        
        RegisteredMarkerCount =  len(json_dict['RegisteredMarkersList'])
        FiducialDataCount = len(json_dict['FiducialDataList'])
        print(f'fiducial data count : {FiducialDataCount}')
        guard_log.info(f"Fiducial Data Count: {FiducialDataCount}")
        time.sleep(delay)
        if  RegisteredMarkerCount != 0: 
            for i in range(RegisteredMarkerCount):      
                if json_dict['RegisteredMarkersList'][i]["MarkerName"] == reference_marker:
                    Marker1 = {}
                    Marker1 = json_dict['RegisteredMarkersList'][i]
                    ref_position = Marker1['Top']['point'] 
                    ref_pos = [ref_position['x'],ref_position['y'],ref_position['z'] ]
                    # print(f'reference marker pos {ref_pos}')

                if json_dict['RegisteredMarkersList'][i]["MarkerName"] == tool_tip:
                    Marker0 = {}
                    Marker0 = json_dict['RegisteredMarkersList'][i]
                    pos = Marker0['Top']['point'] 
                    position = [pos['x'],pos['y'],pos['z'] ] 
                    # print(type(position))
                    # position 1 and position2 are random fidicial data of all the retro balls
                    for i in range(FiducialDataCount): 
                        
                        fid_array = json_dict['FiducialDataList']
                        global fiducials 
                        self.fid_list = fid_array
                        fiducials = fid_array
                        min_guard_dist =  norm(np.array(fid_array[i]) - np.array(position))
                        dist_arr.append(min_guard_dist)
                        # print(f'stray fiducial list {fiducials}')

            if len(dist_arr) > 0:
                arr = dist_arr
                minElement = np.amin(arr)
                print(f'The closest fiducials is at {minElement} mm distance from the tip: ')
                guard_log.info(f'The closest fiducials is at {minElement} mm distance from the tip: ')
                global result
                result = np.where(arr == np.amin(arr))
                # print('index',result[0][0])
                # print(fiducials[result[0][0]])
                if minElement > 1 and minElement < 14:
                    guard_pos = fid_array[result[0][0]]
                    print(f'guard pos {guard_pos}')
            

        if len(ref_pos)>0 and len(guard_pos)>0:
            ref_guard_dist = norm(np.array(ref_pos) - np.array(guard_pos))
            print(f'reference and guard marker distance {ref_guard_dist}')
            guard_log.info(f'reference and guard marker distance {ref_guard_dist}')
        if len(guard_pos)>1:
            for i in self.camera.allMarkerData.RegisteredMarkersList:
                    if i.MarkerName == reference_marker:
                        pos = np.array([[i.Top.point.x, i.Top.point.y, i.Top.point.z]])
                        rot = np.array([i.Top.rotation.x,i.Top.rotation.y,i.Top.rotation.z,i.Top.rotation.w])
                        r = R.from_quat(rot)
                        cam2ref_r = r.as_matrix()
                        ref2cam_r =np.linalg.inv(cam2ref_r)
                        temp = np.hstack((ref2cam_r,pos.transpose()))
                        ref2cam_tf = np.vstack((temp,[[0,0,0,1]]))
                        cam2ref = np.linalg.inv(ref2cam_tf)
                        
                        #print(f'ref2cam_tf {ref2cam_tf}')

                    
                        #print(f'ref2cam_tf {ref2cam_tf}')
                        
                        grd_ref = cam2ref@np.hstack((guard_pos,[1]))
                        
                        guard_ref = grd_ref[:3]
               
            self.guard = grd_ref[:3]

        return guard_pos,ref_pos

    def collect(self,tool_marker,reference_marker,delay,maxPoints):
        self.reference = reference_marker
        self.tool = tool_marker
        self.maxPoints = maxPoints
        self.delay = delay
        
        
        method = 'static'
        (tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter) =  self.camera.data_fetch(method,tool_marker,reference_marker,delay,maxPoints)   
        if reg_counter <10:
            return 0,0
        my_array = np.hstack((tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4))
        df = pd.DataFrame(my_array, columns = ['toolx','tooly','toolz','tool_qx','tool_qy','tool_qz','tool_qw','refx','refy','refz','ref_qx','ref_qy','ref_qz','ref_qw','fid1x','fid1y','fid1z','fid2x','fid2y','fid2z','fid3x','fid3y','fid3z','fid4x','fid4y','fid4z'])

        # _pivoting is for collect data(check for this file to avoid recollecting static data)
        df.to_csv(self.path+'\Data\\'+tool_marker+'_collect.csv')
        tool_registration_log.info("csv file created in Data")
        print('Done')
        tool_registration_log.info("Static data collection done")
        sd = np.std(norm(tool_pos,axis=1))
        tool_registration_log.info("SD: %f,Collection Type: Static",sd)
        return reg_counter,sd

    def calibrate_tool(self,tool_marker, reference_marker,RegistrationType,delay,Sample):
        self.tool = tool_marker
        
        needle_marker = tool_marker
        maxPoints = Sample
        tool_registration_log.info(f"Input Sample Size:{maxPoints}")
        self.method = RegistrationType
        method = RegistrationType
        dirname=os.getcwd()
        filename=os.path.join(dirname,'env\GeometryFilePath.txt')
        fd=os.open(filename, os.O_RDONLY)
        red=os.read(fd,10000)
        collect_path= red.decode('utf-8')
        Static_data_path = collect_path+'\Data\\' +needle_marker+'_collect.csv'
        staticdata_file=os.path.join((collect_path+'\\Data'), needle_marker+'_collect.csv')
        
        if os.path.exists(staticdata_file) == False:
            cnt,sd_piv = register.collect(tool_marker,reference_marker,0,1000)
        else:
            pass
        Static_data = static_data(Static_data_path)
        
        (tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter) =  self.camera.data_fetch(method,tool_marker,reference_marker,delay,maxPoints)     

        if reg_counter < 10:
            return 0,0


        my_array = np.hstack((tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4))

        df = pd.DataFrame(my_array, columns = ['toolx','tooly','toolz','tool_qx','tool_qy','tool_qz','tool_qw','refx','refy','refz','ref_qx','ref_qy','ref_qz','ref_qw','fid1x','fid1y','fid1z','fid2x','fid2y','fid2z','fid3x','fid3y','fid3z','fid4x','fid4y','fid4z'])
       

        #Input the Pivot data here
        Pivot_data = pivot_data(df,Static_data)

        Filter = ['IMD_Intersection', 'RMS']
        SD =[0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5 , 3]
        df_filtered = pivot_data_filter(Pivot_data,Filter[1],SD[5])
        df_in = df_filtered

        xdata = df_in['toolx'].to_numpy()
        ydata = df_in['tooly'].to_numpy()
        zdata = df_in['toolz'].to_numpy()

        
        
        tool_pos = df_in[['toolx','tooly','toolz']].to_numpy()
        tool_quat = df_in[['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
        ref_pos = df_in[['refx','refy','refz']].to_numpy()

        # The reference pos can be changed to the average marker values
        centre = np.mean(ref_pos,axis=0)

        # fig = plt.figure()
        # ax = plt.axes(projection='3d')
        # ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
        # plt.show(block=False)
        # plt.pause(1) # 3 seconds, I use 1 usually
        # plt.close("all")
        cost = 0
        radii = 0
        offset = np.zeros(3)
        
        if method == 'sphere':
            offset,dataa = self.sphere_fit(df_in)
            cost = dataa.cost
            radii = norm(offset)
            print('The radius is',radii)
            tool_registration_log.info(f'The radius is: {radii}')
            print('The offset is',offset)
            tool_registration_log.info(f'The offset is: {offset}')

        if method == 'baseplate':
            filter = 'on'
            offset =  self.base_plate_registration(df_in,filter)
            
            
        if method == 'baseplateverification':
            filter = 'on'
            offset =  self.base_plate_registration(df_in,filter)
            
            
        
        dir=os.getcwd()
        pivotcsv_path= os.path.join(dir,'PivotCsv')

        if os.path.exists(pivotcsv_path)==False:
    
            direc="PivotCsv"
            folder=os.path.join(dir,direc)
            fld=os.mkdir(folder)  



        df.to_csv('PivotCsv\\'+needle_marker+'_whilePivot.csv')
       
        path = 'geometry' + needle_marker
        print('Final_reg',path)
        config = self.pivot_geometry(offset,path)

        dirname=os.getcwd()
        filename=os.path.join(dirname,'env\GeometryFilePath.txt')
        fd=os.open(filename, os.O_RDONLY)
        red=os.read(fd,10000)
        geo_path= red.decode('utf-8')
        print(method)
        status = 0
        # check out the optimality parameter
        # if the file is over written, check the  contents of the file
        # if method == 'sphere' and radii > 100 and cost < 2 :
        print(f'velocity_std {self.camera.vel_std}')
        # if method == 'sphere' and cost < 4 and self.camera.vel_std > 0.5:
        velocity_thrsh = 50
        vel= self.camera.vel_std
        
        toolConfig = configparser.ConfigParser()
        toolConfig.read("Config\\toolConfig.ini")
        keyss = toolConfig.sections()
        
        for k in keyss:
            if method == 'sphere' and path == 'geometry'+ k:
                cost_thrsh = int(toolConfig.get(k,'cost'))
                if cost < cost_thrsh and vel > velocity_thrsh:
                    cfgfile = open(geo_path+"\\"+path+'9.ini','w')
                    config.write(cfgfile,space_around_delimiters=False)
                    cfgfile.close()
                    rot_fids = align_x(path+'9')
                    print('sphere fit', rot_fids)
                    print(norm(rot_fids))
                    status = 1
                    reload_stat=cam.ReloadGeometry(int(register.tool))
                    if reload_stat==1:
                        print(f"Geometry File Reload Status : {reload_stat}")
                        tool_registration_log.info(f"Geometry File Reload Status : {reload_stat}")
                    else:
                        print(f"Error can't reload Geometry File {reload_stat}")
                        tool_registration_log.info(f"Error can't reload Geometry File {reload_stat}")
                    break
                        
            elif method == 'sphere' and path == 'geometry'+ k and toolConfig.get(k,'name') == 'Astura':
                if cost < int(toolConfig.get(k,'cost')) and vel > velocity_thrsh:
                    cfgfile = open(geo_path+"\\"+path+'9.ini','w')
                    config.write(cfgfile,space_around_delimiters=False)
                    cfgfile.close()
                    rot_fids = align_astura(path+'9')
                    print('sphere fit', rot_fids)
                    print(norm(rot_fids))
                    status = 1
                    reload_stat=cam.ReloadGeometry(int(register.tool))
                    if reload_stat==1:
                        print(f"Geometry File Reload Status : {reload_stat}")
                        tool_registration_log.info(f"Geometry File Reload Status : {reload_stat}")
                    else:
                        print(f"Error can't reload Geometry File {reload_stat}")
                        tool_registration_log.info(f"Error can't reload Geometry File {reload_stat}")
                    break
        key_list=[]
        for kk in keyss:
            key_list.append(kk)
        if path[-7:] not in key_list:    
            if method == 'sphere' and cost < float(toolConfig.get('common','cost')) and vel>velocity_thrsh:
                cfgfile = open(geo_path+"\\"+path+'9.ini','w')
                config.write(cfgfile,space_around_delimiters=False)
                cfgfile.close()
                rot_fids = align_x(path+'9')
                print('sphere fit', rot_fids)
                print(norm(rot_fids))
                status = 1
                reload_stat=cam.ReloadGeometry(int(register.tool))
                if reload_stat==1:
                    print(f"Geometry File Reload Status : {reload_stat}")
                    tool_registration_log.info(f"Geometry File Reload Status : {reload_stat}")
                else:
                    print(f"Error can't reload Geometry File {reload_stat}")
                    tool_registration_log.info(f"Error can't reload Geometry File {reload_stat}")
                
            
        if method == 'baseplate' :
            status=1
            cfgfile = open(geo_path+"\\"+path+'9.ini','w')
            config.write(cfgfile,space_around_delimiters=False)
            cfgfile.close()
            inline_vector= getdivot_x_axis(df)
            rot_fids = align_baseplate(path+'9',inline_vector)
            
            reload_stat=cam.ReloadGeometry(int(register.tool))
            if reload_stat==1:
                print(f"Geometry File Reload Status : {reload_stat}")
                tool_registration_log.info(f"Geometry File Reload Status : {reload_stat}")
            else:
                print(f"Error can't reload Geometry File {reload_stat}")
                tool_registration_log.info(f"Error can't reload Geometry File {reload_stat}")
            
            pass
        if method == 'baseplateverification':
            error_off= norm(offset)
            print(f" Offset: {offset}")
            print(f"Error: {error_off}")
            tool_registration_log.info(f"BasePlate Error:{error_off}")
            if error_off < 2:
                status=1
            else:
                status=0
            cost=error_off
        print(reg_counter,cost,status)
        tool_registration_log.info(f"Count: {reg_counter},Cost; {cost}")
        tool_registration_log.info(f"Status: {status}")
        return reg_counter,cost,status
    
    def getwedge_x_axis(self,toolmarker,wedgemarker,delay,samplesize):
        
        maxPt= samplesize
        method=""
        
        (tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter) =  self.camera.data_fetch(method,toolmarker,wedgemarker,delay,maxPt)
        
        arr= np.hstack((tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4))
        
        df = pd.DataFrame(arr, columns = ['toolx','tooly','toolz','tool_qx','tool_qy','tool_qz','tool_qw','refx','refy','refz','ref_qx','ref_qy','ref_qz','ref_qw','fid1x','fid1y','fid1z','fid2x','fid2y','fid2z','fid3x','fid3y','fid3z','fid4x','fid4y','fid4z'])
        df.to_csv(self.path+'\\Data\\oreintation_data_'+toolmarker+'.csv')
        #df = pd.read_csv(self.path+'\\Data\\oreintation_data_'+referencemarker+'.csv')
        
        
        divot_pos = df[['refx','refy','refz']].to_numpy()
        divot_quat = df [['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
        tool_poss = df[['toolx','tooly','toolz']].to_numpy()
        tool_quatt = df [['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
        tool_fid1 = df [['fid1x','fid1y','fid1z']].to_numpy()
        tool2divot_tf=[]
        divot2tool_tf=[]
        x_axis=[]
        inline_stack = []
        
        #divot2cam
        for i in range(len(divot_pos)):
            divot2cam_r=R.from_quat(divot_quat[i]).as_matrix().transpose()
            tool2cam_r= R.from_quat(tool_quatt[i]).as_matrix().transpose()
            
            divot2cam=rot2tf(divot2cam_r,divot_pos[i])
            cam2divot = np.linalg.inv(divot2cam)
            tool2cam= rot2tf(tool2cam_r,tool_poss[i])
            cam2tool = np.linalg.inv(tool2cam)
            divot2tool = cam2tool@divot2cam
            tool2divot = np.linalg.inv(divot2tool)
            print(cam2tool@np.hstack((tool_fid1[0],[1])))
            
            tool2divot_tf.append(tool2divot)
            divot2tool_tf.append(divot2tool)
            # x_divot = tool2divot@[1,0,0,1]
            # x_tool = divot2tool@[1,0,0,1]
            # print(x_tool,x_divot)
            # x_axis.append(x_divot[i])
            inline_stack.append(divot2tool[:3,0])
            
            
        inline= np.mean(np.array(inline_stack),axis=0)
        tool_registration_log.info(f"Inline Vector: {inline}")   
        return inline
    def sphere_fit(self,df):
        # df = pd.read_csv("PivotCsv\8200098_whilePivot.csv")
        tool_pos = df[['toolx','tooly','toolz']].to_numpy()
        tool_quat = df[['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
        ref_pos = df[['refx','refy','refz']].to_numpy()
        ref_quat = df[['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
        centre = np.mean(tool_pos,axis=0)
        radius = 300
        tool_pos_ref_space = []
        ref2tool_r = []
        ref_pos_m = np.mean(ref_pos,axis=0)
        ref_quat_m = np.mean(ref_quat,axis =0)

        ref2cam_r_m = R.from_quat(ref_quat[0]).as_matrix().transpose()
        ref2cam_tf_m = rot2tf(ref2cam_r_m,ref_pos[0])
         #Pivoting in reference space
        for i in range(len(tool_pos)):
            tool2cam_r = R.from_quat(tool_quat[i]).as_matrix().transpose()
            tool2cam_tf = rot2tf(tool2cam_r,tool_pos[i])
            cam2tool_tf = np.linalg.inv(tool2cam_tf)
            ref2cam_r = R.from_quat(ref_quat[i]).as_matrix().transpose()
            ref2cam_tf = rot2tf(ref2cam_r,ref_pos[i])
            cam2ref_tf = np.linalg.inv(ref2cam_tf)
            tool_pos_hm = np.append(tool_pos[i],1)
            tool_pos_in_ref = cam2ref_tf @ tool_pos_hm
            
            tool_pos_ref_space.append(tool_pos_in_ref[:3])
            ref2tool_tf = ref2cam_tf @ cam2tool_tf
            ref2tool_quat = R.from_matrix(ref2tool_tf[:3,:3]).as_quat()
            ref2tool_r.append(ref2tool_quat)
        
        center_ref = cam2ref_tf @ np.append(centre,1)
        ref2tool_r = np.array(ref2tool_r)
        xref = np.array(tool_pos_ref_space).transpose()[0]
        yref = np.array(tool_pos_ref_space).transpose()[1]
        zref = np.array(tool_pos_ref_space).transpose()[2]
        center_refspace = center_ref[:3]
        
        # xdata = np.array(tool_pos).transpose()[0]
        # ydata = np.array(tool_pos).transpose()[1]
        # zdata = np.array(tool_pos).transpose()[2]
        initial_parameters = np.array([center_refspace[0],center_refspace[1],center_refspace[2],radius])
        data = fit_sphere_least_squares(xref, yref, zref, initial_parameters, bounds=((-np.inf, -np.inf, -np.inf, -np.inf), (np.inf, np.inf, np.inf, np.inf)) )
        print("cost",data.cost)
        tool_registration_log.info('Sphere Fit Cost: %f',data.cost)
        print("total fit info ",data)
        tool_registration_log.info(f'Fit Info {data}')

        points = np.vstack((xref,yref,zref)).transpose()
        pivot_point = data.x[:3]
        tool_registration_log.info(f'Pivot Point: {pivot_point}')

        pivot_vector =  pivot_point - points
        local2tracker = R.from_quat(ref2tool_r) # input quaternions is tracker to local
        r = local2tracker
        p_vec_local = rotate_vec(r.inv(),pivot_vector)
        print("local tip",np.mean(p_vec_local,axis=0))
        tip=np.mean(p_vec_local,axis=0)
        
        tool_registration_log.info(f"Local Tip: {tip}")

        tip_pos = rotate_vec(r,p_vec_local)+tool_pos
        tool_registration_log.info(f"Tip Position: {tip_pos}")
        

        err = tip_pos - ref_pos
        tool_registration_log.info(f"Error: {err}")
        # plot_vec(err)
        # plt.xlabel('tip error')
        # plt.show(block=False)
        # plt.pause(10) # 3 seconds, I use 1 usually
        # plt.close("all")
        # minor_angle, major_angle = major_minor_angle(pivot_vector)
        # print(" min angle ",minor_angle," max angle ",major_angle)

        return np.mean(p_vec_local,axis=0),data

    def base_plate_registration(self,df,filter):
        tool_pos = df[['toolx','tooly','toolz']].to_numpy()
        tool_quat = df[['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
        ref_pos = df[['refx','refy','refz']].to_numpy()

        p_vec = ref_pos - tool_pos
        tracker2local = R.from_quat(tool_quat)
        r = tracker2local.inv()
        p_vec_local = rotate_vec(r.inv(),p_vec)
        pivot_len = norm(p_vec_local,axis = 1)
        inlier_pivot = ransac(len(pivot_len),pivot_len)
    

        if filter == 'on':
            n = np.sum(inlier_pivot)
            r = r[inlier_pivot]
            tool_pos = tool_pos[inlier_pivot]
            ref_pos = ref_pos[inlier_pivot]
            p_vec_local = p_vec_local[inlier_pivot]
            pivot_point = np.mean(p_vec_local,axis=0)

        else:
            pivot_point = np.mean(p_vec_local,axis=0)               
            n = len(pivot_len)

        pivot_point_list = []
        # making copies of the computed local axis vector
        # applying rotaion of the marker to the computed local frame vector and
        # to compute tip_pos

        for i in range(n):
            pivot_point_list.append(pivot_point)

        tip_pos = rotate_vec(r,pivot_point_list)+tool_pos
        err = norm(tip_pos-ref_pos,axis=1)

        print(f'The distance between tip_pos and ref_pos{np.mean(err)}')
        print(f"tip position is at {np.mean(tip_pos,axis = 0)}")
        print(f"The base plate tip is at{np.mean(ref_pos,axis = 0)}")
        print('marker - ref_pos = ' ,np.mean(norm(p_vec_local,axis=1)))
        length = np.mean(norm(p_vec_local,axis=1))
        
        return np.mean(p_vec_local,axis=0)

    # #################################################################
    # filtering

    def filt(self,fid1_pos,fid2_pos):
        vec = fid1_pos-fid2_pos
        dif= np.diff(vec,axis=0)

        dif_mag = norm(dif,axis=1)

        # plt.plot(dif_mag)
        # plt.xlabel('intermarker dist ')
        # plt.show(block=True)
        # plt.pause(3) # 3 seconds, I use 1 usually
        inter_dist = norm(vec, axis = 1)
        mean = np.mean(inter_dist)
        n = len(inter_dist)
        inlier = ransac(n,inter_dist)
        # plt.ylabel('intermarker distance of tool variation')
        # plt.show()
        return inlier


    def pivot_geometry(self,pointer_offset,path):
        config = configparser.ConfigParser()
        configin = configparser.ConfigParser()
        dirname=os.getcwd()
        filename=os.path.join(dirname,'env\GeometryFilePath.txt')
        fd=os.open(filename, os.O_RDONLY)
        red=os.read(fd,10000)
        geo_path= red.decode('utf-8')
        configin.read(geo_path+"\\"+path+'.ini')

        section = []
        nbr_fid = 4
        origin_fid = 3
        for i in range(nbr_fid):
            section.append('fiducial'+str(i))
            config.add_section('fiducial'+str(i))
        config.add_section('geometry')

        # for fiducia 1
        axes = ['x','y','z']

        keys = configin.sections()

        for fid in keys:
            if fid != 'geometry':
                xval = float(configin[fid]['x']) - pointer_offset[0]
                yval = float(configin[fid]['y']) - pointer_offset[1]
                zval = float(configin[fid]['z']) - pointer_offset[2]

                
                config.set(fid,'x',str(xval))
                config.set(fid,'y',str(yval))
                config.set(fid,'z',str(zval))

        config.set('geometry','count',configin['geometry']['count'])
        config.set('geometry','id',configin['geometry']['id']+'9')
    # add 9 to the new geometry
        # cfgfile = open('/home/ssr/fusionTrack_SDK-v4.5.2-linux64/data/'+path+'9.ini','w')
        # config.write(cfgfile,space_around_delimiters=False)
        # cfgfile.close()       
        return config

global cam    
cam = CameraConnection()

def initialisation_thread():
    global qin , qiout
    args = qin.get()
    status,geo_status = checkConnection(args)
    qiout.put([status,geo_status])
    
def enumerate_device():
    var_status = 0
    if cam.tracking_system.enumerate_devices() != tracker.Status.Ok:
        print('Error,Cant enumerate device')
        var_status = 0
    else:
        var_status = 1
    return var_status



#CollectStaticData
class collectStatic(Resource):

    def get(self):
        global initialise_status
        if connectStatus == 1:
            if initialise_status == 1:
                parser = reqparse.RequestParser()
                parser.add_argument('data',required=True,type = str)
                args = parser.parse_args()
                print(args['data'])

                try:
                    if "data" in args:
                        global maxPoints 

                        data = args['data']
                        retVal = json.loads(data)
                        # return{"status":str(retVal)},200
                        tool_marker = retVal["ToolMarker"]
                        reference_marker = retVal["ReferenceMarker"]
                        delay = int(retVal["StartDelay"])
                        maxPoints = int(retVal["MaximumPoint"])
                    

                        global p,q,geo_status,sd,procs,flag
                    
                        
                        p = Thread(target = collection)
                        p.daemon = True

                    
                        procs.append(p)

                        if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
                            tool_registration_log.warning("Collect Static Data: marker not loaded")
                            return {'status':'marker not loaded',
                            },200
                        
                        count_alive_static = 0
                        for pp in procs:
                            count_alive_static = count_alive_static + int(pp.is_alive())    
                        print(f"Number of process static running {count_alive_static}")
                    
                        if count_alive_static == 0: 
                            q.put([tool_marker,reference_marker,delay,maxPoints])
                            procs[-1].start()
                            global last_process 
                            last_process = len(procs)-1
                            flag = 1
                            # print(f'exit code post the process{procs[-1].exitcode}')
                            # procs[-1].join()
                            # print(f'exit code post the process{procs[-1].exitcode}')
                            tool_registration_log.info('Collect Static Data : 1')
                            return {'status':str(1), 'ConnectionStatus':'1'       
                            },200
                        else: 
                            tool_registration_log.info ('Collect Static Data: 0')
                            return {'Status':str(0),'ConnectionStatus':'1'
                            },200

                    else:
                        tool_registration_log.error("Collect Static Data: format error")   
                        return {'Status':"formatError" ,'ConnectionStatus':'1'      
                            },200


                except:
                    tool_registration_log.error("Collect Static Data: Error")
                    return {'Status':"Error",'ConnectionStatus':'1'       
                            },200
            else:
                tool_registration_log.warning("Collect Static Data: Not Initialised")
                return {'Status':"not initialised" ,'ConnectionStatus':'1'      
                            },200
        else:
            
            return{'Status':'0','ConnectionStatus':'0'}       
api.add_resource(collectStatic,'/CollectStaticData') 

#Pivoting
class pivoting(Resource):
    def get(self):
        try:
            if connectStatus == 1 and initialise_status == 1:
                parser = reqparse.RequestParser()
                parser.add_argument('data',required=True,type = str)
                args = parser.parse_args()
                print(args['data'])
                if "data" in args:
                    data = args['data']
                    retVal = json.loads(data)
                    tool_marker = retVal['ToolMarker']
                    reference_marker = retVal['ReferenceMarker']
                    RegistrationType = retVal['RegistrationType']
                    StartDelay = int(retVal['StartDelay'])
                    Sample= int(retVal['MaxPoints'])
                    
                    p_piv = Thread(target = pivoting_collect)
                    p_piv.daemon = True

                    
                    global geo_status,sd,status
                    procs_piv.append(p_piv)
                    print(geo_status,sd,status)
                    if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
                        tool_registration_log.warning("marker not loaded")
                        return {'count':'marker not loaded',
                                'sd':'marker not loaded'
                                },200
                    
                    count_alive_pivoting = 0
                    for p_piv in procs_piv:
                        count_alive_pivoting = count_alive_pivoting + int(p_piv.is_alive())
                    print(f"No process running{count_alive_pivoting}")
                    # ConnectCamera()
                    # LoadGeometryFiles()
                    if count_alive_pivoting == 0: 
                        q.put([tool_marker,reference_marker,RegistrationType,StartDelay,Sample])
                        global last_piv_process,flag
                        last_piv_process =  len(procs_piv)-1
                        procs_piv[-1].start()
                        flag = 2
                        tool_registration_log.info("Pivoting Status: 1")
                        return {'Status':str(1) , 'ConnectionStatus':'1'
                                },200
                    else:
                        tool_registration_log.info("Pivoting Status:0")
                        return {'Status':str(0) ,'ConnectionStatus':'1'
                                },200                   

                else:
                    tool_registration_log.error("Pivoting: Format Error")
                    return {'Status':f'FormatError','ConnectionStatus':'1'
                                },200
            else:
                
                return{'Status': '0', 'ConnectionStatus':'0'}
        except:
            tool_registration_log.error("Pivoting: Error")
            return {'Status':f'Error'
                        },200

api.add_resource(pivoting,'/StartPivoting')  

#GetCollectedPointCount
class getCollectedPointCount(Resource):

    def get(self):
        if connectStatus == 1 and initialise_status == 1:
        # 0 for none,1 for static, 2 for pivoting
            global flag
            
            global procs,procs_piv,procs_recal,procs_orientation
            count_alive_static = 0
            count_alive_pivoting = 0
            count_alive_recalibration =0
            count_alive_orientation =0
            for pp in procs:
                count_alive_static = count_alive_static + int(pp.is_alive())  
            for p_piv in procs_piv:
                count_alive_pivoting = count_alive_pivoting + int(p_piv.is_alive())
            for p_recal in procs_recal:
                count_alive_recalibration = count_alive_recalibration + int(p_recal.is_alive())
            for p_or in procs_orientation:
                count_alive_orientation = count_alive_orientation + int(p_or.is_alive())
            print(f'count alive static {count_alive_static}')
            if count_alive_static == 1:
                flag = 1
                # print(f'Thread name when alive {procs[-1].name}' )
                # print(os.getpid())
                tool_registration_log.info("Collection Type= Static, Status: Collecting")
                return  {"Count":reg_counter,
                "CollectionType":"static",
                        "ErrorMessage":"0",
                        'Status':"Collecting"},200
            
            if count_alive_pivoting == 1:
                flag = 2
                tool_registration_log.info("Collection type= Sphere, Status= Collecting")
                return  {"Count":reg_counter,
                "CollectionType":"pivoting",
                        "ErrorMessage":"0",
                        'Status':"Collecting"},200
            if count_alive_recalibration == 1:
                flag =3
                tool_registration_log.info("Collection Type = Marker Recal , Status = Collecting")
                return {"Count": reg_counter , 
                        "CollectionType":"Recalibration",
                        "ErrorMessage":"0",
                        "Status":"Collecting"},200
            if count_alive_orientation == 1:
                flag =4 
                tool_registration_log.info("Collection Type= Orientation , Status = Collecting")
                return {"Count": reg_counter , 
                        "CollectionType":"Orientation",
                        "ErrorMessage":"0",
                        "Status":"Collecting"},200
            if count_alive_pivoting == 0 and count_alive_static == 0 and count_alive_recalibration == 0:
                if flag == 1:
                    tool_registration_log.info("Collection type= Static, Status= Done")
                    return {"Count":reg_counter,
                        "CollectionType":"static",
                                "ErrorMessage":"0",
                                'Status':"Done"},200
                elif flag == 2:
                    tool_registration_log.info("Collection type= pivoting, Status= Done")
                    return {"Count":reg_counter,
                        "CollectionType":"pivoting",
                                "ErrorMessage":"0",
                                'Status':"Done"},200
                elif flag == 3:
                    tool_registration_log.info("Collection Type = Marker Recal , Status = Done")
                    return{"Count": reg_counter,
                        "CollectionType": "Recalibration",
                        "ErrorMessage":"0",
                        "Status":"Done"}
                elif flag == 4:
                    tool_registration_log.info("Collection Type= Orientation , Status = Done")
                    return{"Count": reg_counter,
                        "CollectionType": "Orientation",
                        "ErrorMessage":"0",
                        "Status":"Done"}
                else:
                    tool_registration_log.info("Null")
                    return {"Count":"0",
                        "CollectionType":"null",
                                "ErrorMessage":"0",
                                'Status':"null"},200
        else:
            return {"Count":"0","CollectionType":"0","ErrorMessage":"0","Status":"0"}

 
api.add_resource(getCollectedPointCount,'/GetCollectedPointCount')
 
#GetCollectedPointStatus
class getCollectedPointStatus(Resource):
    
    
    def get(self):
        if connectStatus == 1 and initialise_status == 1:
            global last_process, flag , procs,procs_piv,qout,procs_recal,procs_orientation
            print(len(procs))
            if len(procs)>0:
                print(f'The type of collection flag {flag} last process{last_process} {len(procs)}  ')
                if procs[last_process].is_alive() == 0   and flag == 1:
                    # print(f'sentinal values {procs[last_process].sentinel}')
                    try:
                        args  = qout.get(timeout=1)
                        if args is None:
                            print('log que empty')
                            tool_registration_log.info("Log que empty")
                    except:
                        tool_registration_log.info("Status: 0, SD: 0 ")
                        return {'Status':'0',
                                'SD':str(0),
                                'Cost':'0',
                                'ErrorMessage':'null'
                            },200
                        
                    

                    if args[0] == maxPoints:
                        tool_registration_log.info("Status: 1, SD: %f",args[1])
                        return {'Status':'1',
                                'SD':str(args[1]),
                                'Cost':'0',
                                'ErrorMessage':'null'
                            },200
                    else:
                        tool_registration_log.info("Status: 0, SD: %f",args[1])
                        return {'Status':'0',
                                'SD':str(args[1]),
                                'Cost':'0',
                                'ErrorMessage':'null'
                            },200

            global last_piv_process
            try:
                if len(procs_piv) > 0 and procs_piv[last_piv_process].is_alive() ==0 and flag == 2:
                    try:
                        args  = qout.get(timeout=1)
                        if args is None:
                            print('log que empty')
                        # if args[2] == 1:
                        #     global cam
                        #     stat = cam.ReloadGeometry(int(register.tool))
                        #     print(f'geometry reload stat{stat}')
                            
                    except:
                        tool_registration_log.info("Null")
                        return {'Status':"null",
                        'SD':"null",
                        'Cost':'0',
                        'ErrorMessage':'null'
                        },200
                        
                    tool_registration_log.info(f"Cost: {args[1]}, SD: {args[0]}")
                    return {
                                'Cost':str(args[1]),
                                'Status':str(args[2]),
                                'SD':str(args[0]),
                                'ErrorMessage':'null'
                        },200
            except:
                tool_registration_log.info("Processing Hit Again")
                return {
                                'Cost':'0',
                                'Status':'0',
                                'SD':'0',
                                'ErrorMessage':'processingHitAgain'
                        },200
            global last_recal_process
            
            if len(procs_recal)>0 and procs_recal[last_recal_process].is_alive() == 0 and flag ==3:
                try:
                        args  = qout.get(timeout=1)
                        if args is None:
                            print('log que empty')
                        # if args[2] == 1:
                        #     global cam
                        #     stat = cam.ReloadGeometry(int(register.tool))
                        #     print(f'geometry reload stat{stat}')
                            
                except:
                    tool_registration_log.info("Null")
                    return {'Status':"null",
                    'SD':"null",
                    'Cost':'0',
                    'ErrorMessage':'null'
                    },200
                
                return{'Status': str(args[1]),
                    'GeometryName': str(args[2])},200
            global last_orient_process
            if len(procs_orientation) > 0 and procs_orientation[last_orient_process].is_alive()==0 and flag == 4:
                try:
                        args  = qout.get(timeout=1)
                        if args is None:
                            print('log que empty')
                        # if args[2] == 1:
                        #     global cam
                        #     stat = cam.ReloadGeometry(int(register.tool))
                        #     print(f'geometry reload stat{stat}')
                            
                except:
                    tool_registration_log.info("Null")
                    return {'Status':"null",
                    'SD':"null",
                    'Cost':'0',
                    'ErrorMessage':'null'
                    },200
                
                return{'Status': str(args[0]),
                    },200
                    
            tool_registration_log.info ("Null")
            return {'Status':"null",
                        'SD':"null",
                        'Cost':'0',
                        'ErrorMessage':'null'
                },200
        else: 
            return {'Status': '0', 'SD':'0','Cost':'0','ErrorMessage':'0'}   

api.add_resource(getCollectedPointStatus,'/GetCollectedPointStatus') 

#Guard Pick
class guard(Resource):
    def get(self):
        try:
            parser = reqparse.RequestParser()
            parser.add_argument('data',required=True,type = str)
            args = parser.parse_args()
            print(args['data'])
            if "data" in args:
                data = args['data']
                retVal = json.loads(data)   
                tool_marker = retVal['ToolMarker']
                reference_marker = retVal['ReferenceMarker']
                delay = int(retVal['StartDelay'])
                global geo_status
                
            
                if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
                    guard_log.warning("GuardPosition: marker not loaded, ReferencePosition: marker not loaded")
                    return {'GuardPosition':'marker not loaded',
                            'ReferencePosition':'marker not loaded'
                            },200
                else:

                    print(f'tool marker{tool_marker},reference marker{reference_marker},delay {delay}')
                    
                    guard_pos,ref_pos = register.guard_detect(tool_marker,reference_marker,delay)
                    
                    guard_log.info(f"GuardPosition: {guard_pos}, ReferencePosition : {ref_pos}")
                    if len(guard_pos) == 0 or len(ref_pos) == 0 :
                        return {'Status': "0",
                            'GuardPosition':[],
                                'ReferencePosition' : [],
                                'ErrorMessage': ""
                        },200
                    else:
                        return {'Status': "1",
                            'GuardPosition':guard_pos,
                                'ReferencePosition' : ref_pos,
                                'ErrorMessage': ""
                        },200

            else:
                guard_log.error("GuardPosition: Format Error, ReferencePosition: Format Error")
                return {'Status': "0",
                    'GuardPosition':[],
                        'ReferencePosition' :[],
                        'ErrorMessage': "Format Error"
                },200
        except:
            guard_log.error("GuardPosition: Error, ReferencePosition:Error")
            return {'Status': "0",
                'GuardPosition':[],
                    'ReferencePosition' : [],
                    'ErrorMessage': "Error"
            },200
api.add_resource(guard,'/Guard')

#Initialisation
def checkConnection(geo_path):
    
    
    
    geo = next(walk(geo_path), (None, None, []))[2]  # [] if no file
    
    cam.ConnectCamera()
    status = cam.status
    if cam.status == 1:
        cam.LoadGeometryFiless(geo,geo_path)
    geo_status = cam.geo_status

    return status,geo_status
class initialise(Resource):
    
    def get(self):
        global geo_status,initialise_status
        if initialise_status == 0:
            status = 0
            parser = reqparse.RequestParser()
            parser.add_argument('data',required=True,type = str)

            args = parser.parse_args()
            try:
                data = args['data']
                retVal = json.loads(data)
                GeometryPath =  retVal["GeometryPath"]
                CameraIPAddress = retVal["CameraIP"]
                CameraPort = retVal["CameraPort"]
                AverageCount = retVal["AverageCount"]
                dirname=os.getcwd()
                filename=os.path.join(dirname,'env\GeometryFilePath.txt')
                with open(filename,'w') as f:
                    f.write(GeometryPath)
                
                dirname=os.getcwd()
                filename=os.path.join(dirname,'env\GeometryFilePath.txt')
                fd=os.open(filename, os.O_RDONLY)
                red=os.read(fd,10000)
                collec_path= red.decode('utf-8')


                status,geo_status = checkConnection(GeometryPath)
                if status == 1:
                    global register
                    register = Registration(cam,collec_path)
                # global initialise_status
                initialise_status = status
                print(f'geo_status {geo_status}')
                # dirname=os.getcwd()
                # filename=os.path.join(dirname,'env\GeometryFilePath.txt')
                # with open(filename,'w') as f:
                #     f.write(GeometryPath)
                camera_data_log.info(f"Geometry Files : {geo_status}, Geometry File Path: {GeometryPath}") 
                return {'CameraConnectionStatus':str(status),
                        'GeometryFiles':geo_status,
                        'GeometryFilePath':GeometryPath,
                        'ErrorMessage':str(cam.errorMessage)
                },200
            except:
                camera_data_log.error("Camera Connection Status: 0, Error in initialising")
                return {'CameraConnectionStatus':str(0),
                        'GeometryFiles':[],
                        'GeometryFilePath':str(0),
                        'ErrorMessage':"Error in initialising "+ str(cam.errorMessage)
                },200          
        else:
            camera_data_log.info("Camera Connection Status:1 ,Camera is already connected")
            
            return {'CameraConnectionStatus':str(1),
                    'GeometryFiles':[],
                    'GeometryFilePath':str(0 ),
                    'ErrorMessage':"Camera is already connected"
            },200              

api.add_resource(initialise,'/Initialization') 

#Reload Geometry
def rld(geometry,type):
    dirname=os.getcwd()
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,10000)
    geo_path= red.decode('utf-8')
    # geo = next(walk(geo_path), (None, None, []))[2]  # [] if no file
    status = cam.status
    if cam.status == 1:
       reload_stat= cam.LoadGeometryFiless_in(path=geo_path, geometry=geometry,type=type)
    geo_status = cam.geo_status
    
    return reload_stat
class reload(Resource):
    def get(self):
        if connectStatus == 1 and initialise_status == 1:
            parser = reqparse.RequestParser()
            parser.add_argument('data',required=True,type = str)

            args = parser.parse_args()
        
            data = args['data']
            retVal = json.loads(data)
            geometry =  retVal["GeometryName"]
            typeof=retVal["Type"]
            stat=rld(geometry,typeof)
            if stat==1:
                camera_data_log.info(f"status:{stat}")
                return {'Status':str(stat),},200
            else:
                camera_data_log.info(f"status:{stat}")
                return {'Status':str(stat),},200
        else:
            return {'Status':'0'}
                
        

api.add_resource(reload,'/ReloadGeometry') 
get_cam_json = {"QueryType": "TestData", "Version": "A1.0", "IsCameraDataObtained": False,"ConnectionStatus":"0", "InitialisationStatus":"0","RegisteredMarkersList": [], "XPointsList":[], "FiducialDataList": []}

#GetCameraData
@app.route('/GetCameraData',methods=["GET"])
def get_cam():
    
    if initialise_status == 1 and connectStatus == 1 :
        
        
        camera_data_log.info(f"Cam Status {cam.status}")
        markerdata = cam.GetCurrentMarkerData()
        data = GetJSONFormatData(markerdata) 
        camera_data_log.info(data)
        return data
    else:
        camera_data_log.info(get_cam_json)
        
        
        
        return get_cam_json,200

#GetVersionNumber
@app.route('/GetVersionNumber',methods=["GET"])
def get_vers():
    version_num = {"Version": "A1.0"}
    
    return version_num

@app.route('/GuardClear',methods =["GET"])
def guard_clear():
    
    global guard_ref
    
    guard_ref = [0,0,0]
    
    if guard_ref[0] == 0:
        return {"Status":"1",},200
    else: 
        return {"Status": "0",},200
#GetGuardDistance
@app.route('/GuardDistance',methods=["GET"])
def guardSearch():
    try:
        if cam.status == 1:
                fiducial=cam.GetCurrentMarkerData()
                parser = reqparse.RequestParser()
                parser.add_argument('data',required=True,type = str)
                args = parser.parse_args()
                print(args['data'])
                if "data" in args:
                    data = args['data']
                    retVal = json.loads(data)   
                    reference_marker = retVal['ReferenceMarker']
                    distance_threshold =int( retVal['DistanceThreshold'])
                def compute_cam2ref(cam,reference_marker):
                    cam2ref = []
                    for i in cam.allMarkerData.RegisteredMarkersList:
                        if i.MarkerName == reference_marker:
                            pos = np.array([[i.Top.point.x, i.Top.point.y, i.Top.point.z]])
                            rot = np.array([i.Top.rotation.x,i.Top.rotation.y,i.Top.rotation.z,i.Top.rotation.w])
                            r = R.from_quat(rot)
                            cam2ref_r = r.as_matrix()
                            ref2cam_r =np.linalg.inv(cam2ref_r)
                            temp = np.hstack((ref2cam_r,pos.transpose()))
                            ref2cam_tf = np.vstack((temp,[[0,0,0,1]]))
                            cam2ref = np.linalg.inv(ref2cam_tf)
                            
                            # print(f'ref2cam_tf {ref2cam_tf}')
                            #guard_log.info(f'ref2cam_tf {ref2cam_tf}')
                    return cam2ref
                cam2ref = compute_cam2ref(cam,reference_marker)
                ref2cam = np.linalg.inv(cam2ref)
                global current_guard,current_guard_cam

                        
                if guard_ref[0] == 0 :
                   
                    current_guard = []
                    return {'GuardPos':[]},200
            
                guard_pos = []
                fiducial=cam.GetCurrentMarkerData()
                
                print(guard_ref)
                
                guard_reg = guard_ref
                
                # get_data=cam.Get_camera_quats(geometry)
                Fiducial_list=np.array(fiducial.FiducialDataList)
                
                fid_list=np.hstack((Fiducial_list,np.ones((len(Fiducial_list),1))  ))
                fid_ref = cam2ref@fid_list.transpose()
                fid_ref_xyz = np.array(fid_ref.transpose()[:,:3])
                #print(f'fiducials in  reference space{fid_ref_xyz}')
                #guard_log.info(f'fiducials in  reference space{fid_ref_xyz}')

            


                distance = []
                for i in range(len(Fiducial_list)):   
                    distance.append(norm(fid_ref_xyz[i]-np.array(guard_reg)))

                    if len(distance) > 0:
                        arr = distance
                        minElement = np.amin(arr)
                        print(f'The closest fiducials distance to guard marker {minElement} ')
                    
                        result = np.where(arr == np.amin(arr))
                        # print('index',result[0][0])
                        # print(fiducials[result[0][0]])
                        if minElement < distance_threshold:
                            guard_pos = fid_ref_xyz[result[0][0]].tolist()
                print(f'guard pos reference space {guard_pos}') 
                guard_pos_cam_hom = ref2cam@np.append(guard_pos,1)
                guard_pos_cam = guard_pos_cam_hom[:3]
                print(f'guard pos in camera space')
                guard_log.info(f'guard pos {guard_pos}')           
                cam.allMarkerData.guardMarker = guard_pos
                
                current_guard = guard_pos
                current_guard_cam = guard_pos_cam

                cam.GetCurrentMarkerData()
                return {'GuardPos':guard_pos} , 200   
        else:
            current_guard = []
            return {'GuardPos':[]},200
    except:
        current_guard =  []
        return{"GuardPos":[] },200
        
#InternalReinitialisation
def reinitialise():
    dirname=os.getcwd()
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,10000)
    geo_path_rein= red.decode('utf-8')
    global initialise_status
    geo = next(walk(geo_path_rein), (None, None, []))[2]  # [] if no file
    
    cam.ConnectCamera()
    status_rein = cam.status
    if cam.status == 1:
        cam.LoadGeometryFiless(geo,geo_path_rein)
    geo_status_rein = cam.geo_status
    # status_rein,geo_status_rein = checkConnection(geo_path_rein)
    if status_rein == 1:
        camera_data_log.info('reinitialization status : 1')
        initialise_status_rein =1
        global register
        register = Registration(cam,geo_path_rein)
    else:
        camera_data_log.info('reinitialization status : 0')
        initialise_status_rein =0
    print(geo_status_rein)
    initialise_status = initialise_status_rein
    return initialise_status_rein 

#Connection Check Thread
qCs= Queue()
def connectionCheckWithDelay():
    connection_stats=deque([0])
    while True:
        resp = connectionCheck()
        qCs.put(resp)
        global connectStatus
        global initialise_status
        connectStatus = qCs.get()
        camera_data_log.info(f"Connection Status: {connectStatus}")
        connection_stats.append(connectStatus)
        while len(connection_stats) == 2:
            
            intdiff_buf= np.diff(connection_stats)
            # Internal Initialization
            if intdiff_buf==[1] and connectStatus == 1 and initialise_status==0:
                
                global qin,qiout
                dirname=os.getcwd()
                filename=os.path.join(dirname,'env\GeometryFolderName.txt')
                fd=os.open(filename, os.O_RDONLY)
                red=os.read(fd,10000)
                collec_path= red.decode('utf-8')
                
                camera_data_log.info(f"Geometry File Path: {collec_path}")
                global geo_status
                path_p= os. getcwd()
                thread_init = Thread(target=initialisation_thread)
                thread_init.daemon = True
                
                prev_dir = os.path.abspath(os.path.join(path_p, os.pardir))
                # geo_path = prev_dir+collec_path
                # qin.put(geo_path)
                # thread_init.start()
                status,geo_status = checkConnection((prev_dir+collec_path))
                # args = qiout.get()
                # status = args[0]
                # geo_status = args[1]
                if status == 1:
                    global register 
                    register = Registration(cam,prev_dir+collec_path)
                    dirnam=os.getcwd()
                    filena=os.path.join(dirnam,'env\GeometryFilePath.txt')
                    with open(filena,'w') as f:
                        f.write(prev_dir+collec_path)
                    
                # global initialise_status
                initialise_status = status
                print(geo_status)
                camera_data_log.info(f"Initialisation Status: {initialise_status}")
                connection_stats.popleft()
            
            diff_buf= np.diff(connection_stats)
            #Re-Initialization
            if diff_buf == [1] and connectStatus== 1 and initialise_status==1:
                initialise_status=0
                print("Gone in")
                print("Deleting Previous Object")
                del cam.tracking_system
                del cam.frame
                print("Initialising")
                time.sleep(initi_delay)
            
                time.sleep(1)
                cam.tracking_system = tracker.TrackingSystem()
                cam.frame = tracker.FrameData()
                camera_data_log.info("Re Initialising")
                reinitialise()
                
            else:
                pass
            connection_stats.popleft()
       

connec=Thread(target=connectionCheckWithDelay)
connec.daemon = True
connec.start()
      

def marker_recal(geometry_id,sample_size):
    end = time.time() + 60
    global reg_counter
    reg_counter = 0
    # Prepare data for a maximum of 6 fiducials
    data = [[],[],[],[],[],[]]
    ErrorMes = ""
    data_marker = []
    position_val=[]
    # Data aquisition
    #recco_stat = cam.tracking_system.set_int_option('New marker reco algorithm',1)
    tol_stat = cam.tracking_system.set_float_option('Distance matching tolerance',1)
    tool_registration_log.info('Marker Recalibration Started')
    while reg_counter<sample_size:
        
        if time.time() < end:
            if cam.tracking_system.get_last_frame(100, cam.frame) == tracker.Status.Ok:
                status_flag = 1
                matching_markers = [m for m in cam.frame.markers if m.geometry_id ==geometry_id]
                if len(matching_markers) == 1:
                    for fiducial_id in range(0, 6):
                        fiducial = matching_markers[0].corresponding_fiducial(fiducial_id)
                        if fiducial.valid:
                            data[fiducial_id].append(fiducial.position)
                            p = np.array(fiducial.position)
                            position_val.append(p)
                            # print(fiducial.position)
                            reg_counter+=1
                            print(reg_counter)
                    data_marker.append( [ matching_markers[0].position, \
                                        matching_markers[0].rotation[0], \
                                        matching_markers[0].rotation[1], \
                
                                        matching_markers[0].rotation[2] ] )
                else:
                    print("Marker Not Visibile")
                    tool_registration_log.info("Marker Not Visibile")
                    status_flag =0 
                    ErrorMes = "Marker Not Visibile"
                    
                    
            else:
                status_flag = 0
                print("Not able to get frame")
                tool_registration_log.info("Not able to get frame")
                ErrorMes = "Not able to get frame"
        else:
            print('Time Out')
            break
        
    if status_flag == 1:
        print("Number of registred frames: {0}".format(len(data_marker)))
                    
        data_np = np.array([x for x in data if x != []])
        data_marker_np = np.array(data_marker)

        nbr_frames,_,_ = data_marker_np.shape
        try:
            nbr_fiducials,__,__ = data_np.shape
        except:
            _,nbr_fiducials,_ = data_marker_np.shape


        # Compute the `marker' position with an additional point: the previously computed centre
        size = (nbr_fiducials, nbr_frames, 3)

        
        if nbr_fiducials ==3:
            cam2tool_pos=[]
            fiducial_0=[]
            fiducial_1=[]
            fiducial_2=[]
        
            for i_frame in range(nbr_frames):
                
                # For each frame, get the transformation which expresses the device coordinate into marker coordinates
                hom_trans = np.zeros((4,4))
                hom_trans[3,3] = 1
                hom_trans[0,0:3] = data_marker_np[i_frame,1]
                hom_trans[1,0:3] = data_marker_np[i_frame,2]
                hom_trans[2,0:3] = data_marker_np[i_frame,3]
                hom_trans[0:3,3] = data_marker_np[i_frame,0]
                cam2tool = np.linalg.inv(hom_trans)
                arr = np.zeros((4,3))
                arr[3,0:4] = 1
                arr[0:3,0]= data[0][i_frame]
                arr[0:3,1]=data[1][i_frame]
                arr[0:3,2]=data[2][i_frame]
                
                final_pos = cam2tool@arr
                cam2tool_pos.append(final_pos[:3])
                fiducial_0.append(final_pos[:3,0])
                fiducial_1.append(final_pos[:3,1])
                fiducial_2.append(final_pos[:3,2])
                
                
                
                # # For each fiducial, invert its coordinates.
                # for i_fid in range(nbr_fiducials):
                #     hom_pos = np.zeros(4)
                #     hom_pos[0] = data_np[i_fid][i_frame][0]
                #     hom_pos[1] = data_np[i_fid][i_frame][1]
                #     hom_pos[2] = data_np[i_fid][i_frame][2]
                #     hom_pos[3] = 1
                #     tmp = hom_pos
                #     # tmp2 = np.matmul(inverted, hom_pos)
                #     data_np_in_marker[i_fid][i_frame][0] = tmp[0]
                #     data_np_in_marker[i_fid][i_frame][1] = tmp[1]
                #     data_np_in_marker[i_fid][i_frame][2] = tmp[2]

            fiducials_coordinates = np.zeros((nbr_fiducials, 3))
            local_dis=[]
            # for k in range(len(cam2tool_pos)):
            #     for l in range(len(cam2tool_pos)):
            #         disss= distance(cam2tool_pos[k],cam2tool_pos[l])
            #         local_dis.append(disss)
            #     if len(local_dis) == 500:
            #         break

            dist1=[]
            dist2=[]
            
            for i in range(len(fiducial_0)):
                dist1.append(distance(fiducial_0[i],fiducial_1[i]))
                dist2.append(distance(fiducial_0[i],fiducial_2[i]))
                
                
            inlier_dist1 = ransac(len(dist1),dist1)
            inlier_dist2 = ransac(len(dist2),dist2)
            
            fiducial_0 =np.array(fiducial_0)
            fiducial_1 =np.array(fiducial_1)
            fiducial_2 =np.array(fiducial_2)
            
            
            fiducial_0 = fiducial_0[np.logical_and(inlier_dist1,inlier_dist2)]
            fiducial_1 = fiducial_1[np.logical_and(inlier_dist1,inlier_dist2)]
            fiducial_2 = fiducial_2[np.logical_and(inlier_dist1,inlier_dist2)]
            
            
            fiducial_0 = np.mean(fiducial_0,axis=0)
            fiducial_1 = np.mean(fiducial_1,axis=0)
            fiducial_2 =np.mean(fiducial_2,axis=0)
            
            

            marker_value= np.zeros((3,3))
            marker_value[0,:3] = fiducial_0 - fiducial_0
            marker_value[1,:3] = fiducial_1 - fiducial_0
            marker_value[2,:3] = fiducial_2 - fiducial_0
            
        if nbr_fiducials == 4:
            cam2tool_pos=[]
            fiducial_0=[]
            fiducial_1=[]
            fiducial_2=[]
            fiducial_3=[]
            for i_frame in range(nbr_frames):
                
                # For each frame, get the transformation which expresses the device coordinate into marker coordinates
                hom_trans = np.zeros((4,4))
                hom_trans[3,3] = 1
                hom_trans[0,0:3] = data_marker_np[i_frame,1]
                hom_trans[1,0:3] = data_marker_np[i_frame,2]
                hom_trans[2,0:3] = data_marker_np[i_frame,3]
                hom_trans[0:3,3] = data_marker_np[i_frame,0]
                cam2tool = np.linalg.inv(hom_trans)
                arr = np.zeros((4,4))
                arr[3,0:4] = 1
                arr[0:3,0]= data[0][i_frame]
                arr[0:3,1]=data[1][i_frame]
                arr[0:3,2]=data[2][i_frame]
                arr[0:3,3]=data[3][i_frame]
                final_pos = cam2tool@arr
                cam2tool_pos.append(final_pos[:3])
                fiducial_0.append(final_pos[:3,0])
                fiducial_1.append(final_pos[:3,1])
                fiducial_2.append(final_pos[:3,2])
                fiducial_3.append(final_pos[:3,3])
                
                
                # # For each fiducial, invert its coordinates.
                # for i_fid in range(nbr_fiducials):
                #     hom_pos = np.zeros(4)
                #     hom_pos[0] = data_np[i_fid][i_frame][0]
                #     hom_pos[1] = data_np[i_fid][i_frame][1]
                #     hom_pos[2] = data_np[i_fid][i_frame][2]
                #     hom_pos[3] = 1
                #     tmp = hom_pos
                #     # tmp2 = np.matmul(inverted, hom_pos)
                #     data_np_in_marker[i_fid][i_frame][0] = tmp[0]
                #     data_np_in_marker[i_fid][i_frame][1] = tmp[1]
                #     data_np_in_marker[i_fid][i_frame][2] = tmp[2]

            fiducials_coordinates = np.zeros((nbr_fiducials, 3))
            local_dis=[]
            # for k in range(len(cam2tool_pos)):
            #     for l in range(len(cam2tool_pos)):
            #         disss= distance(cam2tool_pos[k],cam2tool_pos[l])
            #         local_dis.append(disss)
            #     if len(local_dis) == 500:
            #         break

        
            dist1=[]
            dist2=[]
            dist3=[]
            for l in range(len(fiducial_0)):
                m1 = fiducial_0[l]
                m2 = fiducial_1[l]
                m3 = fiducial_2[l]
                v1 = unit_vector(m1-m2)
                v2 = unit_vector(m3-m2)
                vnorm = np.cross(v1,v2)
                zaxis_cam = np.array([0,0,1])
                temp = np.nan_to_num(angle_between(vnorm, zaxis_cam))
                
                if temp> 90:
                    angulation = 180 - temp
                else:
                    angulation = temp
                
                print(angulation)
                
                
            for i in range(len(fiducial_0)):
                dist1.append(distance(fiducial_0[i],fiducial_1[i]))
                dist2.append(distance(fiducial_0[i],fiducial_2[i]))
                dist3.append(distance(fiducial_0[i],fiducial_3[i]))
                
            inlier_dist1 = ransac(len(dist1),dist1)
            inlier_dist2 = ransac(len(dist2),dist2)
            inlier_dist3 = ransac(len(dist3),dist3)
            fiducial_0 =np.array(fiducial_0)
            fiducial_1 =np.array(fiducial_1)
            fiducial_2 =np.array(fiducial_2)
            fiducial_3 =np.array(fiducial_3)
            
            fiducial_0 = fiducial_0[np.logical_and(inlier_dist1,inlier_dist2,inlier_dist3)]
            fiducial_1 = fiducial_1[np.logical_and(inlier_dist1,inlier_dist2,inlier_dist3)]
            fiducial_2 = fiducial_2[np.logical_and(inlier_dist1,inlier_dist2,inlier_dist3)]
            fiducial_3 = fiducial_3[np.logical_and(inlier_dist1,inlier_dist2,inlier_dist3)]
            
            fiducial_0 = np.mean(fiducial_0,axis=0)
            fiducial_1 = np.mean(fiducial_1,axis=0)
            fiducial_2 = np.mean(fiducial_2,axis=0)
            fiducial_3 = np.mean(fiducial_3,axis=0)
            

            marker_value= np.zeros((4,3))
            marker_value[0,:3] = fiducial_0 - fiducial_0
            marker_value[1,:3] = fiducial_1 - fiducial_0
            marker_value[2,:3] = fiducial_2 - fiducial_0
            marker_value[3,:3] = fiducial_3 - fiducial_0
        
        if nbr_fiducials == 5:
            cam2tool_pos=[]
            fiducial_0=[]
            fiducial_1=[]
            fiducial_2=[]
            fiducial_3=[]
            fiducial_4 =[]
            for i_frame in range(nbr_frames):
                
                # For each frame, get the transformation which expresses the device coordinate into marker coordinates
                hom_trans = np.zeros((4,4))
                hom_trans[3,3] = 1
                hom_trans[0,0:3] = data_marker_np[i_frame,1]
                hom_trans[1,0:3] = data_marker_np[i_frame,2]
                hom_trans[2,0:3] = data_marker_np[i_frame,3]
                hom_trans[0:3,3] = data_marker_np[i_frame,0]
                cam2tool = np.linalg.inv(hom_trans)
                arr = np.zeros((4,5))
                arr[3,0:5] = 1
                arr[0:3,0]= data[0][i_frame]
                arr[0:3,1]=data[1][i_frame]
                arr[0:3,2]=data[2][i_frame]
                arr[0:3,3]=data[3][i_frame]
                arr[0:3,4]=data[4][i_frame]
                final_pos = cam2tool@arr
                cam2tool_pos.append(final_pos[:3])
                fiducial_0.append(final_pos[:3,0])
                fiducial_1.append(final_pos[:3,1])
                fiducial_2.append(final_pos[:3,2])
                fiducial_3.append(final_pos[:3,3])
                fiducial_4.append(final_pos[:3,4])
                
                
                # # For each fiducial, invert its coordinates.
                # for i_fid in range(nbr_fiducials):
                #     hom_pos = np.zeros(4)
                #     hom_pos[0] = data_np[i_fid][i_frame][0]
                #     hom_pos[1] = data_np[i_fid][i_frame][1]
                #     hom_pos[2] = data_np[i_fid][i_frame][2]
                #     hom_pos[3] = 1
                #     tmp = hom_pos
                #     # tmp2 = np.matmul(inverted, hom_pos)
                #     data_np_in_marker[i_fid][i_frame][0] = tmp[0]
                #     data_np_in_marker[i_fid][i_frame][1] = tmp[1]
                #     data_np_in_marker[i_fid][i_frame][2] = tmp[2]

            fiducials_coordinates = np.zeros((nbr_fiducials, 3))
            local_dis=[]
            # for k in range(len(cam2tool_pos)):
            #     for l in range(len(cam2tool_pos)):
            #         disss= distance(cam2tool_pos[k],cam2tool_pos[l])
            #         local_dis.append(disss)
            #     if len(local_dis) == 500:
            #         break

            dist1=[]
            dist2=[]
            dist3=[]
            dist4=[]
            for i in range(len(fiducial_0)):
                dist1.append(distance(fiducial_0[i],fiducial_1[i]))
                dist2.append(distance(fiducial_0[i],fiducial_2[i]))
                dist3.append(distance(fiducial_0[i],fiducial_3[i]))
                dist4.append(distance(fiducial_0[i],fiducial_4[i]))
                
                
            inlier_dist1 = ransac(len(dist1),dist1)
            inlier_dist2 = ransac(len(dist2),dist2)
            inlier_dist3 = ransac(len(dist3),dist3)
            inlier_dist4 = ransac(len(dist4),dist4)
            
            fiducial_0 =np.array(fiducial_0)
            fiducial_1 =np.array(fiducial_1)
            fiducial_2 =np.array(fiducial_2)
            fiducial_3 =np.array(fiducial_3)
            fiducial_4 =np.array(fiducial_4)
            inlier_sep = np.logical_and((inlier_dist1,inlier_dist2),(inlier_dist3,inlier_dist4))
            inlier_final = np.logical_and(inlier_sep[0],inlier_sep[1])
            
            fiducial_0 = fiducial_0[inlier_final]
            fiducial_1 = fiducial_1[inlier_final]
            fiducial_2 = fiducial_2[inlier_final]
            fiducial_3 = fiducial_3[inlier_final]
            fiducial_4 = fiducial_4[inlier_final]
            
            fiducial_0 = np.mean(fiducial_0,axis=0)
            fiducial_1 = np.mean(fiducial_1,axis=0)
            fiducial_2 =np.mean(fiducial_2,axis=0)
            fiducial_3 = np.mean(fiducial_3,axis=0)
            fiducial_4 = np.mean(fiducial_4,axis=0)
            

            marker_value= np.zeros((5,3))
            marker_value[0,:3] = fiducial_0 - fiducial_0
            marker_value[1,:3] = fiducial_1 - fiducial_0
            marker_value[2,:3] = fiducial_2 - fiducial_0
            marker_value[3,:3] = fiducial_3 - fiducial_0
            marker_value[4,:3] = fiducial_4 - fiducial_0
            
            print(marker_value)
            
        if nbr_fiducials ==6:
            cam2tool_pos=[]
            fiducial_0=[]
            fiducial_1=[]
            fiducial_2=[]
            fiducial_3=[]
            fiducial_4 =[]
            fiducial_5 =[]
            for i_frame in range(nbr_frames):
                
                # For each frame, get the transformation which expresses the device coordinate into marker coordinates
                hom_trans = np.zeros((4,4))
                hom_trans[3,3] = 1
                hom_trans[0,0:3] = data_marker_np[i_frame,1]
                hom_trans[1,0:3] = data_marker_np[i_frame,2]
                hom_trans[2,0:3] = data_marker_np[i_frame,3]
                hom_trans[0:3,3] = data_marker_np[i_frame,0]
                cam2tool = np.linalg.inv(hom_trans)
                arr = np.zeros((4,6))
                arr[3,0:6] = 1
                arr[0:3,0]= data[0][i_frame]
                arr[0:3,1]=data[1][i_frame]
                arr[0:3,2]=data[2][i_frame]
                arr[0:3,3]=data[3][i_frame]
                arr[0:3,4]=data[4][i_frame]
                arr[0:3,5]=data[5][i_frame]
                final_pos = cam2tool@arr
                cam2tool_pos.append(final_pos[:3])
                fiducial_0.append(final_pos[:3,0])
                fiducial_1.append(final_pos[:3,1])
                fiducial_2.append(final_pos[:3,2])
                fiducial_3.append(final_pos[:3,3])
                fiducial_4.append(final_pos[:3,4])
                fiducial_5.append(final_pos[:3,5])
                
                
                # # For each fiducial, invert its coordinates.
                # for i_fid in range(nbr_fiducials):
                #     hom_pos = np.zeros(4)
                #     hom_pos[0] = data_np[i_fid][i_frame][0]
                #     hom_pos[1] = data_np[i_fid][i_frame][1]
                #     hom_pos[2] = data_np[i_fid][i_frame][2]
                #     hom_pos[3] = 1
                #     tmp = hom_pos
                #     # tmp2 = np.matmul(inverted, hom_pos)
                #     data_np_in_marker[i_fid][i_frame][0] = tmp[0]
                #     data_np_in_marker[i_fid][i_frame][1] = tmp[1]
                #     data_np_in_marker[i_fid][i_frame][2] = tmp[2]

            fiducials_coordinates = np.zeros((nbr_fiducials, 3))
            local_dis=[]
            # for k in range(len(cam2tool_pos)):
            #     for l in range(len(cam2tool_pos)):
            #         disss= distance(cam2tool_pos[k],cam2tool_pos[l])
            #         local_dis.append(disss)
            #     if len(local_dis) == 500:
            #         break
        
            

            dist1=[]
            dist2=[]
            dist3=[]
            dist4=[]
            dist5=[]
            for i in range(len(fiducial_0)):
                dist1.append(distance(fiducial_0[i],fiducial_1[i]))
                dist2.append(distance(fiducial_0[i],fiducial_2[i]))
                dist3.append(distance(fiducial_0[i],fiducial_3[i]))
                dist4.append(distance(fiducial_0[i],fiducial_4[i]))
                dist5.append(distance(fiducial_0[i],fiducial_5[i]))
                
                
            inlier_dist1 = ransac(len(dist1),dist1)
            inlier_dist2 = ransac(len(dist2),dist2)
            inlier_dist3 = ransac(len(dist3),dist3)
            inlier_dist4 = ransac(len(dist4),dist4)
            inlier_dist5 = ransac(len(dist5),dist5)
            
            fiducial_0 =np.array(fiducial_0)
            fiducial_1 =np.array(fiducial_1)
            fiducial_2 =np.array(fiducial_2)
            fiducial_3 =np.array(fiducial_3)
            fiducial_4 =np.array(fiducial_4)
            fiducial_5 =np.array(fiducial_5)
            
            inlier_seg_1 = np.logical_and(inlier_dist1,inlier_dist2,inlier_dist3)
            inlier_final = np.logical_and(inlier_seg_1,inlier_dist4,inlier_dist5)
            
            fiducial_0 = fiducial_0[inlier_final]
            fiducial_1 = fiducial_1[inlier_final]
            fiducial_2 = fiducial_2[inlier_final]
            fiducial_3 = fiducial_3[inlier_final]
            fiducial_4 = fiducial_4[inlier_final]
            fiducial_5 = fiducial_5[inlier_final]
            
            fiducial_0 = np.mean(fiducial_0,axis=0)
            fiducial_1 = np.mean(fiducial_1,axis=0)
            fiducial_2 =np.mean(fiducial_2,axis=0)
            fiducial_3 = np.mean(fiducial_3,axis=0)
            fiducial_4 = np.mean(fiducial_4,axis=0)
            fiducial_5 = np.mean(fiducial_5,axis=0)

            marker_value= np.zeros((6,3))
            marker_value[0,:3] = fiducial_0 - fiducial_0
            marker_value[1,:3] = fiducial_1 - fiducial_0
            marker_value[2,:3] = fiducial_2 - fiducial_0
            marker_value[3,:3] = fiducial_3 - fiducial_0
            marker_value[4,:3] = fiducial_4 - fiducial_0
            marker_value[5,:3] = fiducial_5 - fiducial_0
            
            print(marker_value)
        import configparser
        import io

        new_geometry_id = geometry_id 
        geometry_ini = configparser.ConfigParser()
        
        geometry_ini.add_section('geometry')
        geometry_ini.set('geometry', 'count', str(nbr_fiducials))
        geometry_ini.set('geometry', 'id', str(new_geometry_id))
        
        for i in range(nbr_fiducials):
            geometry_ini.add_section("fiducial{0}".format(i))
            geometry_ini.set("fiducial{0}".format(i), 'x', "{0:.4f}".format(marker_value[i][0]))
            geometry_ini.set("fiducial{0}".format(i), 'y', "{0:.4f}".format(marker_value[i][1]))
            geometry_ini.set("fiducial{0}".format(i), 'z', "{0:.4f}".format(marker_value[i][2]))

        config_file_io = io.StringIO("")
        geometry_ini.write(config_file_io)
        cam.LoadGeometryFiless_in("geometry{0}.ini".format(new_geometry_id),cam.geo_path,"Unload")
        os.remove(cam.geo_path+"\\"+"geometry{0}.ini".format(new_geometry_id))
        
        with open(cam.geo_path+"\\"+"geometry{0}.ini".format(new_geometry_id), 'w', newline='\n') as config_file:
            config_file.write(config_file_io.getvalue().replace("\n\n","\n").replace(" = ","="))
        reload = cam.LoadGeometryFiless_in("geometry{0}.ini".format(new_geometry_id),cam.geo_path,"Load")
        if reload == 1:
            status_flag = 1
        else:
            status_flag = 0
        
        tool_registration_log.info('Marker Recalibration Done: %d',new_geometry_id)
        tool_registration_log.info(f'Marker Recalibration Status: {status_flag}')
        cam.tracking_system.set_float_option('Distance matching tolerance',0.5)
        
        return reg_counter,status_flag,new_geometry_id
    else:
        return 0,0,ErrorMes
class marker_recalibration(Resource):
    def get(self):
        parser = reqparse.RequestParser()
        parser.add_argument('data',required=True,type = str)

        args = parser.parse_args()
        if connectStatus == 1:
            data = args['data']
            retVal = json.loads(data)
            marker_name = retVal["MarkerName"]
            sampleSze = int(retVal["MaxPoints"])
            p_recal = Thread(target=recal_collect)
            p_recal.daemon = True
            
            procs_recal.append(p_recal)
            
            if marker_loaded_one(cam.geo_status,marker_name) == 0:
                tool_registration_log.error('Marker Recalibration, Geometry Files not loaded')
                return{'Status': '0', 
                       'ConnectionStatus': str(connectStatus)}
                
            count_alive_recalibration = 0
            for p_recal in procs_recal:
                count_alive_recalibration = count_alive_recalibration + int(p_recal.is_alive())
            print(f"No Process Running {count_alive_recalibration}")
            
            if count_alive_recalibration == 0:
                q.put([int(marker_name),sampleSze])
                global last_recal_process,flag
                last_recal_process = len(procs_recal)-1
                procs_recal[-1].start()
                flag = 3
                return {'Status': '1','ConnectionStatus': str(connectStatus)}
            else:
                return {'Status': '0','ConnectionStatus': str(connectStatus)}
        else:
            return{'Status': '0','ConnectionStatus': str(connectStatus)}
        
        
api.add_resource(marker_recalibration,'/MarkerRecalibration')        

def orientation_correction(tool_marker_name,wedge_marker_name,delay_w,samplesize):
    r = Registration(cam,collect_path)
    tool_registration_log.info('Orientation Correction Started')
    inine_vec = r.getwedge_x_axis(tool_marker_name,wedge_marker_name,delay_w,samplesize)
    rot_pivv = align_wedge("geometry"+str(tool_marker_name),-inine_vec)
    tool_registration_log.info('Orientation Corrected')
    reload_stat=cam.ReloadGeometry(int(tool_marker_name))
    
    return reload_stat

def enable_lasers(value):
    
    laserss = 0
    if value == "1":
       if cam.tracking_system.set_int_option("Enables lasers",3) == tracker.Status.Ok:
           laserss= 1
           camera_data_log.info('Lasers enabled')
       else:
           laserss = 0
           camera_data_log.error('Error, Lasers not enabled')
    elif value == "0":
        if cam.tracking_system.set_int_option("Enables lasers", 0) == tracker.Status.Ok:
            laserss = 2
            camera_data_log.info('Lasers disabled')
        else:
            laserss = 0
            camera_data_log.error('Error,Laser not disabled')
    
    return laserss

class laser_enable(Resource):
    def get(self):
        parser = reqparse.RequestParser()
        parser.add_argument('data',required=True,type = str)

        args = parser.parse_args()
        data = args['data']
        retVal = json.loads(data)
        mode = retVal["Mode"]
        laser_var = 4
        if connectStatus == 1:
            laser_status = enable_lasers (mode)
            if mode == '2':
                laser_s , laser_status = cam.tracking_system.get_int_option('Enables lasers')
                if laser_s == tracker.Status.Ok:
                    if laser_status != 3:
                        laser_var = 4
                    else:
                        laser_var = 3
                    
                else:
                    laser_var = 4
                return {'Status':str(laser_var),"Message": " "}
                    
            if laser_status == 1:
                return{"Status": "1","Message":"Laser Enabled"}
            else:
                return{"Status":"0","Message":"Laser Disabled"}
    
                
        else:
            return{"Status": "2","Message":"Camera Not Connected"}
            
api.add_resource(laser_enable,'/Lasers')
        
        
class wedge_based_orientation(Resource):
     def get(self):
        parser = reqparse.RequestParser()
        parser.add_argument('data',required=True,type = str)

        args = parser.parse_args()
        if connectStatus == 1:
            data = args['data']
            retVal = json.loads(data)
            tool_marker_name = retVal["ToolMarker"]
            wedge_marker_name= retVal["ReferenceMarker"]
            sampleSizeW = int(retVal["MaxPoints"])
            
            global geo_status
            if marker_loaded(geo_status,tool_marker_name,wedge_marker_name) == 0:
                return{"Status":"0", "ConnectionStatus":str(connectStatus),"ErrorMessage":"Marker Not Loaded"},200
            p_orientation = Thread(target=orientation_collect)
            p_orientation.daemon = True
            procs_orientation.append(p_orientation)
            
            count_alive_orientation = 0
            for p_or in procs_orientation:
                count_alive_orientation = count_alive_orientation + int(p_or.is_alive())
            print(f"No Process Running {count_alive_orientation}")
            
            
            if count_alive_orientation == 0:
                q.put([tool_marker_name,wedge_marker_name,0,sampleSizeW])
                global last_orient_process ,flag
                last_orient_process = len(procs_orientation)-1
                procs_orientation[-1].start()
                flag =4
                tool_registration_log.info('Orientation Corrected,Geometry file reloaded')
                return{"Status":"1","ConnectionStatus":str(connectStatus)}
            else:
                return{"Status":"0","ConnectionStatus":str(connectStatus)}
        else:
            return {"Status":"0","ConnectionStatus":str(connectStatus)}
        
api.add_resource(wedge_based_orientation,'/Orientation')

def single_pose_regis(reference_marker,end_effector,method,robot_pose):
    dock_pos = ""
    isVisibile = False
    status_of = 0
    try:
        robot_pose_new = robot_orientation_correction(robot_pose)
    except:
        return status_of ,"0",[0.0],"Orientation value cannot be fetched"
        
        
    
    if method == 'Without':
        #while isVisibile == False:
        geometry = [end_effector,reference_marker]
        cam.GetCurrentMarkerData()
        cma_data = cam.Get_camera_quats(geometry)
        if reference_marker in cma_data:
            ref_quat = cma_data[reference_marker][0]
            ref_pos = cma_data[reference_marker][1]
        else:
            status_of = 0
            return status_of ,"0",[0.0],"Reference marker not visible"
        
        if end_effector in cma_data:
            
            endef_quat = cma_data[end_effector][0]
            endef_pos = cma_data[end_effector][1] 
        else:
            status_of = 0
            return status_of ,"0",[0.0],"End effector marker not visible"
        try:     
        #cam2end_eff
            endeff2cam_r = R.from_quat(endef_quat).as_matrix().transpose()
            cam2endeff_r = np.linalg.inv(endeff2cam_r)
            endeff2cam_tf = rot2tf(endeff2cam_r,endef_pos)
            cam2endeff_tf = np.linalg.inv(endeff2cam_tf)
            y_axis_vector = endeff2cam_tf[:,:3]@ np.array([0,1,0])
            
            if np.sign(y_axis_vector[2]) == 1.0:
                status_of = 1
                dock_pos = "R"
                print (dock_pos)
            elif np.sign(y_axis_vector[2]) == -1.0:
                status_of = 1
                dock_pos = "L"
                print(dock_pos)
            
            pos_vector = np.array(endef_pos) -np.array(ref_pos)
            
            return status_of, dock_pos , [0.0],""
        except:
            return status_of, dock_pos , [0.0],""
            
        
    elif method == 'With':    
        geometry = [end_effector,reference_marker]
        cam.GetCurrentMarkerData()
        camera_data = cam.Get_camera_quats(geometry)
        if end_effector in camera_data:
            endef_quat = camera_data[end_effector][0]
            endef_pos = camera_data[end_effector][1] 
        else:
            status_of = 0
            return status_of ,"0",[0.0],"End effector marker not visible"
        if reference_marker in  camera_data :
            ref_quat = camera_data[reference_marker][0]
            ref_pos = camera_data[reference_marker][1]
        else:
            return status_of,"0", [0.0] , "Reference marker not visible"
            
    #Registration
        try:
            permutation_matrix = np.array(([0,1,0],[1,0,0],[0,0,-1]))
            endeff2cam_rot = R.from_quat(endef_quat).as_matrix().transpose()
            endeff2cam_rot = endeff2cam_rot @ permutation_matrix
            endeff2cam_tf = rot2tf(endeff2cam_rot,endef_pos)
            
            ref2cam_rot = R.from_quat(ref_quat).as_matrix().transpose()
            ref2cam_tf = rot2tf(ref2cam_rot,ref_pos)
            cam2ref_tf = np.linalg.inv(ref2cam_tf)
            ref_trans = ref2cam_tf @ np.array([0,0,0,1])
            
            
            
            
            # s = connect(robot_ip)
            # current_pose = getl(s)
            current_pose = robot_pose_new
            rot_mat = R.from_euler('xyz', current_pose[-3:], degrees=True).as_matrix()
            pos = current_pose[:3]
            endeff2robo_tf = rot2tf(rot_mat,pos)
            robo2eff_tf = np.linalg.inv(endeff2robo_tf)
            
            robo2cam_tf = endeff2cam_tf @ robo2eff_tf 
            robo_trans = robo2cam_tf @ np.array([0,0,0,1])
            
            direction_vector = robo_trans - ref_trans
            robo2ref_tf = cam2ref_tf @ robo2cam_tf
            ref2robo_tf = np.linalg.inv(robo2ref_tf)
            
            robo2cam_axes = robo2cam_tf[:3:,:3].transpose()
            if np.sign(direction_vector[0]) == -1.0:
                status_of = 1
                dock_pos = "R"
            elif np.sign(direction_vector[0]) == 1.0:
                status_of = 1
                dock_pos = "L"
            
            return status_of ,dock_pos,robo2cam_tf.tolist(),""
        except:
            return status_of ,"0",[0.0],"Computational error"
            
class SinglePose(Resource):
    def get(self):
        if connectStatus == 1:
            if initialise_status == 1:
                parser = reqparse.RequestParser()
                parser.add_argument('data',required=True,type = str)

                args = parser.parse_args()
                data = args['data']
                retVal = json.loads(data)
                ref_marker = retVal['ReferenceMarker']
                end_effector = retVal['EndEffectorMarker']
                meth = retVal['Method']
                robot_pose = retVal["RobotPose"]
                
                
                if marker_loaded_one(cam.geo_status,end_effector) == 0:
                    return {'Status':0,'DockStatus': "0", "TransformationMatrix": [0.0],"Message": "Marker not loaded"}
                
                if meth == 'With':
                    status_api , dock_statuss,tf,msg= single_pose_regis(ref_marker,end_effector,meth,robot_pose)
                    return {'Status':status_api,'DockStatus': dock_statuss, "TransformationMatrix": tf,"Message":msg}
                if meth == 'Without':
                    status_api,dock_statuss,tf,msg = single_pose_regis(ref_marker,end_effector,meth,robot_pose)
                    return {'Status':status_api,'DockStatus': dock_statuss, "TransformationMatrix": tf,"Message":msg}
            else:
                return {'Status':0,'DockStatus': "0", "TransformationMatrix": [0.0],"Message": "Camera not initialised"}
        else:
            return {'Status':0,'DockStatus': "0", "TransformationMatrix": [0.0],"Message": "Camera not connected"}
        
api.add_resource(SinglePose,'/SPR')



@app.route('/GetConnectionStatus',methods=["GET"])
def get_status():
    
    return {"ConnectionStatus": str(connectStatus),"InitialisationStatus":str(initialise_status)}
    

if __name__ == "__main__":
 
    dirname=os.getcwd()
    filename=os.path.join(dirname,'env\port.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,200)
    port= red.decode('utf-8')
    camera_data_log.info(port)
    tool_registration_log.info(port)
    guard_log.info(port)
    try:
        app.run(host=port, port=8081,debug=False)
        camera_data_log.info(f"Running in {port}")
        tool_registration_log.info(f"Running in {port}")
        guard_log.info(f"Running in {port}")
    except:
        print('check the port')
        camera_data_log.warning("Check the port")
        tool_registration_log.info("Check the port")
        guard_log.info("Check the port")
        
