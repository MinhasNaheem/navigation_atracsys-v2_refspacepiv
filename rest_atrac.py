import logging
import atracsys.ftk as tracker
import os
import time
import json
from scipy.spatial.transform import Rotation as R
from os import walk
from flask import Flask
import numpy as np
# from flask_restful import Resource, Api, reqparse
app = Flask(__name__)
# api = Api(app)

with open('env/GeometryFilePath.txt') as f:
    geo_path = f.readlines()[0]
geo = next(walk(geo_path), (None, None, []))[2]  # [] if no file
tracking_system = tracker.TrackingSystem()
frame = tracker.FrameData()

def LoadGeometryFiles():
    geometry_path = tracking_system.get_data_option(geo_path)
    for geometry in geo:
        print(geometry)
        if tracking_system.set_geometry(geo_path+"\\"+ geometry) != tracker.Status.Ok:
            print("Error, can't create frame object. tracking_system")
            logging.error("Error, can't create frame object. tracking_system")
    if tracking_system.set_int_option("Strobe mode", 2) != tracker.Status.Ok:
        print("Error, disable strobe tracking_system")
        logging.error("Error, disable strobe tracking_system")

def ConnectCamera():
    print("Camera connection started")
    if tracking_system.initialise() != tracker.Status.Ok:
        print("Error, can't initialise the atracsys SDK api.")
    if tracking_system.enumerate_devices() != tracker.Status.Ok:
        print("Error, can't enumerate devices.")
    if tracking_system.create_frame(False, 10, 20, 20, 10) != tracker.Status.Ok:
        print("Error, can't create frame object.")
        status = 0
    else:
        status = 1            
    if tracking_system.set_int_option('Active Wireless Pairing Enable', 0) != tracker.Status.Ok:
        print("Error, can't enable wireless marker pairing.")
        IsCameraConnected = False  
    print("Camera detected successfully")
    return status

def MaintainActiveState():
    while True:
        ConnectCamera()
        time.sleep(5)

class CameraData:
    RegisteredMarkersList = []
    XPointsList = []
    QueryType = 'TestData'
    Version = 'V0.2'
    IsCameraDataObtained = False
    
    def __init__(self):
        self.RegisteredMarkersList = []
        self.XPointsList = []
        self.QueryType = 'TestData'
        self.Version = 0.2
        self.IsCameraDataObtained = False  

class Point:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class Point1:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0      

class Point2:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0    

class Point3:
    x = 0
    y = 0
    z = 0
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class Point4:
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
    point1 = Point1()
    point2 = Point2()
    point3 = Point3()
    point4 = Point4()
    rotation = Rotation()
    scale = Scale()
    angle = Angle()

    def __init__(self):
        self.point = Point()
        self.point1 = Point1()
        self.point2 = Point2()
        self.point3 = Point3()
        self.point4 = Point4()
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

def GetCurrentMarkerData():
    # tracking_system = tracker.TrackingSystem()
    # frame = tracker.FrameData()

    iscameraconnected = False
    allMarkerData = CameraData()
    connectioncheck = 0
    while iscameraconnected == False:
        t = tracking_system.get_last_frame(100, frame)
        allMarkerData = CameraData()
        # print("tracker status",t)
        if  t == tracker.Status.Ok:
            
            if len(frame.markers) !=0 or connectioncheck>5:
                iscameraconnected = True
            for x in frame.markers:
                
    #                 print("In for loop")
    #                 print(str(x.geometry_id))
    #                 print(x.position)
    #                 print(x.rotation)
    #                 print(x.registration_error)
    #                 print(x.status.left_edge)
    #                 print(x.index)
    #                 print(x.presence_mask)
    #                 print(x.tracking_id)
    #                 print(x.valid)
                
                registeredMarker = RegisteredMarker()
                registeredMarker.MarkerName = str(x.geometry_id)
                registeredMarker.MarkerBallsList = []
                registeredMarker.ErrorValue = x.registration_error
                registeredMarker.ErrorStatus = 'Enabled'

                translation = Point()
                translation.x = x.position[0]
                translation.y = x.position[1]
                translation.z = x.position[2]
                
 #                 changed the fiducail index from 0,1,2,3 to 4,5,6,7
                translation1 = Point1()
                translation2 = Point2()
                translation3 = Point3()
                translation4 = Point4()
                

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
                ang_temp = angle_between(v_norm,np.array([0,0,1]))
                if ang_temp > 90:
                    angulation.value = 180 - ang_temp
                else:
                    angulation.value = ang_temp  


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

                allMarkerData.RegisteredMarkersList.append(registeredMarker)
                
            connectioncheck = connectioncheck+1

        else :
            print("Not able to get frame from camera")
            

    return allMarkerData

def GetJSONFormatData(cameraData):
    cameraJson = {
        "QueryType": cameraData.QueryType,
        "Version": 'V0.2',
        "IsCameraDataObtained": True
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
    return json.dumps(cameraJson)

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))


def checkConnection(geo_path):
    status = 0
    geo = next(walk(geo_path), (None, None, []))[2]  # [] if no file
    geo_status = []
    
    # tracking_system = tracker.TrackingSystem()
    # frame = tracker.FrameData()
    def LoadGeometryFiless():
        for geometry in geo:    
            if tracking_system.set_geometry(geo_path+"\\"+ geometry) != tracker.Status.Ok:
                print("Error, can't create frame object. tracking_system")
                geo_status.append([geometry,0])
            else:
                geo_status.append([geometry,1])
        if tracking_system.set_int_option("Strobe mode", 2) != tracker.Status.Ok:
            print("Error, disable strobe tracking_system")
        

    status = ConnectCamera()
    LoadGeometryFiless()

    return status,geo_status
@app.route('/')  
def get_cam():
    
    markerdata = GetCurrentMarkerData()
    data = GetJSONFormatData(markerdata) 
    return data



if __name__ == '__main__':
    checkConnection(geo_path)
        
    with open('env/port.txt') as f:
        port = f.readlines()[0]
        print(port)
    try:
        app.run(host=port, port=8081)
    except:
        print('check the port')

    
