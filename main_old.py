from json import tool
import atracsys.ftk as tracker
import os
import time
import json
from scipy.spatial.transform import Rotation as R
from os import walk
from flask import Flask
from flask_restful import Resource, Api, reqparse
import pandas as pd
import ast
from functions import geometric_median, unit_vector,angle_between,guard_detect,marker_loaded

from Final_registration import calibrate_tool,collect
import numpy as np
from ast import literal_eval
from kill_port import kill
import threading
from multiprocessing import Process,Pipe
from flask import request
app = Flask(__name__)
api = Api(app)



with open('env/GeometryFilePath.txt') as f:
    geo_path = f.readlines()[0]

# geo_path = "C:\\Users\\minha\\AppData\\Local\\Atracsys\\PassiveTrackingSDK"
geo = next(walk(geo_path), (None, None, []))[2]  # [] if no file
tracking_system = tracker.TrackingSystem()
frame = tracker.FrameData()

markers = []
geo_status = []

sd = 0
count = 0
status = 0
global initialise_status, flag,procs,procs_piv
initialise_status = 0
count_alive_static = 0
count_alive_pivoting = 0
flag = 0
maxPoints = 0
procs = []
procs_piv = []


parent,child = Pipe()
parent_result,child_result = Pipe()

def collect_pipe(parent,child_result):
    global reg_counter,sd
    args = parent.recv()
    # print(args)
    reg_counter,sd = collect(args[0],args[1],args[2],args[3])
    child_result.send([reg_counter,sd])
    
parent_pivot,child_pivot = Pipe()

def pivot_pipe(parent_pivot,child_result):
    global status_piv,sd
    args = parent_pivot.recv()
    count,cost,status_piv = calibrate_tool(args[0],args[1],args[2],args[3])
    child_result.send([count,cost,status_piv])

def LoadGeometryFiles():
    geometry_path = tracking_system.get_data_option(geo_path)
    for geometry in geo:
        print(geometry)
        if tracking_system.set_geometry(geo_path+"\\"+ geometry) != tracker.Status.Ok:
            print("Error, can't create frame object. tracking_system")
    if tracking_system.set_int_option("Strobe mode", 2) != tracker.Status.Ok:
        print("Error, disable strobe tracking_system")

def ConnectCamera():
    print("Camera connection started")
    if tracking_system.initialise() != tracker.Status.Ok:
        print("Error, can't initialise the atracsys SDK api.")
    if tracking_system.enumerate_devices() != tracker.Status.Ok:
        print("Error, can't enumerate devices.")
#     frame = tracker.FrameData()
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
    FiducialDataList = []
    
    def __init__(self):
        self.RegisteredMarkersList = []
        self.XPointsList = []
        self.QueryType = 'TestData'
        self.Version = 0.2
        self.IsCameraDataObtained = False  
        self.FiducialDataList = [] 
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

def GetCurrentMarkerData():
    
    iscameraconnected = False
    allMarkerData = CameraData()
    connectioncheck = 0
    while iscameraconnected == False:
        t = tracking_system.get_last_frame(100, frame)
        allMarkerData = CameraData()
        if  t == tracker.Status.Ok:
            if len(frame.markers) !=0 or connectioncheck>5:
                iscameraconnected = True
            # print('frame.fiducials',type(frame.fiducials))
            if len(frame.fiducials) !=0:
                for fid in frame.fiducials:
                    # print(fid.position)
                    allMarkerData.FiducialDataList.append(fid.position)
            for x in frame.markers:
                
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
                angulation.value = angle_between(v_norm,np.array([0,0,1]))


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
    cameraJson["FiducialDataList"] = []
    for fiducial_pos in cameraData.FiducialDataList:
        cameraJson["FiducialDataList"].append(fiducial_pos)
    return json.dumps(cameraJson)

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

    

def reloadGeometryFiles():
    # out = tracking_system.allow_simulator 
    # out2 = tracking_system.enumerate_devices()
    # out3 = tracking_system.get_enumerated_devices()
    
    
    # out_status_1 = tracking_system.create_frame(False, 10, 20, 20, 10)
    global frame
    
    tracking_system.get_last_frame(100,frame) 
    print(geo_status)
    print(f'initial of geometry {[x.position[0] for x in frame.markers]}')

    tracking_system.unset_geometry('C:\\Users\\minha\\AppData\\Local\\Atracsys\\PassiveTrackingSDK\\geometry8100098.ini')
    tracking_system.get_last_frame(100,frame) 
    print(f'post unset of geometry {[x.position[0] for x in frame.markers]}')
    
    tracking_system.set_geometry('C:\\Users\\minha\\AppData\\Local\\Atracsys\\PassiveTrackingSDK\\geometry8100098.ini')
    tracking_system.get_last_frame(100,frame) 
    print(f'post set of geometry {[x.position[0] for x in frame.markers]}')

   
    return {'serial':'enumerated_list'},200


class reload(Resource):
    def get(self):
        reloadGeometryFiles()

api.add_resource(reload,'/ReloadGeometry') 
class initialise(Resource):
    
    def get(self):
        global initialise_status
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
                print(GeometryPath)

                global geo_status
                status,geo_status = checkConnection(GeometryPath)
                # global initialise_status
                initialise_status = status
                print(geo_status)
                with open('env/GeometryFilePath.txt', 'w') as f:
                    f.write(GeometryPath)
                # with open('env/port.txt', 'w') as f:
                #     f.write(args['CameraPort'])
                    
                return {'CameraConnectionStatus':str(status),
                        'GeometryFiles':geo_status,
                        'GeometryFilePath':GeometryPath,
                        'ErrorMessage':""
                },200
            except:
                return {'CameraConnectionStatus':str(0),
                        'GeometryFiles':str(0),
                        'GeometryFilePath':str(0),
                        'ErrorMessage':"Error in initialising"
                },200          
        else:
            return {'CameraConnectionStatus':str(1),
                    'GeometryFiles':str(0),
                    'GeometryFilePath':str(0 ),
                    'ErrorMessage':"Camera is already connected"
            },200              

api.add_resource(initialise,'/Initialization') 

class getCollectedPointCount(Resource):

    def get(self):
        # 0 for none,1 for static, 2 for pivoting
        global flag
        
        global procs,procs_piv
        count_alive_static = 0
        count_alive_pivoting = 0
        for pp in procs:
            count_alive_static = count_alive_static + int(pp.is_alive())  
        for p_piv in procs_piv:
            count_alive_pivoting = count_alive_pivoting + int(p_piv.is_alive())
    
        print(f'count alive static {count_alive_static}')
        if count_alive_static == 1:
            flag = 1
            # print(f'Thread name when alive {procs[-1].name}' )
            # print(os.getpid())
            return  {"Count":"0",
            "CollectionType":"static",
                    "ErrorMessage":"0",
                    'Status':"Collecting"},200
        if count_alive_pivoting == 1:
            flag = 2
            return  {"Count":"0",
            "CollectionType":"sphere",
                    "ErrorMessage":"0",
                    'Status':"Collecting"},200
        if count_alive_pivoting == 0 and count_alive_static == 0:
            if flag == 1:
                return {"Count":"0",
                    "CollectionType":"static",
                            "ErrorMessage":"0",
                            'Status':"Done"},200
            elif flag == 2:
                return {"Count":"0",
                    "CollectionType":"pivoting",
                            "ErrorMessage":"0",
                            'Status':"Done"},200
            else:
                return {"Count":"0",
                    "CollectionType":"null",
                            "ErrorMessage":"0",
                            'Status':"null"},200



api.add_resource(getCollectedPointCount,'/GetCollectedPointCount') 

class getCollectedPointStatus(Resource):
    
    
    def get(self):
        global last_process, flag , procs,procs_piv, parent_result,ma
        print(len(procs))
        if len(procs)>0:
            print(f'The type of collection flag {flag} last process{last_process}one {procs[last_process].exitcode} {len(procs)} {parent_result.poll(timeout = 0.4)} ')
            if procs[last_process].exitcode == 0  and parent_result.poll(timeout = 0.4) and flag == 1:
                # print(f'sentinal values {procs[last_process].sentinel}')
                args  = parent_result.recv()

                if args[0] == maxPoints:
                
                    return {'Status':'1',
                            'SD':str(args[1]),
                            'Cost':'0',
                            'ErrorMessage':'null'
                        },200
                else:
                    return {'Status':'0',
                            'SD':str(args[1]),
                            'Cost':'0',
                            'ErrorMessage':'null'
                        },200

        global last_piv_process
        try:
            if len(procs_piv) > 0 and procs_piv[last_piv_process].exitcode == 0 and parent_result.poll(timeout = 0.5) and flag == 2:
                args  = parent_result.recv()
                # 'Count':str(args[0])
                return {
                            'Cost':str(args[1]),
                            'Status':str(args[2]),
                            'SD':str(args[0]),
                            'ErrorMessage':'null'
                    },200
        except:
            return {
                            'Cost':'0',
                            'Status':'0',
                            'SD':'0',
                            'ErrorMessage':'processingHitAgain'
                    },200


        return {'Status':"null",
                    'SD':"null",
                    'Cost':'0',
                    'ErrorMessage':'null'
            },200


api.add_resource(getCollectedPointStatus,'/GetCollectedPointStatus') 

class collectStatic(Resource):

    def get(self):
        global initialise_status
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
                

                    global p
                    p = Process(target = collect_pipe, args=(parent,child_result,),daemon=True)

                    global geo_status,sd,procs
                    procs.append(p)

                    if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
                        return {'status':'marker not loaded',
                        },200
                    
                    count_alive_static = 0
                    for pp in procs:
                        count_alive_static = count_alive_static + int(pp.is_alive())    
                    print(f"Number of process static running {count_alive_static}")
                    global flag
                    if count_alive_static == 0: 
                        child.send([tool_marker,reference_marker,delay,maxPoints])
                        procs[-1].start()
                        global last_process 
                        last_process = len(procs)-1
                        flag = 1
                        # print(f'exit code post the process{procs[-1].exitcode}')
                        # procs[-1].join()
                        # print(f'exit code post the process{procs[-1].exitcode}')
                        return {'status':str(1)       
                        },200
                    else: 
                        return {'status':str(0)
                        },200

                else:   
                    return {'status':"formatError"       
                        },200


            except:
                return {'status':"Error"       
                        },200
        else:
            return {'status':"not initialised"       
                        },200       
api.add_resource(collectStatic,'/CollectStaticData') 
        
@app.route('/GetCollectStatus',methods=["GET"])  

def get_collect_status():
    global last_process
    if procs[last_process].exitcode == 0 and len(procs)>0 and parent_result.poll(timeout = 0.4):
        # print(f'sentinal values {procs[last_process].sentinel}')
        args  = parent_result.recv()
        return {'count':str(args[0]),
                'sd':str(args[1])
        },200
    return {'status':'processing'
        },200


        
class pivoting(Resource):
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
                RegistrationType = retVal['RegistrationType']
                StartDelay = int(retVal['StartDelay'])

                p_piv = Process(target = pivot_pipe, args=(parent_pivot,child_result,),daemon=True)
                
                global geo_status,sd,status
                procs_piv.append(p_piv)
                if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
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
                    child_pivot.send([tool_marker,reference_marker,RegistrationType,StartDelay])
                    global last_piv_process,flag
                    last_piv_process =  len(procs_piv)-1
                    procs_piv[-1].start()
                    flag = 2
                    return {'status':str(1) 
                            },200
                else:
                    return {'status':str(0) 
                            },200                   

            else:
                return {'status':f'FormatError'
                            },200
        except:
            return {'status':f'Error'
                        },200

api.add_resource(pivoting,'/StartPivoting')  

@app.route('/GetPivotStatus',methods=["GET"]) 

def get_pivot_status():
    global last_piv_process
   
    if len(procs_piv) > 0 and procs_piv[last_piv_process].exitcode == 0 and parent_result.poll(timeout = 0.5):
        args  = parent_result.recv()
        return {'count':str(args[0]),
                    'cost':str(args[1]),
                    'status':str(args[2])
            },200
    
    return {'status':f'processing'
    },200

    
class filter(Resource):
    def get(self):
        parser = reqparse.RequestParser()
        parser.add_argument('Data',required=True,type = str)

        args = parser.parse_args()
        
        filtered_data = geometric_median(np.array(literal_eval(args['Data'])))
        print(filtered_data)
        print(type(args['Data']))

            
        return {'Data':args['Data'],
                
                'Output':str(filtered_data)
        },200
class guard(Resource):
    def get(self):
        try:
            parser = reqparse.RequestParser()
            parser.add_argument('data',required=True,type = str)
            args = parser.parse_args()
            # print(args['data'])
            if "data" in args:
                data = args['data']
                retVal = json.loads(data)   
                tool_marker = retVal['ToolMarker']
                reference_marker = retVal['ReferenceMarker']
                delay = int(retVal['StartDelay'])
                global geo_status
            
                if marker_loaded(geo_status,tool_marker,reference_marker) == 0:
                    return {'GuardPosition':'marker not loaded',
                            'ReferencePosition':'marker not loaded'
                            },200
                else:

                    print(f'tool marker{tool_marker},reference marker{reference_marker},delay {delay}')
                    guard_pos,ref_pos = guard_detect(tool_marker,reference_marker,delay)

                    return {'GuardPosition':str(guard_pos),
                            'ReferencePosition' : str(ref_pos)
                    },200
            else:
                return {'GuardPosition':'formatError',
                        'ReferencePosition' : 'formatError'
                },200
        except:
            return {'GuardPosition':'Error',
                    'ReferencePosition' : 'Error'
            },200
api.add_resource(guard,'/Guard')  
    
# api.add_resource(filter,'/Filter')  
get_cam_error = {"QueryType": "TestData", "Version": "V0.2", "IsCameraDataObtained": False, "RegisteredMarkersList": [], "XPointsList":
[], "FiducialDataList": []}


@app.route('/GetCameraData',methods=["GET"])  
def get_cam():
    if initialise_status == 1:
        markerdata = GetCurrentMarkerData()
        data = GetJSONFormatData(markerdata) 
        return data
    else:
        
        return get_cam_error,200

if __name__ == "__main__":

    with open('env/port.txt') as f:
        port = f.readlines()[0]
        print(port)

    # status = ConnectCamera()
    # LoadGeometryFiles()  
    try:
        app.run(host=port, port=8081)
    except:
        print('check the port')





