import configparser
import logging
import numpy as np
from numpy.linalg import norm 
from scipy.spatial.transform import Rotation as R
from mpl_toolkits import mplot3d
# import matplotlib.pyplot as plt
import urllib3
import json
from sklearn import linear_model
from itertools import combinations
import plotly.graph_objects as go
from scipy.ndimage import median_filter
from scipy.spatial.distance import cdist, euclidean
import time
import os
import socket
http = urllib3.PoolManager(maxsize=10)
#logging
# logging.basicConfig(filename='Log\\function.log',format='%(asctime)s || %(levelname)s || %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',level=logging.DEBUG)

def marker_loaded(geo_status,tool_marker,reference_marker):
        marker_count = 0
        for markers in geo_status:
            if markers[0][8:-4] == reference_marker or markers[0][8:-4] == tool_marker:
                # print(f'{markers[0][8:-4]} marker loaded')
                marker_count += 1
            
        # print('count of existing',marker_count,tool_marker,reference_marker)
        if marker_count < 2:   
            return 0
        else:
            return 1

def marker_loaded_one(geo_status,markername):
    marker_ct=0
    for marker in geo_status:
        if marker[0][8:-4] == markername:
            marker_ct+=1
    if marker_ct == 1:
        return 1
    else:
        return 0

def guard_detect(tool_tip,reference_marker,delay):
    guard_pos = []
    ref_pos = []
    global dist_arr
    dist_arr = []
    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\port.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,20)
    port= red.decode('utf-8')
    print(port)
    
    r = http.request('GET','http://'+port+':8081/GetCameraData')
    logging.info(r)
    json_dict = json.loads(r.data)
    # print(json_dict)
    
    RegisteredMarkerCount =  len(json_dict['RegisteredMarkersList'])
    FiducialDataCount = len(json_dict['FiducialDataList'])
    print(f'fiducial data count : {FiducialDataCount}')
    logging.info(f"Fiducial Data Count: {FiducialDataCount}")
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
                    fiducials = fid_array
                    min_guard_dist =  norm(np.array(fid_array[i]) - np.array(position))
                    dist_arr.append(min_guard_dist)
                    # print(f'stray fiducial list {fiducials}')

        if len(dist_arr) > 0:
            arr = dist_arr
            minElement = np.amin(arr)
            print(f'The closest fiducials is at {minElement} mm distance from the tip: ')
            logging.info(f'The closest fiducials is at {minElement} mm distance from the tip: ')
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
        logging.info(f'reference and guard marker distance {ref_guard_dist}')

    return guard_pos,ref_pos

def clean_ini(filepath):
    def readfile(filepath): 
        with open(filepath, "r") as f: 
            for line in f:
                yield line

    lines = readfile(filepath)

    n_lines = ["\n%s" % line if "[Sect" in line else line for line in lines if line.strip()]

    f = open(filepath, "w")
    f.write("".join(n_lines).lstrip())
    f.close()
    
def data_fetch(method,tool_marker,reference_marker,delay,maxPoints):
    if method == 'baseplate' :
        vel_thresh = 0.1
        sample_size = 500
        lenFlag = True
        print('Please enter the aproximate length or place the tool in pivot point')

    elif method == 'static' :
        vel_thresh = 1.5
        sample_size = maxPoints
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
    while reg_counter < sample_size:            
        if time.time() < end_time  :   
            camera_data = Get_camera_quats(geometry)
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
                        print(f'count {reg_counter}')
        else:
            print('Time out')
            break
            
    tool_pos = np.array(needle_pos_list)
    ref_pos = np.array(ref_pos_list) 
    tool_quat = np.array(needle_quat_list)
    ref_quat = np.array(ref_quat_list)
    fid1 = np.array(fiducial1)
    fid2 = np.array(fiducial2)
    fid3 = np.array(fiducial3)
    fid4 = np.array(fiducial4)

    
    return(tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter)

def geometric_median(X, eps=1e-5):
    y = np.mean(X, 0)

    while True:
        D = cdist(X, [y])
        nonzeros = (D != 0)[:, 0]

        Dinv = 1 / D[nonzeros]
        Dinvs = np.sum(Dinv)
        W = Dinv / Dinvs
        T = np.sum(W * X[nonzeros], 0)

        num_zeros = len(X) - np.sum(nonzeros)
        if num_zeros == 0:
            y1 = T
        elif num_zeros == len(X):
            return y
        else:
            R = (T - y) * Dinvs
            r = np.linalg.norm(R)
            rinv = 0 if r == 0 else num_zeros/r
            y1 = max(0, 1-rinv)*T + min(1, rinv)*y

        if euclidean(y, y1) < eps:
            return y1

        y = y1

def distanceFromLine(p1,p2,p3):
    d = norm(np.cross(p2-p1, p1-p3))/norm(p2-p1)
    return d

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

def sphere(radius,centre,angle):
# The function will rotate the points and translate to the centre point
    vec = np.array([1,1,1])
    vec = (vec/norm(vec))*radius
    rot = R.from_quat([0,0,np.sin(np.pi/8),np.cos(np.pi/8)])
    rot2 = R.from_rotvec([np.random.randint(10)*0.1,np.random.randint(10)*0.1,np.deg2rad(angle)])
    rot_vec = rot2.apply(vec)
    rot_vec = rot_vec + centre

    return rot_vec

def rotate_vec(r,p_vec):
    rotated_vec = []
    for i in range (len(p_vec)):
        rot_vec =r[i].apply(p_vec[i])
        rotated_vec.append(rot_vec)  
    return np.array(rotated_vec)


def plot_vec(p_vec):
    xdata = np.array(p_vec).transpose()[0]
    ydata = np.array(p_vec).transpose()[1]
    zdata = np.array(p_vec).transpose()[2]

    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')
    



geometry = ['1500', '4200']
def Get_camera_quats(geometry):
    RegisteredMarkerCount = 0
    data = {}
    # r = http.request('GET','http://localhost:8081')
    # port = 'http://172.16.101.138'
    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\port.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,20)
    port= red.decode('utf-8')
        # print(port)
    try:
        r = http.request('GET',port+':8081/GetCameraData')
        
        json_dict = json.loads(r.data)
        # print(json_dict)
        RegisteredMarkerCount =  len(json_dict['RegisteredMarkersList'])
    # print(json_dict)
    except:
        print('Connection Error')
    

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
                    position = [pos['x'],pos['y'],pos['z'] ] 
                    position1 =[pos1['x'],pos1['y'],pos1['z'] ]
                    position2 =[pos2['x'],pos2['y'],pos2['z'] ]
                    position3 =[pos3['x'],pos3['y'],pos3['z'] ]
                    position4 =[pos4['x'],pos4['y'],pos4['z'] ]
                    quat = [ rot['x'],rot['y'],rot['z'],rot['w'] ]
                    #position 1 and position2 are random fidicial data of all the retro balls

                    data[Markers] = (quat,position,position1,position2,position3,position4) 
                    logging.info(f"{data[Markers]}")
                    

    else:
        print("Marker not visible")
        logging.error("Marker Not Visibile")
        
    
    return data


def ransac(n,pivot_len):
    X=np.linspace(1,n,n)[:,np.newaxis]
    y = pivot_len
    ransac = linear_model.RANSACRegressor()
    ransac.fit(X, y)
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    # plt.scatter(X[inlier_mask], y[inlier_mask], color='yellowgreen', marker='.',
    #             label='Inliers')
    # plt.scatter(X[outlier_mask], y[outlier_mask], color='gold', marker='.',
    #             label='Outliers')


    return inlier_mask

def major_minor_angle(pivot_vector):
    angle = []
    base_normal = np.mean(pivot_vector,axis = 0)
    for i in range(len(pivot_vector)):
        angle.append(angle_between(base_normal,pivot_vector[i]))

    # plt.ticklabel_format(useOffset=False)
    # plt.plot(angle)

    # plt.xlabel("sample")
    # plt.ylabel("angle in degrees")
    # plt.show(block=False)
    # plt.pause(2) # 3 seconds, I use 1 usually
    # plt.close("all")

    minor_angle = np.min(angle)
    major_angle = np.max(angle)
    return minor_angle,major_angle

def nor(v):
    return v/norm(v)

def ang(vector_1,vector_2):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle

def combination(marker_count,f):
    comb = combinations([x for x in range(marker_count)],2)

    imd = []
    for indices in comb:
    #     print(indices)
        dist = norm(f[indices[0]]-f[indices[1]])
        imd.append([indices[0],indices[1],dist])
        # print(np.rad2deg(ang(f[indices[0]],f[indices[1]])))
        # print(f[indices[0]],f[indices[1]])

    imd_array = np.array(imd)
    return imd_array

def rotateInline2X(fids,inlineTop,inlineBottom):
    v_inline = fids[inlineTop]-fids[inlineBottom]
    vector1 = v_inline
    if norm(vector1[1:]) != 0:
        a = vector1 / norm(vector1)
        b = np.array([-1,0,0])
        axis = np.cross(a,b)
        print(axis)
        axis = axis/norm(axis)

        angle = ang(a,b)
        r = R.from_rotvec(axis*angle)
        rot_fid = r.apply(fids-fids[inlineBottom,:])
        tip_vec = r.apply(-fids[inlineBottom,:])
        rot_fid_piv = rot_fid-tip_vec
    else:
        print("it is already inline with x axis")
        logging.info("It is already inline with x axis")
        rot_fid_piv = fids

    return rot_fid_piv
def rotateInline2theta(fids,inlineTop,inlineBottom,theta):
    v_inline = fids[inlineTop]-fids[inlineBottom]
    v_plane = fids[2]-fids[inlineBottom]
    vec_plane = v_plane/norm(v_plane)
    

    # if norm(v_inline[1:]) != 0:
    vec_diag = v_inline / norm(v_inline)
    vec_m_normal = np.cross(vec_plane,vec_diag)
    vec_m_normal = vec_m_normal/norm(vec_m_normal)  
    b = np.array([-1,0,0])
    
    if (b[0] == vec_diag[0]) & (b[1] == vec_diag[1]) & (b[2] == vec_diag[2]): 
        axis = np.array([0,0,-1])
    else:
        axis = np.cross(vec_diag,b)

    axis = axis/norm(axis)
    rot_tool = R.from_rotvec(vec_m_normal*np.deg2rad(theta))
    tool_axis = rot_tool.apply(vec_diag)
    org = [0,0,0]
    root = fids[inlineBottom]
    to_plot = np.vstack((org,root,root+vec_diag*50,root,root+tool_axis*50,root,fids[inlineTop]))
    fig = plot_fids(to_plot)
    # fig.show()
    print(np.rad2deg(ang(tool_axis,vec_diag)))

    angle = ang(tool_axis,b)
    print(f'angle to be rotated{np.deg2rad(angle)}')
    r = R.from_rotvec(axis*-angle)
    rot_fid = r.apply(fids-fids[inlineBottom,:])
    tip_vec = r.apply(-fids[inlineBottom,:])
    rot_fid_piv = rot_fid-tip_vec

    # else:
    #     print("it is already inline with x axis")
    #     rot_fid_piv = fids

    return rot_fid_piv

def rotateInline2Z(fids,inlineTop,inlineBottom):
    v_inline = fids[inlineTop]-fids[inlineBottom]
    vector1 = v_inline
    if norm(vector1[1:]) != 0:
        a = vector1 / norm(vector1)
        b = np.array([0,0,-1])
        axis = np.cross(a,b)
        print(axis)
        axis = axis/norm(axis)

        angle = ang(a,b)
        r = R.from_rotvec(axis*angle)
        rot_fid = r.apply(fids-fids[inlineBottom,:])
        tip_vec = r.apply(-fids[inlineBottom,:])
        rot_fid_piv = rot_fid-tip_vec
    else:
        print("it is already inline with y axis")
        logging.info("It is already inline with y axis")
        rot_fid_piv = fids

    return rot_fid_piv

def plot_fids(fids):
    fids=np.vstack((fids,[0, 0, 0]))
    fiducials = go.Scatter3d(
        x=fids[:,0], y=fids[:,1], z=fids[:,2],
        marker=dict(
            size=4,
            colorscale='Viridis',
        ),
        line=dict(
            color='darkblue',
            width=2
        )
    )
    axes   = go.Scatter3d( x = [0, 0,   0  , 100, 0, 0  ],
                           y = [0, 100, 0  , 0,   0, 0  ],
                           z = [0, 0,   0  , 0,   0, 100],
                           marker = dict( size = 1,
                                          color = "rgb(84,48,5)"),
                           line = dict( color = "rgb(84,48,5)",
                                        width = 6)
                         )
    data = [fiducials,axes]
    name = 'default'
# Default parameters which are used when `layout.scene.camera` is not provided
    camera = dict(
        up=dict(x=-1, y=0, z=0),
        center=dict(x=0, y=0, z=0),
        eye=dict(x=0, y=0, z=1.25)
        )

    fig = go.Figure(data=data)



    fig.update_layout(scene_camera=camera, title=name)

    fig.update_layout(
        scene = dict(
            xaxis = dict(nticks=4, range=[-800,800],),
            yaxis = dict(nticks=4, range=[-800,800],),
            zaxis = dict(nticks=4, range=[-800,800],),
            ),
            width=1000,
            margin=dict(r=20, l=10, b=10, t=10))
    return fig

def median3d(a,n,m):
    return median_filter(a,footprint = np.ones((n,m)))

def unit(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def getdivot_x_axis(df):
        
        
        
        
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
        return inline
    
def rot2tf(rot,pos):
    
    pos_s = np.array(pos)
    rot_matrix = rot
    temp= np.column_stack((rot_matrix.transpose(),pos_s))
    tf= np.vstack((temp,[0,0,0,1]))
    return tf

def rotateInline2X_baseplate(fids,divot_x):
    v_inline = divot_x
    vector1 = v_inline
    if norm(vector1[1:]) != 0:
        a = vector1 / norm(vector1)
        b = np.array([-1,0,0])
        axis = np.cross(a,b)
        print(axis)
        axis = axis/norm(axis)

        angle = ang(a,b)
        r = R.from_rotvec(axis*angle)
        rot_fid = r.apply(v_inline)
        tip_vec = r.apply(-fids)
        rot_fid_piv = rot_fid-tip_vec
    else:
        print("it is already inline with x axis")
        logging.info("It is already inline with x axis")
        rot_fid_piv = fids

    return rot_fid_piv

def connectionCheck():
    
    atarc = configparser.ConfigParser()
    atarc.read('Config\\NavigationConfig.ini')
    atarc_ip = atarc.get("Atracsys","IP")
    atarc_port = atarc.get("Atracsys","Port")
    IP=atarc_ip
    PORT = int(atarc_port)
    
    s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   
    time.sleep(2)
    res = s.connect_ex((IP,PORT))
    
    if res == 0:
        return 1
         
    else:
        return 0
        

def distance(p1,p2):
    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)
    # print(dist)
 
    return dist   