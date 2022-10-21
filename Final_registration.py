import numpy as np
import pandas as pd
from numpy import inf 
from numpy.linalg import norm 
from scipy.spatial.transform import Rotation as R
from functions import *
from  sksurgeryspherefitting.algorithms.sphere_fitting import fit_sphere_least_squares
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go
import matplotlib.pyplot as plt
from filter_func import *
from align_needle_x import align_x
import configparser
import os
np.set_printoptions(suppress=True)

def collect(tool_marker,reference_marker,delay,maxPoints):

    method = 'static'
    (tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter) =  data_fetch(method,tool_marker,reference_marker,delay,maxPoints)   
    if reg_counter <10:
        return 0,0
    my_array = np.hstack((tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4))
    df = pd.DataFrame(my_array, columns = ['toolx','tooly','toolz','tool_qx','tool_qy','tool_qz','tool_qw','refx','refy','refz','ref_qx','ref_qy','ref_qz','ref_qw','fid1x','fid1y','fid1z','fid2x','fid2y','fid2z','fid3x','fid3y','fid3z','fid4x','fid4y','fid4z'])
    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,100)
    collect_path= red.decode('utf-8')
    # _pivoting is for collect data(check for this file to avoid recollecting static data)
    df.to_csv(collect_path+'\Data\\'+tool_marker+'_collect.csv')
    print('Done')
    sd = np.std(norm(tool_pos,axis=1))
    return reg_counter,sd


def calibrate_tool(tool_marker, reference_marker,RegistrationType,delay):
    
    needle_marker = tool_marker
    maxPoints = 3000
    method = RegistrationType
    (tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4,reg_counter) =  data_fetch(method,tool_marker,reference_marker,delay,maxPoints)     

    if reg_counter < 10:
        return 0,0


    my_array = np.hstack((tool_pos,tool_quat,ref_pos,ref_quat,fid1,fid2,fid3,fid4))

    df = pd.DataFrame(my_array, columns = ['toolx','tooly','toolz','tool_qx','tool_qy','tool_qz','tool_qw','refx','refy','refz','ref_qx','ref_qy','ref_qz','ref_qw','fid1x','fid1y','fid1z','fid2x','fid2y','fid2z','fid3x','fid3y','fid3z','fid4x','fid4y','fid4z'])
    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,100)
    collect_path= red.decode('utf-8')
    Static_data_path = collect_path+'\Data\\' +needle_marker+'_collect.csv'
    Static_data = static_data(Static_data_path)

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
        offset,dataa = sphere_fit(df_in)
        cost = dataa.cost
        radii = norm(offset)
        print('The radius is',radii)
        print('The offset is',offset)

    if method == 'baseplate':
        filter = 'on'
        offset =  base_plate_registration(df_in,filter)



    df.to_csv(collect_path+needle_marker+'_whilePivot.csv')

    path = 'geometry' + needle_marker
    print('Final_reg',path)
    config = pivot_geometry(offset,path)

    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,100)
    geo_path= red.decode('utf-8')
    print(method)
    status = 0
    # check out the optimality parameter
    # if the file is over written, check the  contents of the file
    # if method == 'sphere' and radii > 100 and cost < 2 :
    if method == 'sphere' and cost < 2 :
        
        cfgfile = open(geo_path+"\\"+path+'9.ini','w')
        config.write(cfgfile,space_around_delimiters=False)
        cfgfile.close()
        rot_fids = align_x(path+'9')
        print('sphere fit', rot_fids)
        print(norm(rot_fids))
        status = 1
    # else:
        # cfgfile = open(geo_path+"\\"+path+'9.ini','w')
        # config.write(cfgfile,space_around_delimiters=False)
        # cfgfile.close()
        # rot_fids = align_x(path+'9')
        # pass
    print(reg_counter,cost,status)
    return reg_counter,cost,status

def sphere_fit(df):
    tool_pos = df[['toolx','tooly','toolz']].to_numpy()
    tool_quat = df[['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
    ref_pos = df[['refx','refy','refz']].to_numpy()
    centre = np.mean(tool_pos,axis=0)
    radius = 300

    xdata = np.array(tool_pos).transpose()[0]
    ydata = np.array(tool_pos).transpose()[1]
    zdata = np.array(tool_pos).transpose()[2]
    initial_parameters = np.array([centre[0],centre[1],centre[2],radius])
    data = fit_sphere_least_squares(xdata, ydata, zdata, initial_parameters, bounds=((-np.inf, -np.inf, -np.inf, -np.inf), (np.inf, np.inf, np.inf, np.inf)) )
    print("cost",data.cost)
    print("total fit info ",data)

    points = np.vstack((xdata,ydata,zdata)).transpose()
    pivot_point = data.x[:3]

    pivot_vector =  pivot_point - points
    local2tracker = R.from_quat(tool_quat) # input quaternions is tracker to local
    r = local2tracker.inv()
    p_vec_local = rotate_vec(r.inv(),pivot_vector)
    print("local tip",np.mean(p_vec_local,axis=0))

    tip_pos = rotate_vec(r,p_vec_local)+tool_pos

    err = tip_pos - ref_pos
    # plot_vec(err)
    # plt.xlabel('tip error')
    # plt.show(block=False)
    # plt.pause(10) # 3 seconds, I use 1 usually
    # plt.close("all")
    # minor_angle, major_angle = major_minor_angle(pivot_vector)
    # print(" min angle ",minor_angle," max angle ",major_angle)

    return np.mean(p_vec_local,axis=0),data

def base_plate_registration(df,filter):
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

def filt(fid1_pos,fid2_pos):
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


def pivot_geometry(pointer_offset,path):
    config = configparser.ConfigParser()
    configin = configparser.ConfigParser()
    dirname=os.path.dirname('rest_atracsys')
    filename=os.path.join(dirname,'env\GeometryFilePath.txt')
    fd=os.open(filename, os.O_RDONLY)
    red=os.read(fd,100)
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
