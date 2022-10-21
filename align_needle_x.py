import configparser
import plotly.graph_objects as go
import numpy as np
from numpy.linalg import norm
import pandas as pd
# from pytest import Config
from functions import *
from scipy.spatial.transform import Rotation as R
np.set_printoptions(suppress=True)
# path = 'geometry8000029'
import os


def align_astura(path):
    config = configparser.ConfigParser()

    
    with open('env/GeometryFilePath.txt') as f:
        geo_path = f.readlines()[0]
    config.read(geo_path+"\\"+path+'.ini')
    
    keys = config.sections()
    marker_count = len(keys)-1
    inlineTop = 0
    inlineBottom = 3
    xCoord = []
    yCoord = []
    zCoord = []
    for k in keys:
        if k != 'geometry':
            xCoord.append (float(config[k]['x']))
            yCoord.append (float(config[k]['y']))
            zCoord.append (float(config[k]['z']))
            


    xx= np.array(xCoord)
    yy= np.array(yCoord)
    zz= np.array(zCoord)
    fids = np.vstack((xx,yy,zz)).transpose()
    inter_dist = []
    for i in range(marker_count):
        inter_dist.append(str(norm(fids[i])))
    xCoord.append(0)
    yCoord.append(0)
    zCoord.append(0)
    inter_dist.append('norm'+str(norm(fids[3])))
    inter_dist.append('norm'+str(00))


    conf = configparser.ConfigParser()
    fig = plot_fids(fids)
    print(fids)
    # fig.show()
    # inlineTop = int(input("Enter the index of the inline top marker"))
    # inlineBottom = int(input("Enter the index of the inline Bottom marker"))


    imd_array = combination(marker_count,fids)
    df_imd = pd.DataFrame(imd_array, columns = ['fid1','fid2','inter_marker'])
    df_imd.to_csv('Geometry/'+path+'.csv')
    theta = 39.53
    rot_fid_piv = rotateInline2theta(fids,inlineTop,inlineBottom,theta)
    fig = plot_fids(rot_fid_piv)
    # fig.show()

    for i in range(marker_count):
        conf.add_section('fiducial'+str(i))
    conf.add_section('geometry')
    
    num = 0
    for k in keys:
        if k != 'geometry':
            
            xval,yval,zval =  rot_fid_piv[num,:] 
            conf.set(k,'x',str(xval))
            conf.set(k,'y',str(yval))
            conf.set(k,'z',str(zval))
            num = num + 1
            
    id=path[8:]
    conf.set('geometry','count',str(marker_count))
    conf.set('geometry','id',id[:-1])
    cfgfile = open(geo_path+"\\"+path[:-1]+'.ini','w')
    conf.write(cfgfile,space_around_delimiters=False)
    cfgfile.close()  
    clean_ini(geo_path+"\\"+path[:-1]+'.ini')
    os.remove(geo_path+"\\"+path+'.ini')

    return rot_fid_piv

def align_x(path):
    config = configparser.ConfigParser()
    # path = 'geometry22309'
    
    with open('env/GeometryFilePath.txt') as f:
        geo_path = f.readlines()[0]
    config.read(geo_path+"\\"+path+'.ini')
    
    keys = config.sections()
    marker_count = len(keys)-1
    # inlineTop = 1
    # inlineBottom = 3
    xCoord = []
    yCoord = []
    zCoord = []
    for k in keys:
        if k != 'geometry':
            xCoord.append (float(config[k]['x']))
            yCoord.append (float(config[k]['y']))
            zCoord.append (float(config[k]['z']))
            


    xx= np.array(xCoord)
    yy= np.array(yCoord)
    zz= np.array(zCoord)
    fids = np.vstack((xx,yy,zz)).transpose()
    inter_dist = []
    for i in range(marker_count):
        inter_dist.append(str(norm(fids[i])))
    xCoord.append(0)
    yCoord.append(0)
    zCoord.append(0)
    inter_dist.append('norm'+str(norm(fids[3])))
    inter_dist.append('norm'+str(00))
    

    conf = configparser.ConfigParser()
    
    
    inlineConfig = configparser.ConfigParser()
    inlineConfig.read("Config\\toolConfig.ini")
    inlineKeys = inlineConfig.sections()
    inlineTop = 0
    inlineBottom = 0
    for ik in inlineKeys:
        if path == 'geometry'+ik+'9':
            inlineTop = int(inlineConfig.get(ik,'InlineTop'))
            inlineBottom =int( inlineConfig.get(ik,'InlineBottom'))
            
    if inlineTop == 0 and inlineBottom == 0:
        inlineTop = int(inlineConfig.get("common","top"))
        inlineBottom = int(inlineConfig.get("common","bottom"))

    imd_array = combination(marker_count,fids)
    df_imd = pd.DataFrame(imd_array, columns = ['fid1','fid2','inter_marker'])
    df_imd.to_csv('Geometry/'+path+'.csv')

    rot_fid_piv = rotateInline2X(fids,inlineTop,inlineBottom)
    # fig = plot_fids(rot_fid_piv)
    # fig.show()
    for i in range(marker_count):
        # section.append('fiducial'+str(i))
        conf.add_section('fiducial'+str(i))
    conf.add_section('geometry')

    num = 0
    for k in keys:
        if k != 'geometry':
            
            xval,yval,zval =  rot_fid_piv[num,:] 
            conf.set(k,'x',str(xval))
            conf.set(k,'y',str(yval))
            conf.set(k,'z',str(zval))
            num = num + 1
            
    id=path[8:]
    conf.set('geometry','count',str(marker_count))
    conf.set('geometry','id',id[:-1])
    cfgfile = open(geo_path+"\\"+path[:-1]+'.ini','w')
    conf.write(cfgfile,space_around_delimiters=False)
    cfgfile.close()  
    clean_ini(geo_path+"\\"+path[:-1]+'.ini')
    os.remove(geo_path+"\\"+path+'.ini')
    
    return rot_fid_piv

# path = 'geometry8000029'
# rot_fids = align_x(path)
def align_baseplate(path,inline_vec):
    config = configparser.ConfigParser()
    # path = 'geometry22309'
    
    with open('env/GeometryFilePath.txt') as f:
        geo_path = f.readlines()[0]
    config.read(geo_path+"\\"+path+'.ini')
    
    keys = config.sections()
    marker_count = len(keys)-1
    # inlineTop = 1
    # inlineBottom = 3
    xCoord = []
    yCoord = []
    zCoord = []
    for k in keys:
        if k != 'geometry':
            xCoord.append (float(config[k]['x']))
            yCoord.append (float(config[k]['y']))
            zCoord.append (float(config[k]['z']))
            


    xx= np.array(xCoord)
    yy= np.array(yCoord)
    zz= np.array(zCoord)
    fids = np.vstack((xx,yy,zz)).transpose()
    inter_dist = []
    for i in range(marker_count):
        inter_dist.append(str(norm(fids[i])))
    xCoord.append(0)
    yCoord.append(0)
    zCoord.append(0)
    inter_dist.append('norm'+str(norm(fids[3])))
    inter_dist.append('norm'+str(00))


    conf = configparser.ConfigParser()
    # fig = plot_fids(fids)
    # print(fids)
    # fig.show()
    # inlineTop = int(input("Enter the index of the inline top marker"))
    # inlineBottom = int(input("Enter the index of the inline Bottom marker"))
    # inline=configparser.ConfigParser()
    # inline.read("Config\\inline.ini")
    # if path=="geometry8882989":
    #     inlineTop=int(inline.get("drill","top"))
    #     inlineBottom=int(inline.get("drill","bottom"))
    # elif path=="geometry8883989":
    #     inlineTop=int(inline.get("tap","top"))
    #     inlineBottom=int(inline.get("tap","bottom"))
    # elif path == "geometry8884989":
    #     inlineTop=int(inline.get("pedicleprobe","top"))
    #     inlineBottom=int(inline.get("pedicleprobe","bottom"))
    # elif path=="geometry8885989":
    #     inlineTop=int(inline.get("awl","top"))
    #     inlineBottom=int(inline.get("awl","bottom"))
    # else:
    #     inlineTop = int(inline.get("needletrack","top"))
    #     inlineBottom = int(inline.get("needletrack","bottom"))

    imd_array = combination(marker_count,fids)
    df_imd = pd.DataFrame(imd_array, columns = ['fid1','fid2','inter_marker'])
    df_imd.to_csv('Geometry/'+path+'.csv')

    rot_fid_piv = rotateInline2X_baseplate(fids,inline_vec)
    # fig = plot_fids(rot_fid_piv)
    # fig.show()
    for i in range(marker_count):
        # section.append('fiducial'+str(i))
        conf.add_section('fiducial'+str(i))
    conf.add_section('geometry')

    num = 0
    for k in keys:
        if k != 'geometry':
            
            xval,yval,zval =  rot_fid_piv[num,:] 
            conf.set(k,'x',str(xval))
            conf.set(k,'y',str(yval))
            conf.set(k,'z',str(zval))
            num = num + 1
            
    id=path[8:]
    conf.set('geometry','count',str(marker_count))
    conf.set('geometry','id',id[:-1])
    cfgfile = open(geo_path+"\\"+path[:-1]+'.ini','w')
    conf.write(cfgfile,space_around_delimiters=False)
    cfgfile.close()  
    clean_ini(geo_path+"\\"+path[:-1]+'.ini')
    os.remove(geo_path+"\\"+path+'.ini')
   
    return rot_fid_piv

def align_wedge(path,inline_vec):
    config = configparser.ConfigParser()
    # path = 'geometry22309'
    
    with open('env/GeometryFilePath.txt') as f:
        geo_path = f.readlines()[0]
    config.read(geo_path+"\\"+path+'.ini')
    
    keys = config.sections()
    marker_count = len(keys)-1
    # inlineTop = 1
    # inlineBottom = 3
    xCoord = []
    yCoord = []
    zCoord = []
    for k in keys:
        if k != 'geometry':
            xCoord.append (float(config[k]['x']))
            yCoord.append (float(config[k]['y']))
            zCoord.append (float(config[k]['z']))
            


    xx= np.array(xCoord)
    yy= np.array(yCoord)
    zz= np.array(zCoord)
    fids = np.vstack((xx,yy,zz)).transpose()
    inter_dist = []
    for i in range(marker_count):
        inter_dist.append(str(norm(fids[i])))
    xCoord.append(0)
    yCoord.append(0)
    zCoord.append(0)
    inter_dist.append('norm'+str(norm(fids[3])))
    inter_dist.append('norm'+str(00))


    conf = configparser.ConfigParser()
    # fig = plot_fids(fids)
    # print(fids)
    # fig.show()
    # inlineTop = int(input("Enter the index of the inline top marker"))
    # inlineBottom = int(input("Enter the index of the inline Bottom marker"))
    # inline=configparser.ConfigParser()
    # inline.read("Config\\inline.ini")
    # if path=="geometry8882989":
    #     inlineTop=int(inline.get("drill","top"))
    #     inlineBottom=int(inline.get("drill","bottom"))
    # elif path=="geometry8883989":
    #     inlineTop=int(inline.get("tap","top"))
    #     inlineBottom=int(inline.get("tap","bottom"))
    # elif path == "geometry8884989":
    #     inlineTop=int(inline.get("pedicleprobe","top"))
    #     inlineBottom=int(inline.get("pedicleprobe","bottom"))
    # elif path=="geometry8885989":
    #     inlineTop=int(inline.get("awl","top"))
    #     inlineBottom=int(inline.get("awl","bottom"))
    # else:
    #     inlineTop = int(inline.get("needletrack","top"))
    #     inlineBottom = int(inline.get("needletrack","bottom"))

    imd_array = combination(marker_count,fids)
    df_imd = pd.DataFrame(imd_array, columns = ['fid1','fid2','inter_marker'])
    df_imd.to_csv('Geometry/'+path+'.csv')

    rot_fid_piv = rotateInline2X_baseplate(fids,inline_vec)
    # fig = plot_fids(rot_fid_piv)
    # fig.show()
    for i in range(marker_count):
        # section.append('fiducial'+str(i))
        conf.add_section('fiducial'+str(i))
    conf.add_section('geometry')

    num = 0
    for k in keys:
        if k != 'geometry':
            
            xval,yval,zval =  rot_fid_piv[num,:] 
            conf.set(k,'x',str(xval))
            conf.set(k,'y',str(yval))
            conf.set(k,'z',str(zval))
            num = num + 1
            
    id=path[8:]
    conf.set('geometry','count',str(marker_count))
    conf.set('geometry','id',id)
    cfgfile = open(geo_path+"\\"+path+'.ini','w')
    conf.write(cfgfile,space_around_delimiters=False)
    cfgfile.close()  
    clean_ini(geo_path+"\\"+path+'.ini')
    # os.remove(geo_path+"\\"+path+'.ini')
   
    return rot_fid_piv