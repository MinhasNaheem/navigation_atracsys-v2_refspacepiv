import pandas as pd
import numpy as np
from  sksurgeryspherefitting.algorithms.sphere_fitting import fit_sphere_least_squares
from scipy.spatial.transform import Rotation as R
from functions import rot2tf,rotate_vec
from filter_func import pivot_data , pivot_data_filter,static_data
import matplotlib.pyplot as plt
from functions import plot_fids,unit
import os 
from numpy import set_printoptions
set_printoptions(suppress = True)

Static_data = static_data("PivotCsv\\8200098_whilePivot.csv")
df = pd.read_csv("PivotCsv\\8200098_whilePivot_last.csv")
Pivot_data = pivot_data(df,Static_data)

Filter = ['IMD_Intersection', 'RMS']
SD =[0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5 , 3]
df_filtered = pivot_data_filter(df,Filter[1],SD[5])
df_in = df_filtered


tool_pos = df_in[['toolx','tooly','toolz']].to_numpy()
tool_quat = df_in[['tool_qx','tool_qy','tool_qz','tool_qw']].to_numpy()
ref_pos = df_in[['refx','refy','refz']].to_numpy()
ref_quat = df_in[['ref_qx','ref_qy','ref_qz','ref_qw']].to_numpy()
centre = np.mean(tool_pos,axis=0)
radius = 300
tool_pos_ref_space = []
ref2tool_r = []
tool_pos_ref_normal = []
#Pivoting in reference space
for i in range(len(tool_pos)):
    tool2cam_r = R.from_quat(tool_quat[i]).as_matrix().transpose()
    tool2cam_tf = rot2tf(tool2cam_r,tool_pos[i])
    cam2tool_tf = np.linalg.inv(tool2cam_tf)
    ref2cam_r = R.from_quat(ref_quat[i]).as_matrix().transpose()
    ref2cam_tf = rot2tf(ref2cam_r,ref_pos[i])
    cam2ref_tf = np.linalg.inv(ref2cam_tf)
    tool_pos_hm = np.append(tool_pos[i],1)
    
    tool2ref_tf = cam2ref_tf @ tool2cam_tf
    ref2tool_tf = np.linalg.inv(tool2ref_tf)
    ref2tool_quat = R.from_matrix(ref2tool_tf[:3,:3]).as_quat()
    ref2tool_r.append(ref2tool_quat)
    tool_pos_in_ref = cam2ref_tf @ tool_pos_hm
    
    tool_pos_ref_space.append(tool_pos_in_ref[:3])
    tool_pos_ref_normal.append(1000*unit(tool_pos_in_ref[:3]))

center_ref =  cam2ref_tf @ np.append(centre,1)
ref2tool_r = np.array(ref2tool_r)
tool_pos_inRef = np.array(tool_pos_ref_space)
xref = tool_pos_inRef.transpose()[0]
yref = tool_pos_inRef.transpose()[1]
zref = tool_pos_inRef.transpose()[2]
center_refspace = center_ref[:3]
xdata = np.array(tool_pos).transpose()[0]
ydata = np.array(tool_pos).transpose()[1]
zdata = np.array(tool_pos).transpose()[2]
initial_parameters = np.array([100,0,0,radius])
bounds = 2000
data = fit_sphere_least_squares(xref, yref, zref, initial_parameters, bounds=((-bounds, -bounds, -bounds, -bounds), (bounds, bounds, bounds, bounds)) )
print("cost",data.cost)
print(data)
# xdata = np.array(tool_pos).transpose()[0]
# ydata = np.array(tool_pos).transpose()[1]
# zdata = np.array(tool_pos).transpose()[2]

points = np.vstack((xref,yref,zref)).transpose()
pivot_point = data.x[:3]

fid = plot_fids(np.vstack((points,pivot_point)))
fid.show()

pivot_vector =  pivot_point - points
local2tracker = R.from_quat(ref2tool_r) # input quaternions is tracker to local
r = local2tracker
p_vec_local = rotate_vec(r.inv(),-pivot_vector)
print(f'SD: {np.std(p_vec_local,axis=0)}')
print("local tip",np.mean(p_vec_local,axis=0))
# U,s,VT = np.linalg.svd(tool_pos_ref_space)
# fig = plt.figure()
# ax = plt.axes(projection ='3d')
# ax.plot(U[0],U[1],U[2])
# plt.show()

fid2 = plot_fids(tool_pos_ref_space)
fid2.show()
















