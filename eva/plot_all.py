#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 16 11:14:34 2021

@author: yzx
"""

import matplotlib.pyplot as plt
import numpy as np

def load_poses_txt(path_txt):
    file1 = np.loadtxt(path_txt)
    poses = []

    for line in file1:
        pose = line.reshape([3, 4])
        poses.append(pose)

    return poses

def poses2traj(pose_list):
    X = []
    Y = []
    Z = []
    for pose in pose_list:
        x, y, z = pose[:3, 3:]
        X.append(x[0])
        Y.append(y[0])
        Z.append(z[0])

    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)
    return X,Y,Z


#%%
index = 3

gt_txt = "/home/yzx/yzx/3D_reconstruction/Data/SCARED_tracking_results/Scaled/gtscaled" + str(index)+".txt"
orb_txt = "/home/yzx/yzx/3D_reconstruction/Data/SCARED_tracking_results/Scaled/orb" + str(index)+".txt"
end_txt = "/home/yzx/yzx/3D_reconstruction/Data/SCARED_tracking_results/Scaled/endo" + str(index)+".txt"
my_txt = "/home/yzx/yzx/3D_reconstruction/Data/SCARED_tracking_results/Scaled/my" + str(index)+".txt"

gt = load_poses_txt(gt_txt)
orb = load_poses_txt(orb_txt)
end = load_poses_txt(end_txt)
my = load_poses_txt(my_txt)

X_gt, Y_gt, Z_gt = poses2traj(gt)
X_orb, Y_orb, Z_orb = poses2traj(orb)
X_end, Y_end, Z_end = poses2traj(end)
X_my, Y_my, Z_my = poses2traj(my)
from mpl_toolkits.mplot3d import axes3d, Axes3D
fig = plt.figure()
#ax = plt.axes(projection='3d')
ax = Axes3D(fig)
ax.plot(X_gt, Y_gt, Z_gt, c='r',label="Ground Truth")
ax.plot(X_orb, Y_orb, Z_orb, c='g',label="ORB-SLAM2")
ax.plot(X_end, Y_end, Z_end, c='b',label="Endo-Depth")
ax.plot(X_my, Y_my, Z_my, c='m',label="END-VO")
# ax.scatter(-X_pred_my*1000 , -Y_pred_my*1000 , -Z_pred_my*1000 , c='g', marker='x')
# ax.scatter(-X_pred_my_deep * 1000, Y_pred_my_deep * 1000, -Z_pred_my_deep * 1000, c='b', marker='x')
ax.set_xlabel('X/mm')
ax.set_ylabel('Y/mm')
ax.set_zlabel('Z/mm')
fig.set_facecolor('white')
ax.set_facecolor('white') 


plt.legend(loc='upper left',fontsize=12)
plt.show()
