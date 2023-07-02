#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  4 15:37:56 2021

@author: yzx
"""
from evo.tools import file_interface
from evo.core import sync
import matplotlib.pyplot as plt
import evo.main_ape as main_ape
import evo.common_ape_rpe as common
from evo.core.metrics import PoseRelation
from evo.tools import plot
from evo.tools.plot import PlotMode
#**Load KITTI files** with entries of the first three rows of $\mathrm{SE}(3)$ matrices per line (no timestamps):

path = "/home/yzx/yzx/3D_reconstruction/Data/SCARED_tracking_results/Scaled/"
index = 1
traj_ref = file_interface.read_kitti_poses_file(path + "gtscaled"+str(index)+".txt")
traj_est = file_interface.read_kitti_poses_file(path + "endo"+str(index)+".txt")

print(traj_ref)
print(traj_est)

count = 0
results = []

est_name="APE Test #{}".format(count)
    

#ape(traj_ref: PosePath3D, traj_est: PosePath3D,
#        pose_relation: metrics.PoseRelation, align: bool = False,
#        correct_scale: bool = False, n_to_align: int = -1,
#        align_origin: bool = False, ref_name: str = "reference",
#        est_name: str = "estimate") -> Result

est_name = "est_name"
align = False
align_origin = False
correct_scale = False
pose_relation = PoseRelation.translation_part#rotation_angle_rad#rotation_angle_deg#translation_part
plot_mode = PlotMode.xyz

result = main_ape.ape(traj_ref, traj_est, est_name=est_name,
                      pose_relation=pose_relation, align=align,align_origin=align_origin, correct_scale=correct_scale)
count += 1
results.append(result)


fig = plt.figure()
ax = plot.prepare_axis(fig, plot_mode)
plot.traj(ax, plot_mode, traj_ref, style="--", alpha=0.5)
plot.traj_colormap(
    ax, result.trajectories[est_name], result.np_arrays["error_array"], plot_mode,
    min_map=result.stats["min"], max_map=result.stats["max"])


for key, value in result.stats.items():
    print(key, ' : ', value)