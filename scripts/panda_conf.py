# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

N_SIMULATION = 6000             # number of time steps simulated
dt = 0.001                      # controller time step

# REFERENCE SINUSOIDAL TRAJECTORY
amp                  = np.array([0.15, 0.3, 0.1])          # amplitude
# amp                  = np.array([0.0, 0.0, 0.0])           # amplitude
phi                  = np.array([0.0, 0.0, 0.0])         # phase
two_pi_f             = 2*np.pi*np.array([0.10, 0.10, 0.10])   # frequency (time 2 PI)
offset               = np.array([0.0, 0.0, 0.0])

w_ee = 1.0                      # weight of end-effector task
w_posture = 1e-3                # weight of joint posture task
w_torque_bounds = 0.0           # weight of the torque bounds
w_joint_bounds = 0.0            # weight of the joint bounds

kp_ee = 200.0                   # proportional gain of end-effector constraint
kp_posture = 100.0               # proportional gain of joint posture task

tau_max_scaling = 0.8           # scaling factor of torque bounds
v_max_scaling = 0.8             # scaling factor of velocity bounds
# q_max_scaling = 0.95             # scaling factor of joint config bounds

ee_frame_name = "panda_link8"        # end-effector frame name
ee_task_mask = np.array([1., 1, 1, 0, 0, 0])
# ee_task_mask = np.array([1., 1, 1, 1, 1, 1])
ee_task_local_frame = False      # specifies whether task is formulated in local frame

PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 20                  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [2.582354784011841, 1.620774507522583, 1.0674564838409424, 0.2770655155181885, 0.5401807427406311, 0.6969326734542847, 0.3817386031150818]
SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
EE_SPHERE_COLOR  = (1, 0.5, 0, 0.5)
EE_REF_SPHERE_COLOR  = (1, 0, 0, 0.5)
