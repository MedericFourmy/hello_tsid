import time
import numpy as np
import matplotlib.pyplot as plt
import plot_utils as plut
import pinocchio as pin
from tsid_manipulator import TsidManipulator

import panda_conf as conf

print(("".center(conf.LINE_WIDTH,'#')))
print((" TSID - Manipulator End-Effector Sin Tracking ".center(conf.LINE_WIDTH, '#')))
print(("".center(conf.LINE_WIDTH,'#')))
print("")

PLOT_EE_POS = 1
PLOT_EE_VEL = 1
PLOT_EE_ACC = 1
PLOT_JOINT_VEL = 1
PLOT_JOINT_ANGLE = 1
PLOT_TORQUES = 1

robot = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path])
tsid = TsidManipulator(robot.model, conf)

N = conf.N_SIMULATION
tau    = np.zeros((tsid.robot.na, N))
q      = np.zeros((tsid.robot.nq, N+1))
v      = np.zeros((tsid.robot.nv, N+1))
ee_pos = np.zeros((3, N))
ee_vel = np.zeros((3, N))
ee_acc = np.zeros((3, N))
ee_pos_ref = np.zeros((3, N))
ee_vel_ref = np.zeros((3, N))
ee_acc_ref = np.zeros((3, N))
ee_acc_des = np.zeros((3, N)) # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

sampleEE = tsid.trajEE.computeNext()
samplePosture = tsid.trajPosture.computeNext()

offset               = sampleEE.value()
offset[3:] = np.array([1., 0, 0, 0., 1, 0, 0., 0, 1]) # useless
offset[:3]          += conf.offset
two_pi_f_amp         = conf.two_pi_f * conf.amp
two_pi_f_squared_amp = conf.two_pi_f * two_pi_f_amp

pEE = offset.copy()
vEE = np.zeros(6)
aEE = np.zeros(6)

tsid.gui.addSphere('world/ee', conf.SPHERE_RADIUS, conf.EE_SPHERE_COLOR)
tsid.gui.addSphere('world/ee_ref', conf.REF_SPHERE_RADIUS, conf.EE_REF_SPHERE_COLOR)

t = 0.0
q[:,0], v[:,0] = tsid.q, tsid.v

for i in range(0, N):
    time_start = time.time()
    
    pEE[:3] = offset[:3] +  conf.amp * np.sin(conf.two_pi_f*t + conf.phi)
    vEE[:3] =  two_pi_f_amp * np.cos(conf.two_pi_f*t + conf.phi)
    aEE[:3] = -two_pi_f_squared_amp * np.sin(conf.two_pi_f*t + conf.phi)
    sampleEE.value(pEE)
    sampleEE.derivative(vEE)
    sampleEE.second_derivative(aEE)
    tsid.eeTask.setReference(sampleEE)

    HQPData = tsid.formulation.computeProblemData(t, q[:,i], v[:,i])
    # if i == 0: HQPData.print_all()

    sol = tsid.solver.solve(HQPData)
    if(sol.status!=0):
        print(("Time %.3f QP problem could not be solved! Error code:"%t, sol.status))
        break
    
    tau[:,i] = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)
    
    ee_pos[:,i] = tsid.robot.framePosition(tsid.formulation.data(), tsid.EE).translation
    ee_vel[:,i] = tsid.robot.frameVelocityWorldOriented(tsid.formulation.data(), tsid.EE).linear
    ee_acc[:,i] = tsid.eeTask.getAcceleration(dv)[:3]
    ee_pos_ref[:,i] = sampleEE.value()[:3]
    ee_vel_ref[:,i] = sampleEE.derivative()[:3]
    ee_acc_ref[:,i] = sampleEE.second_derivative()[:3]
    ee_acc_des[:,i] = tsid.eeTask.getDesiredAcceleration[:3]

    if i%conf.PRINT_N == 0:
        print(("Time %.3f"%(t)))
        print(("\ttracking err %s: %.3f"%(tsid.eeTask.name.ljust(20,'.'), 
                                          np.linalg.norm(tsid.eeTask.position_error, 2))))

    q[:,i+1], v[:,i+1] = tsid.integrate_dv(q[:,i], v[:,i], dv, conf.dt)
    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: 
        tsid.robot_display.display(q[:,i])
        tsid.gui.applyConfiguration('world/ee',     ee_pos[:,i].tolist()+[0,0,0,1.])
        tsid.gui.applyConfiguration('world/ee_ref', ee_pos_ref[:,i].tolist()+[0,0,0,1.])

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)

# PLOT STUFF
time = np.arange(0.0, N*conf.dt, conf.dt)

if(PLOT_EE_POS):
    f, ax = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_pos[i,:], label='EE '+str(i))
        ax[i].plot(time, ee_pos_ref[i,:], 'r:', label='EE Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE [m]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_VEL):
    f, ax = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_vel[i,:], label='EE Vel '+str(i))
        ax[i].plot(time, ee_vel_ref[i,:], 'r:', label='EE Vel Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Vel [m/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_ACC):    
    f, ax = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_acc[i,:], label='EE Acc '+str(i))
        ax[i].plot(time, ee_acc_ref[i,:], 'r:', label='EE Acc Ref '+str(i))
        ax[i].plot(time, ee_acc_des[i,:], 'g--', label='EE Acc Des '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Acc [m/s^2]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

nb_rows = int(tsid.robot.nv/2)+1
if(PLOT_TORQUES):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    for i in range(tsid.robot.nv):
        ax[i].plot(time, tau[i,:], label='Torque '+str(i))
        ax[i].plot([time[0], time[-1]], 2*[tsid.tau_min[i]], ':')
        ax[i].plot([time[0], time[-1]], 2*[tsid.tau_max[i]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Torque [Nm]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_JOINT_ANGLE):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    for i in range(tsid.robot.nv):
        ax[i].plot(time, q[i,:-1], label='Joint angle '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint angle [rad]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_JOINT_VEL):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    for i in range(tsid.robot.nv):
        ax[i].plot(time, v[i,:-1], label='Joint vel '+str(i))
        ax[i].plot([time[0], time[-1]], 2*[tsid.v_min[i]], ':')
        ax[i].plot([time[0], time[-1]], 2*[tsid.v_max[i]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint velocity [rad/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)



############################
# store trajectories in csv
import os
import pandas as pd

df_q = pd.DataFrame(q.T)
df_v = pd.DataFrame(v.T)
df_tau = pd.DataFrame(tau.T)


SAVE_DIR = '/home/mfourmy/Downloads/'
LABEL = 'slow'
traj_dir = os.path.join(SAVE_DIR, f'tsid_panda_traj_{LABEL}')
if not os.path.exists(traj_dir):
   os.makedirs(traj_dir)

df_q.to_csv(  os.path.join(traj_dir, 'q.csv'),   sep=',', header=False, index=False)
df_v.to_csv(  os.path.join(traj_dir, 'v.csv'),   sep=',', header=False, index=False)
df_tau.to_csv(os.path.join(traj_dir, 'tau.csv'), sep=',', header=False, index=False)

print('Saved q,v,tau in ', traj_dir)
############################



plt.show()
