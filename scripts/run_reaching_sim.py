import time
import numpy as np
import matplotlib.pyplot as plt
import plot_utils as plut
import pinocchio as pin

# from unified_simulators.pinocchio_sim import PinocchioSim as Simulator
from unified_simulators.pybullet_sim import PybulletSim as Simulator
from unified_simulators.utils import freezed_robot
from tsid_manipulator import TsidManipulator
import panda_conf as conf

np.set_printoptions(linewidth=150)


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

from example_robot_data import load

robot_name = 'panda'
robot = load(robot_name)
ee_name = 'panda_link8'
fixed_joints = ['panda_finger_joint1', 'panda_finger_joint2']
# fixed_joints = None
robot = freezed_robot(robot, fixed_joints)


# robot = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path])
# robot.model.gravity.linear = np.zeros(3)
robot.model.gravity.linear = np.array([0,0,-9.81])
tsid = TsidManipulator(robot, conf)

# Simulation
# Simulation
fixed_joints = ['panda_finger_joint1', 'panda_finger_joint2']
# fixed_joints = None
# robot = freezed_robot(robot, fixed_joints)

sim = Simulator()
sim.init(conf.dt, 'panda', fixed_joints, visual=True)
sim.setState(conf.x0)




N = conf.N_SIMULATION
tau    = np.zeros((tsid.robot_tsid.na, N))
q      = np.zeros((tsid.robot_tsid.nq, N+1))
v      = np.zeros((tsid.robot_tsid.nv, N+1))
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
    if i == 0: HQPData.print_all()

    sol = tsid.solver.solve(HQPData)
    if(sol.status!=0):
        print(("Time %.3f QP problem could not be solved! Error code:"%t, sol.status))
        break
    
    tau[:,i] = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)

    # print()
    # print('tau', tau[:,i])
    # print('dv', dv)

    # if i == 100:
    #     print(1/0)
    
    ee_pos[:,i] = tsid.robot_tsid.framePosition(tsid.formulation.data(), tsid.EE).translation
    ee_vel[:,i] = tsid.robot_tsid.frameVelocityWorldOriented(tsid.formulation.data(), tsid.EE).linear
    ee_acc[:,i] = tsid.eeTask.getAcceleration(dv)[:3]
    ee_pos_ref[:,i] = sampleEE.value()[:3]
    ee_vel_ref[:,i] = sampleEE.derivative()[:3]
    ee_acc_ref[:,i] = sampleEE.second_derivative()[:3]
    ee_acc_des[:,i] = tsid.eeTask.getDesiredAcceleration[:3]

    if i%conf.PRINT_N == 0:
        print(("Time %.3f"%(t)))
        print(("\ttracking err %s: %.3f"%(tsid.eeTask.name.ljust(20,'.'), 
                                          np.linalg.norm(tsid.eeTask.position_error, 2))))

    sim.step(tau[:,i])
    x = sim.getState()
    q[:,i+1], v[:,i+1] = x[:robot.nq], x[robot.nq:]

    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: 
        tsid.robot_display.display(q[:,i])
        tsid.gui.applyConfiguration('world/ee',     ee_pos[:,i].tolist()+[0,0,0,1.])
        tsid.gui.applyConfiguration('world/ee_ref', ee_pos_ref[:,i].tolist()+[0,0,0,1.])

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)

# PLOT STUFF
t_arr = np.arange(0.0, N*conf.dt, conf.dt)

if(PLOT_EE_POS):
    f, ax = plut.create_empty_figure(3,1)
    f.canvas.manager.set_window_title('EE Ref')
    for i in range(3):
        ax[i].plot(t_arr, ee_pos[i,:], label='EE '+str(i))
        ax[i].plot(t_arr, ee_pos_ref[i,:], 'r:', label='EE Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE [m]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_VEL):
    f, ax = plut.create_empty_figure(3,1)
    f.canvas.manager.set_window_title('EE Vel')
    for i in range(3):
        ax[i].plot(t_arr, ee_vel[i,:], label='EE Vel '+str(i))
        ax[i].plot(t_arr, ee_vel_ref[i,:], 'r:', label='EE Vel Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Vel [m/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_ACC):    
    f, ax = plut.create_empty_figure(3,1)
    f.canvas.manager.set_window_title('EE Acc')
    for i in range(3):
        ax[i].plot(t_arr, ee_acc[i,:], label='EE Acc '+str(i))
        ax[i].plot(t_arr, ee_acc_ref[i,:], 'r:', label='EE Acc Ref '+str(i))
        ax[i].plot(t_arr, ee_acc_des[i,:], 'g--', label='EE Acc Des '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Acc [m/s^2]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

nb_rows = int(tsid.robot_tsid.nv/2)+1
if(PLOT_TORQUES):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    f.canvas.manager.set_window_title('Joint Torques')
    for i in range(tsid.robot_tsid.nv):
        ax[i].plot(t_arr, tau[i,:], label='Torque '+str(i))
        ax[i].plot([t_arr[0], t_arr[-1]], 2*[tsid.tau_min[i]], ':')
        ax[i].plot([t_arr[0], t_arr[-1]], 2*[tsid.tau_max[i]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Torque [Nm]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_JOINT_ANGLE):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    f.canvas.manager.set_window_title('Joint Angles')
    for i in range(tsid.robot_tsid.nv):
        ax[i].plot(t_arr, q[i,:-1], label='Joint angle '+str(i))
        ax[i].plot([t_arr[0], t_arr[-1]], 2*[conf.q0[i]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint angle [rad]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_JOINT_VEL):    
    f, ax = plut.create_empty_figure(nb_rows,2)
    ax = ax.reshape(2*nb_rows)
    f.canvas.manager.set_window_title('Joint Velocities')
    for i in range(tsid.robot_tsid.nv):
        ax[i].plot(t_arr, v[i,:-1], label='Joint vel '+str(i))
        ax[i].plot([t_arr[0], t_arr[-1]], 2*[tsid.v_min[i]], ':')
        ax[i].plot([t_arr[0], t_arr[-1]], 2*[tsid.v_max[i]], ':')
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
