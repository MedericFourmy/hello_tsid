import pinocchio as pin
import tsid
import numpy as np
import os
import gepetto.corbaserver
import time
import subprocess


class TsidManipulator:
    ''' Standard TSID formulation for a robot manipulator
        - end-effector task
        - Postural task
        - torque limits
        - pos/vel limits
    '''
    
    def __init__(self, model: pin.Model, conf: dict(), viewer=True):
        self.conf = conf
        robot = tsid.RobotWrapper(model, tsid.FIXED_BASE_SYSTEM, False)
        self.robot = robot
        self.model = model
        
        assert model.existFrame(conf.ee_frame_name)
        
        formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
        formulation.computeProblemData(0.0, conf.q0, conf.v0)  # TODO: necessary?
                
        postureTask = tsid.TaskJointPosture("task-posture", robot)
        postureTask.setKp(conf.kp_posture * np.ones(robot.nv))
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture) * np.ones(robot.nv))
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)
        
        self.eeTask = tsid.TaskSE3Equality("task-ee", self.robot, self.conf.ee_frame_name)
        self.eeTask.setKp(self.conf.kp_ee * np.ones(6))
        self.eeTask.setKd(2.0 * np.sqrt(self.conf.kp_ee) * np.ones(6))
        self.eeTask.setMask(conf.ee_task_mask)
        self.eeTask.useLocalFrame(conf.ee_task_local_frame)
        self.EE = model.getFrameId(conf.ee_frame_name)
        H_ee_ref = self.robot.framePosition(formulation.data(), self.EE)
        self.trajEE = tsid.TrajectorySE3Constant("traj-ee", H_ee_ref)
        formulation.addMotionTask(self.eeTask, conf.w_ee, 1, 0.0)
        
        self.tau_max = conf.tau_max_scaling*model.effortLimit
        self.tau_min = -self.tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(self.tau_min, self.tau_max)
        if(conf.w_torque_bounds>0.0):
            formulation.addActuationTask(actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)
        
        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
        self.v_max = conf.v_max_scaling * model.velocityLimit
        self.v_min = -self.v_max
        jointBoundsTask.setVelocityBounds(self.v_min, self.v_max)
        if(conf.w_joint_bounds>0.0):
            formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        # NO BINDINGS FOR THIS TASK!
        # jointBoundsTask = tsid.TaskJointPosVelAccBounds('task-joint-bounds', robot, conf.dt)
        # self.q_min = conf.q_min_scaling * model.lowerPositionLimit
        # self.q_max = conf.q_max_scaling * model.upperPositionLimit
        # self.v_max = conf.v_max_scaling * model.velocityLimit
        # jointBoundsTask.setVelocityBounds(self.v_max)
        # jointBoundsTask.setPositionBounds(self.q_min, self.q_max)
        # if(conf.w_joint_bounds>0.0):
        #     formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", conf.q0)
        postureTask.setReference(trajPosture.computeNext())
        
        solver = tsid.SolverHQuadProgFast("qp solver")
        solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)
        
        self.trajPosture = trajPosture
        self.postureTask  = postureTask
        # self.actuationBoundsTask = actuationBoundsTask
        # self.jointBoundsTask = jointBoundsTask
        self.formulation = formulation
        self.solver = solver
        self.q = conf.q0
        self.v = conf.v0


        # SETTING ARMATURE IN PYTHON NOT POSSIBLE
        # armature_scalar = 0.0
        # gear_ratios = np.matrix(np.ones(7))
        # rotor_inertias = armature_scalar*np.matrix(np.ones(7))
        # # robot.model().gear_ratios = gear_ratios 
        # # robot.model().rotor_inertias = rotor_inertias 
        # print(robot.model().rotorGearRatio)
        # print(robot.model().rotorInertia)
        # print(robot.gear_ratios)
        # print(robot.rotor_inertias)
        # # robot.set_gear_ratios(robot, gear_ratios)
        # # robot.set_rotor_inertias(robot, rotor_inertias)
        # # robot.gear_ratios = gear_ratios
        # # robot.rotor_inertias = rotor_inertias
        # # robot.gear_ratios(gear_ratios)
        # # robot.rotor_inertias(rotor_inertias)
        # # robot.updateMd()  # <==> croco "armature" but not binded
        # # robot.setGravity(pin.Motion.Zero())


        # formulation.computeProblemData(0.0, q, v)
        # tsid_data = formulation.data()
        # robot.computeAllTerms(tsid_data, q, v)

        # print('HEY')
        # print(robot.com(tsid_data))
        # robot.set_rotor_inertias(np.ones(7))
                
        # for gepetto viewer
        if(viewer):
            self.robot_display = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path, ])
            l = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
            if int(l[1]) == 0:
                os.system('gepetto-gui &')
            time.sleep(1)
            gepetto.corbaserver.Client()
            self.robot_display.initViewer(loadModel=True)
            self.robot_display.displayCollisions(False)
            self.robot_display.displayVisuals(True)
            self.robot_display.display(self.q)
            self.gui = self.robot_display.viewer.gui
#            self.gui.setCameraTransform(0, conf.CAMERA_TRANSFORM)
