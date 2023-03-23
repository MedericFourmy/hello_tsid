import os
import time
import subprocess
import numpy as np
import tsid
import pinocchio as pin
import gepetto.corbaserver


class TsidManipulator:
    ''' Standard TSID formulation for a robot_tsid manipulator
        - end-effector task
        - Postural task
        - torque limits
        - pos/vel limits
    '''
    
    def __init__(self, robot_pin, conf: dict(), viewer=True):
        model_pin = robot_pin.model
        self.conf = conf
        robot_tsid = tsid.RobotWrapper(model_pin, tsid.FIXED_BASE_SYSTEM, False)
        self.robot_tsid = robot_tsid
        
        assert model_pin.existFrame(conf.ee_frame_name)
        
        v0 = np.zeros(robot_pin.nv)
        formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot_tsid, False)
        formulation.computeProblemData(0.0, robot_pin.q0, v0)  # TODO: necessary?
                
        postureTask = tsid.TaskJointPosture("task-posture", robot_tsid)
        postureTask.setKp(conf.kp_posture * np.ones(robot_tsid.nv))
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture) * np.ones(robot_tsid.nv))
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)
        
        self.eeTask = tsid.TaskSE3Equality("task-ee", self.robot_tsid, self.conf.ee_frame_name)
        self.eeTask.setKp(self.conf.kp_ee * np.ones(6))
        self.eeTask.setKd(2.0 * np.sqrt(self.conf.kp_ee) * np.ones(6))
        self.eeTask.setMask(conf.ee_task_mask)
        self.eeTask.useLocalFrame(conf.ee_task_local_frame)
        self.EE = model_pin.getFrameId(conf.ee_frame_name)
        H_ee_ref = self.robot_tsid.framePosition(formulation.data(), self.EE)
        self.trajEE = tsid.TrajectorySE3Constant("traj-ee", H_ee_ref)
        formulation.addMotionTask(self.eeTask, conf.w_ee, 1, 0.0)
        
        self.tau_max = conf.tau_max_scaling*model_pin.effortLimit
        self.tau_min = -self.tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot_tsid)
        actuationBoundsTask.setBounds(self.tau_min, self.tau_max)
        if(conf.w_torque_bounds>0.0):
            formulation.addActuationTask(actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)
        
        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot_tsid, conf.dt)
        self.v_max = conf.v_max_scaling * model_pin.velocityLimit
        self.v_min = -self.v_max
        jointBoundsTask.setVelocityBounds(self.v_min, self.v_max)
        if(conf.w_joint_bounds>0.0):
            formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        # NO BINDINGS FOR THIS TASK!
        # jointBoundsTask = tsid.TaskJointPosVelAccBounds('task-joint-bounds', robot_tsid, conf.dt)
        # self.q_min = conf.q_min_scaling * model_pin.lowerPositionLimit
        # self.q_max = conf.q_max_scaling * model_pin.upperPositionLimit
        # self.v_max = conf.v_max_scaling * model_pin.velocityLimit
        # jointBoundsTask.setVelocityBounds(self.v_max)
        # jointBoundsTask.setPositionBounds(self.q_min, self.q_max)
        # if(conf.w_joint_bounds>0.0):
        #     formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", robot_pin.q0)
        postureTask.setReference(trajPosture.computeNext())
        
        solver = tsid.SolverHQuadProgFast("qp solver")
        solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)
        
        self.trajPosture = trajPosture
        self.postureTask  = postureTask
        # self.actuationBoundsTask = actuationBoundsTask
        # self.jointBoundsTask = jointBoundsTask
        self.formulation = formulation
        self.solver = solver
        self.q = robot_pin.q0
        self.v = v0

 
        if(viewer):
            self.robot_display = pin.RobotWrapper(robot_pin.model, robot_pin.collision_model, robot_pin.visual_model)
            self.robot_display.q0 = self.robot_display.model.referenceConfigurations['default']

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


        # SETTING ARMATURE IN PYTHON NOT POSSIBLE
        # armature_scalar = 0.0
        # gear_ratios = np.matrix(np.ones(7))
        # rotor_inertias = armature_scalar*np.matrix(np.ones(7))
        # # robot_tsid.model_pin().gear_ratios = gear_ratios 
        # # robot_tsid.model_pin().rotor_inertias = rotor_inertias 
        # print(robot_tsid.model_pin().rotorGearRatio)
        # print(robot_tsid.model_pin().rotorInertia)
        # print(robot_tsid.gear_ratios)
        # print(robot_tsid.rotor_inertias)
        # # robot_tsid.set_gear_ratios(robot_tsid, gear_ratios)
        # # robot_tsid.set_rotor_inertias(robot_tsid, rotor_inertias)
        # # robot_tsid.gear_ratios = gear_ratios
        # # robot_tsid.rotor_inertias = rotor_inertias
        # # robot_tsid.gear_ratios(gear_ratios)
        # # robot_tsid.rotor_inertias(rotor_inertias)
        # # robot_tsid.updateMd()  # <==> croco "armature" but not binded
