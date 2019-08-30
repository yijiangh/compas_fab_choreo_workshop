from __future__ import print_function
import os
import json
import time
import math

from compas.datastructures import Mesh
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.backends.ros import JointState
from compas_fab.backends.ros import Header
from compas_fab.backends.ros import JointTrajectory as JointTrajectoryMsg

from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots.configuration import Configuration
from compas_fab.robots.time_ import Duration
from compas_fab.robots import JointTrajectoryPoint, JointTrajectory

import roslibpy
import threading

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
# misc: for plotting and measuring time
import matplotlib.pyplot as plt

REAL_EXECUTION = True
JOINT_TOPIC_NAME = 'joint_states'

def gripper_srv_call(client, state=0, exe=REAL_EXECUTION):
    """[summary]

    Parameters
    ----------
    client : [type]
        [description]
    state : int, optional
        [description], by default 0
    exe : [type], optional
        [description], by default REAL_EXECUTION
    """
    if not exe:
        return
    service = roslibpy.Service(client, '/ur_driver/set_io', 'ur_msgs/SetIO')
    request = roslibpy.ServiceRequest({'fun': 1, 'pin': 0, 'state': state})
    service.call(request)
    time.sleep(1.0)

def calc_jt_time(jt1, jt2, max_joint_vel=0.01):
    """[summary]

    Parameters
    ----------
    jt1 : list of float
        start joint values
    jt2 : list of float
        end joint values
    max_joint_vel : float, optional
        maximal joint velocity, by default 0.01

    Returns
    -------
    float
        time for executing from jt1 to jt2
    """
    assert len(jt1) == len(jt2)
    min_time = [abs(val1 - val2) / max_joint_vel for val1, val2 in zip(jt1, jt2)]
    return max(min_time)

def traj_reparam(compas_fab_jt_traj, max_jt_vel, max_jt_acc,
                 traj_time_cnt=0, ts_sample_num=100, grid_num=200, inspect_sol=False):
    dof = len(compas_fab_jt_traj.points[0].values)

    # Create path
    way_jt_pts = [jt_pt.values for jt_pt in compas_fab_jt_traj.points]
    path = ta.SplineInterpolator(np.linspace(0, 1, len(compas_fab_jt_traj.points)), way_jt_pts)

    # Create velocity bounds, then velocity constraint object
    vlim_ = np.ones(6) * max_jt_vel
    vlim = np.vstack((-vlim_, vlim_)).T
    # Create acceleration bounds, then acceleration constraint object
    alim_ = np.ones(6) * max_jt_acc
    alim = np.vstack((-alim_, alim_)).T
    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(
        alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

    # Setup a parametrization instance, then retime
    gridpoints = np.linspace(0, path.duration, grid_num)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, solver_wrapper='seidel')
    jnt_traj, aux_traj, int_data = instance.compute_trajectory(0, 0, return_data=True)

    if inspect_sol:
        ts_sample = np.linspace(0, jnt_traj.get_duration(), 100)
        qdds_sample = jnt_traj.evaldd(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)

        fig, axs = plt.subplots(1, 2, sharex=True, figsize=[12, 4])
        for i in range(6):
            axs[0].plot(ts_sample, qdds_sample[:, i], label="J{:d}".format(i + 1))
            axs[1].plot(ts_sample, qds_sample[:, i], label="J{:d}".format(i + 1))
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Joint acceleration (rad/s^2)")
        axs[0].legend()
        axs[1].legend()
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Joint velocity (rad/s)")
        plt.show()

    time_list = np.linspace(0, jnt_traj.get_duration(), ts_sample_num)
    jt_list = jnt_traj.eval(time_list)
    vel_list = jnt_traj.evald(time_list)
    acc_list = jnt_traj.evaldd(time_list)

    reparm_traj_pts = []
    for i in range(ts_sample_num):
        new_jt_pt = JointTrajectoryPoint(values=jt_list[i].tolist(), types=[0] * 6)
        new_jt_pt.velocities = vel_list[i].tolist()
        new_jt_pt.accelerations = acc_list[i].tolist()
        new_jt_pt.time_from_start = Duration(traj_time_cnt, 0)
        traj_time_cnt += time_list[i]
        reparm_traj_pts.append(new_jt_pt)
        if i == 0 and np.array_equal(time_list, np.zeros(ts_sample_num)):
            break
    reparam_traj = JointTrajectory(trajectory_points=reparm_traj_pts, start_configuration=reparm_traj_pts[0])
    return reparam_traj

def exec_jt_traj(client, joint_names, compas_fab_jt_traj, real_exe=REAL_EXECUTION):
    """reparametrize a JointTrajectry using given maximal joint velocity/acceleration, the execute
    the trajectory with ros follow_joint_trajectory action call.

    Note: for now, this is UR5 specific! (mainly the action topic name)

    Parameters
    ----------
    client : compas_fab ros client
    joint_names : str
        joint names of the robot
    compas_fab_jt_traj : compas_fab.backends.ros.JointTrajectory
        Joint Trajectory to be executed
    handle_success : , optional
        action success callback handle, by default None
    handle_failure : [type], optional
        action failure callback handle, by default None

    Returns
    -------
    """
    w = threading.Event()
    def handle_success(*args, **kwargs):
        w.set()
    def handle_failure(*args, **kwargs):
        raise Exception('Something went wrong')

    msg_data = compas_fab_jt_traj.to_data()
    msg_data['header'] = Header().msg
    msg_data['joint_names'] = joint_names
    for i, jt_pt_data in enumerate(msg_data['points']):
        msg_data['points'][i]['positions'] = jt_pt_data['values']

    if real_exe:
        client.follow_joint_trajectory(JointTrajectoryMsg.from_msg(msg_data),
                        action_name='/follow_joint_trajectory', callback=handle_success, errback=handle_failure)
        w.wait()
    else:
        return None


class MsgGetter(object):
    """A dumb message get class
    """
    def __init__(self):
        self.last_msg = None

    def receive_msg(self, msg):
        self.last_msg = msg

    def get_msg(self):
        return self.last_msg


def main():
    choreo_problem_instance_dir = compas_fab.get('choreo_instances')
    result_save_path = os.path.join(choreo_problem_instance_dir, 'results', 'choreo_result.json')
    with open(result_save_path, 'r') as f:
        json_data = json.loads(f.read())

    traj_time_cnt = 0.0
    max_jt_vel = 0.2
    max_jt_acc = 0.1
    last_jt_pt = None

    # pybullet traj preview settings
    pybullet_preview = True
    PB_VIZ_CART_TIME_STEP = 0.05
    PB_VIZ_TRANS_TIME_STEP = 0.025
    PB_VIZ_PER_CONF_SIM = False

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                'pychoreo_workshop_gripper/collision/victor_gripper_jaw03.obj')

    # [0.0, -94.94770102010436, 98.0376624092449, -93.01855212389889, 0.0, 0.0]
    # UR=192.168.0.30, Linux=192.168.0.1, Windows=192.168.0.2
    # the following host IP should agree with the Linux machine
    host_ip = '192.168.0.1' if REAL_EXECUTION else 'localhost'
    with RosClient(host=host_ip, port=9090) as client:
        client.on_ready(lambda: print('Is ROS connected?', client.is_connected))

        # get current configuration
        listener = roslibpy.Topic(client, JOINT_TOPIC_NAME, 'sensor_msgs/JointState')
        msg_getter = MsgGetter()
        listener.subscribe(msg_getter.receive_msg)
        time.sleep(2)
        last_seen_state = msg_getter.get_msg()
        print('current jt state: {}'.format(last_seen_state['position']))

        model = RobotModel.from_urdf_file(urdf_filename)
        semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
        robot = RobotClass(model, semantics=semantics, client=client)
        group = robot.main_group_name
        joint_names = robot.get_configurable_joint_names()
        # base_link_name = robot.get_base_link_name()
        ee_link_name = robot.get_end_effector_link_name()

        if pybullet_preview:
            from conrob_pybullet import connect
            from compas_fab.backends.ros.plugins_choreo import display_trajectory_chunk
            from compas_fab.backends.pybullet import attach_end_effector_geometry, \
            convert_mesh_to_pybullet_body, create_pb_robot_from_ros_urdf

            connect(use_gui=True)
            pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                                    ee_link_name=ee_link_name)
            ee_meshes = [Mesh.from_obj(ee_filename)]
            ee_attachs = attach_end_effector_geometry(ee_meshes, pb_robot, ee_link_name)

        st_conf = Configuration.from_revolute_values(last_seen_state['position'])
        goal_conf = Configuration.from_revolute_values(json_data[0]['place2pick']['start_configuration']['values'])
        goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(1)]*6, group)
        init_traj_raw = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRTStar')
        init_traj = traj_reparam(init_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)

        if pybullet_preview:
            display_trajectory_chunk(pb_robot, joint_names,
                                     init_traj.to_data(), \
                                     ee_attachs=ee_attachs, grasped_attach=[],
                                     time_step=PB_VIZ_TRANS_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)

        print('************\nexecuting init transition')
        exec_jt_traj(client, joint_names, init_traj)

        print('executed?')
        input()

        for seq_id, e_process_data in enumerate(json_data):
            print('************\nexecuting #{} picknplace process'.format(seq_id))

            # open gripper
            gripper_srv_call(client, state=0)

            print('=====\nexecuting #{} place-retreat to pick-approach transition process'.format(seq_id))
            traj_data = e_process_data['place2pick']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)

            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_TRANS_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)
            exec_jt_traj(client, joint_names, ros_jt_traj)

            print('=====\nexecuting #{} pick-approach to pick-grasp process'.format(seq_id))
            traj_data = e_process_data['pick_approach']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_CART_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)

            exec_jt_traj(client, joint_names, ros_jt_traj)

            print('executed?')
            input()

            # close gripper
            gripper_srv_call(client, state=1)

            print('=====\nexecuting #{} pick-grasp to pick-retreat process'.format(seq_id))
            traj_data = e_process_data['pick_retreat']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_CART_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)

            exec_jt_traj(client, joint_names, ros_jt_traj)

            print('executed?')
            input()

            print('=====\nexecuting #{} pick-retreat to place-approach transition process'.format(seq_id))
            traj_data = e_process_data['pick2place']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_TRANS_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)

            exec_jt_traj(client, joint_names, ros_jt_traj)

            print('executed?')
            input()

            print('=====\nexecuting #{} place-approach to place-grasp process'.format(seq_id))
            traj_data = e_process_data['place_approach']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_CART_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)

            exec_jt_traj(client, joint_names, ros_jt_traj)

            # open gripper
            gripper_srv_call(client, state=0)

            print('executed?')
            input()

            print('=====\nexecuting #{} place-grasp to place-retreat process'.format(seq_id))
            traj_data = e_process_data['place_retreat']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_CART_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)
            exec_jt_traj(client, joint_names, ros_jt_traj)

            print('executed?')
            input()

        if 'return2idle' in json_data[-1]:
            print('=====\nexecuting #{} return-to-idle transition process'.format(seq_id))
            traj_data = e_process_data['return2idle']
            ros_jt_traj_raw = JointTrajectory.from_data(traj_data)
            ros_jt_traj = traj_reparam(ros_jt_traj_raw, max_jt_vel, max_jt_acc, inspect_sol=False)
            if pybullet_preview:
                display_trajectory_chunk(pb_robot, joint_names,
                                        ros_jt_traj.to_data(), \
                                        ee_attachs=ee_attachs, grasped_attach=[],
                                        time_step=PB_VIZ_TRANS_TIME_STEP, step_sim=True, per_conf_step=PB_VIZ_PER_CONF_SIM)
            exec_jt_traj(client, joint_names, ros_jt_traj)


if __name__ == '__main__':
    main()
