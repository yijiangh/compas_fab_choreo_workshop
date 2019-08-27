from __future__ import print_function
import os
import json
import time
import math

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

JOINT_TOPIC_NAME = 'joint_states'

def gripper_srv_call(client, state=0):
    service = roslibpy.Service(client, '/ur_driver/set_io', 'ur_msgs/SetIO')
    request = roslibpy.ServiceRequest({'fun': 1, 'pin': 0, 'state': state})
    service.call(request)
    time.sleep(1.0)

def calc_jt_time(jt1, jt2, max_joint_vel=0.01):
    assert len(jt1) == len(jt2)
    min_time = [abs(val1 - val2) / max_joint_vel for val1, val2 in zip(jt1, jt2)]
    return max(min_time)

def exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt, last_jt_pt=None, handle_success=None, handle_failure=None):
    init_time_cnt = traj_time_cnt
    for i, jt_pt in enumerate(ros_jt_traj.points):
        # reparam speed
        ros_jt_traj.points[i].velocities = [max_jt_vel] * len(jt_pt.values)
        ros_jt_traj.points[i].accelerations = [max_jt_acc] * len(jt_pt.values)

        if not last_jt_pt:
            traj_time_cnt = 0.0
        else:
            traj_time_cnt += calc_jt_time(last_jt_pt.values, jt_pt.values)
        ros_jt_traj.points[i].time_from_start = Duration(traj_time_cnt, 0)

        last_jt_pt = ros_jt_traj.points[i]
        # end reparam speed
        # ros_jt_traj.points[i] = jt_pt

    msg_data = ros_jt_traj.to_data()
    msg_data['header'] = Header().msg
    msg_data['joint_names'] = joint_names
    for i, jt_pt_data in enumerate(msg_data['points']):
        msg_data['points'][i]['positions'] = jt_pt_data['values']

    # cancelable_task = client.follow_joint_trajectory(JointTrajectoryMsg.from_msg(msg_data),
    #         action_name='/follow_joint_trajectory', callback=handle_success, errback=handle_failure)

    return ros_jt_traj.points[-1], traj_time_cnt

class MsgGetter(object):
    def __init__(self):
        self.last_msg = None

    def receive_msg(self, msg):
        self.last_msg = msg

    def get_msg(self):
        return self.last_msg


def main():
    w = threading.Event()
    def handle_success(*args, **kwargs):
       w.set()

    def handle_failure(*args, **kwargs):
       raise Exception('Something went wrong')

    result_save_path = 'C:/Users/harry/Documents/choreo_result/choreo_result.json'
    with open(result_save_path, 'r') as f:
        json_data = json.loads(f.read())

    traj_time_cnt = 0.0
    max_jt_vel = 0.2
    max_jt_acc = 0.1
    last_jt_pt = None

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    # [0.0, -94.94770102010436, 98.0376624092449, -93.01855212389889, 0.0, 0.0]
    # UR=192.168.0.30, Linux=192.168.0.1, Windows=192.168.0.2
    # the following host IP should agree with the Linux machine
    with RosClient(host='192.168.0.120', port=9090) as client:
        client.on_ready(lambda: print('Is ROS connected?', client.is_connected))

        # get current configuration
        listener = roslibpy.Topic(client, JOINT_TOPIC_NAME, 'sensor_msgs/JointState')
        # current_jt_state = JointState()
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

        st_conf = Configuration.from_revolute_values(last_seen_state['position'])
        goal_conf = Configuration.from_revolute_values(json_data[0]['place2pick']['start_configuration']['values'])
        print('goal conf: ', goal_conf)
        goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(1)]*6, group)
        init_traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRTStar')

        print('************\nexecuting init transition')
        print(init_traj.points)
        last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, init_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
            last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

        for seq_id, e_process_data in enumerate(json_data):
            print('************\nexecuting #{} picknplace process'.format(seq_id))

            # open gripper
            gripper_srv_call(client, state=0)

            print('=====\nexecuting #{} place-retreat to pick-approach transition process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['place2pick'])
            print(ros_jt_traj.points)
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

            print('=====\nexecuting #{} pick-approach to pick-grasp process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['pick_approach'])
            print(ros_jt_traj.points)
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

            # close gripper
            gripper_srv_call(client, state=1)

            print('=====\nexecuting #{} pick-grasp to pick-retreat process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['pick_retreat'])
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

            print('=====\nexecuting #{} pick-retreat to place-approach transition process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['pick2place'])
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

            print('=====\nexecuting #{} place-approach to place-grasp process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['place_approach'])
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)

            # open gripper
            gripper_srv_call(client, state=0)

            print('=====\nexecuting #{} place-grasp to place-retreat process'.format(seq_id))
            ros_jt_traj = JointTrajectory.from_data(e_process_data['place_retreat'])
            last_jt_pt, traj_time_cnt = exec_jt_traj(client, joint_names, ros_jt_traj, max_jt_vel, max_jt_acc, traj_time_cnt,
                last_jt_pt=last_jt_pt, handle_success=handle_success, handle_failure=handle_failure)


if __name__ == '__main__':
    main()