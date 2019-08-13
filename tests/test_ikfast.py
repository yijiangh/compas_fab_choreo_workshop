import pytest
import numpy as np

from compas.robots import RobotModel

import compas_fab
from compas_fab.backends.ros.plugins_choreo import get_ik_tool_link_pose, \
 sample_tool_ik, best_sol

from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics

from compas_fab.backends.pybullet import create_pb_robot_from_ros_urdf

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint, \
    create_obj, set_pose, get_sample_fn, violates_limits, joints_from_names, \
    set_joint_positions, remove_debug, get_joint_limits, WorldSaver, \
    LockRenderer, update_state, end_effector_from_body, approach_from_grasp, \
    unit_pose, approximate_as_prism, point_from_pose, multiply, quat_from_euler, \
    approximate_as_cylinder, quat_from_matrix, matrix_from_quat

try:
    import ikfast_ur5
except ImportError as e:
    assert False, '\x1b[6;30;43m' + '{}, please install ikfast_pybind'.format(e) + '\x1b[0m'

# @pytest.mark.wip
def test_ikfast_inverse_kinematics():
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    ik_tool_link_name = robot.get_end_effector_link_name()

    connect(use_gui=False)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)

    max_attempts = 20
    EPS = 1e-3
    sample_fn = get_sample_fn(pb_robot, pb_ik_joints)

    for i in range(max_attempts):
        # randomly sample within joint limits
        conf = sample_fn()
        # sanity joint limit violation check
        assert not violates_limits(pb_robot, pb_ik_joints, conf)

        # ikfast's FK
        fk_fn = ikfast_ur5.get_fk
        ikfast_FK_pb_pose = get_ik_tool_link_pose(fk_fn, pb_robot, ik_joint_names, base_link_name, conf)

        if has_gui():
            print('test round #{}: ground truth conf: {}'.format(i, conf))
            handles = draw_pose(ikfast_FK_pb_pose, length=0.04)
            set_joint_positions(pb_robot, pb_ik_joints, conf)
            wait_for_user()

        # ikfast's IK
        ik_fn = ikfast_ur5.get_ik
        ik_sols = sample_tool_ik(ik_fn, pb_robot, ik_joint_names, base_link_name,
                        ikfast_FK_pb_pose, get_all=True)

        # TODO: UR robot or in general joint w/ domain over 4 pi
        # needs specialized distance function
        q_selected = sample_tool_ik(ik_fn, pb_robot, ik_joint_names, base_link_name,
                        ikfast_FK_pb_pose, nearby_conf=True)
        qsol = best_sol(ik_sols, conf, [1.]*6)
        print('q selected: {}'.format(q_selected))
        print('q best: {}'.format(qsol))

        if has_gui():
            set_joint_positions(pb_robot, pb_ik_joints, qsol)
            wait_for_user()
            for h in handles : remove_debug(h)

        if qsol is None:
            qsol = [999.]*6
        diff = np.sum(np.abs(np.array(qsol) - np.array(conf)))
        if diff > EPS:
            print(np.array(ik_sols))
            print('Best q:{}'.format(qsol))
            print('Actual:{}'.format(np.array(conf)))
            print('Diff:  {}'.format(conf - qsol))
            print('Difdiv:{}'.format((conf - qsol)/np.pi))
            assert False
