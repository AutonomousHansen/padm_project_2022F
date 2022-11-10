from __future__ import print_function

import os
import sys
import argparse
import numpy as np

sys.path.extend('/app/padm-project-2022f/' + d for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, step_simulation
from pybullet_tools.utils import CIRCULAR_LIMITS, PANDA_HAND_URDF, get_custom_limits, set_joint_positions, \
                                 interval_generator, get_link_pose, interpolate_poses, single_collision_with_reporting, \
                                 get_body_info, sample_reachable_base, create_shape, euler_from_quat, create_collision_shape, \
                                 get_mesh_geometry, create_visual_shape, load_pybullet, joint_controller, simulate_controller, create_body, \
                                 get_box_geometry, remove_body, NULL_ID, STATIC_MASS, plan_cartesian_motion

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from world import World
from utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_base_sample_fn(body):
    pass

def get_gripper_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def create_box_at_pose(pose):
    viz_id = create_visual_shape(geometry=get_box_geometry(0.1,0.1,0.1), pose=pose)
    return create_body(collision_id=NULL_ID, visual_id=viz_id, mass=STATIC_MASS)

def link_in_collision(point):
    pass

def build_panda_world():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    return world, tool_link

def gripper_ik_plan(world, tool_link, start_pose=None, end_pose=None):
    sample_fn = get_gripper_sample_fn(world.robot, world.arm_joints)
    print("Going to use IK to go from a sample start state to a goal state\n")
    conf = sample_fn()
    set_joint_positions(world.robot, world.arm_joints, conf)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    if start_pose is None:
        start_pose = get_link_pose(world.robot, tool_link)
    print("Start Pose: {}".format(start_pose))
    if end_pose is None:
        end_pose = multiply(start_pose, Pose(Point(x=0.5)))
    goal_box = create_box_at_pose(end_pose)
    print("End Pose: {}".format(end_pose))

    confs = []
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        val, collision = single_collision_with_reporting(world.robot)
        if conf is None:
            print('Failure!')
            remove_body(goal_box)
            return None
        elif val:
            print(collision)
            print("Collision! {}".format(get_joint_name(world.robot, collision)))
            remove_body(goal_box)
            return None
        confs.append(conf)
        set_joint_positions(world.robot, ik_joints, conf)
    remove_body(goal_box)
    return confs

def baselink_plan(world, tool_link, start_pose=None, end_pose=None):
    if start_pose is None:
        start_pose = get_link_pose(world.robot, world.base_link)
    print("Start Pose: {}".format(start_pose))
    if end_pose is None:
        end_pose = multiply(start_pose, Pose(Point(x=0.5)))
    wps = [start_pose, end_pose]
    return plan_cartesian_motion(world.robot, world.base_link, world.base_link, waypoint_poses=wps)
