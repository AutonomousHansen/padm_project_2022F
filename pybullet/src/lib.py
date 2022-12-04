from __future__ import print_function

import os
import sys
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString
import copy
import argparse
import numpy as np
import math
from time import sleep

sys.path.extend('/app/padm-project-2022f/' + d for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, step_simulation
from pybullet_tools.utils import CIRCULAR_LIMITS, PANDA_HAND_URDF, get_custom_limits, set_joint_positions, \
                                 interval_generator, get_link_pose, interpolate_poses, single_collision_with_reporting, single_collision, \
                                 get_body_info, get_body_name, sample_reachable_base, create_shape, euler_from_quat, create_collision_shape, \
                                 get_mesh_geometry, create_visual_shape, load_pybullet, joint_controller, simulate_controller, create_body, \
                                 get_box_geometry, remove_body, NULL_ID, STATIC_MASS, plan_cartesian_motion, quat_from_euler, is_pose_close, \
                                 set_base_values, pose_from_base_values, draw_aabb, AABB, sample_placement_on_aabb, aabb2d_from_aabb, get_aabb_center, \
                                 get_aabb_extent, unit_pose, base_values_from_pose, aabb_contains_aabb, get_configuration, set_configuration, get_movable_joints,\
                                 get_joint_positions, body_from_end_effector, end_effector_from_body, Attachment, create_attachment, approach_from_grasp, invert, sub_inverse_kinematics, \
                                 get_max_limit, get_min_limit, get_max_limits, get_min_limits, get_joint_axis, multiply, draw_pose, get_local_link_pose, get_relative_pose, get_link_ancestors, get_link_children   

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from world import World
from utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR,  APPROACH_DISTANCE, point_from_pose, translate_linearly, \
    get_grasps, set_tool_pose, get_tool_from_root, get_tool_link, iterate_approach_path, RelPose, pose_from_attachment

UNIT_POSE2D = (0., 0., 0.)
STEER_DISTANCE = 0.15
Z_OFFSET = -1.2301278283276562
FLOOR_PLANE = -1.4760000000000004
KITCHEN_WIDTH = 1.052
KITCHEN_LENGTH = 4.1
ROBOT_WIDTH = 2.435
ROBOT_LENGTH = 0.634
BASE_WIDTH = 0.82
BASE_LENGTH = 0.634
KITCHEN_ID = 0
FLOOR_ID = 1
BASE_BUFFER = 0.2
# DEF_SUGAR_POSE = ((-1.26, 0.70, -1.2301278283276562), (0.0, 0.0, 0.0, 1.0))
DEF_SUGAR_POSE = ((-0.70, -0.2, -1.2301278283276562), (0.0, 0.0, 0.3826834559440613, 0.9238795042037964))
DEF_MEAT_POSE = ((-1.2, 0.85, -1.2301278283276562), (0.0, 0.0, 0.0, 1.0))
access_poses = {
    'sugar_box0' : DEF_SUGAR_POSE,
    'potted_meat_can1' : DEF_MEAT_POSE
}
extended_arms = [0.0, 1.5, 0.0, 0.0, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405]
grip_sugar = ((-0.0934628039598465, 0.5329028606414795, -0.39798504114151), (0.3826833963394165, 0.9238795638084412, -5.657131222642009e-17, -2.3432597385528782e-17))
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

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    print(lower_limits)
    print(upper_limits)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def create_box_at_pose(pose):
    viz_id = create_visual_shape(geometry=get_box_geometry(0.1,0.1,0.1), pose=pose)
    return create_body(collision_id=NULL_ID, visual_id=viz_id, mass=STATIC_MASS)

def link_in_collision(point):
    pass

def build_panda_world(gui=True):
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=gui)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    world._update_initial()
    tool_link = link_from_name(world.robot, 'right_gripper')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    return world, tool_link

def panda_arm_sample(world):
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    conf = sample_fn()
    print(conf)
    set_joint_positions(world.robot, world.arm_joints, conf)
    return conf
    # ik_joiget_posents = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    # start_pose = get_link_pose(world.robot, tool_link)

def panda_base_sample(world):
    sample_fn = get_sample_fn(world.robot, world.base_joints)
    conf = sample_fn()
    set_joint_positions(world.robot, world.base_joints, conf)
    return get_pose(world.robot)
    # xyz, _ = get_pose(world.robot)
    # x, y, yaw = sample_reachable_base(world.robot, xyz)
    # quat = quat_from_euler((0,0,yaw))
    # return ((x, y, 0 + Z_OFFSET), quat)

def nearest_base_pose(world, proposed_pose, past_poses):
    # TODO: Implement as a KDTree for speed
    # set_joint_positions(world.robot, world.base_joints, conf)
    # proposed_pose = get_pose(world.robot)
    # kd_tree = KDTree(past_poses)
    # d, i = kd_tree.quert([proposed_pose[0], proposed_pose[1]], 1)
    # return past_poses[i]
    min_dist = float('inf')
    min_pose = None
    for past_pose in past_poses:
        print("PP: {}".format(past_pose))
        print("PPBV: {}".format(base_values_from_pose(past_pose)))
        # Doing position-only distance on first pass
        # TODO: Incorporate orientation distance
        # set_joint_positions(world.robot, world.base_joints, pose)
        # past_pose = get_pose(world.robot)
        pose_dist = math.dist((past_pose[0][0], past_pose[0][1], past_pose[0][2]), \
                    (proposed_pose[0][0],proposed_pose[0][1],proposed_pose[0][2]))
        if pose_dist < min_dist:
            min_dist = pose_dist
            min_pose = past_pose
    print("Min Pose: {}".format(min_pose))
    return min_pose

def panda_gripper_closest_ik_conf(world, pose):
    conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, world.tool_link, pose, max_time=0.05), None)
    return conf

def base_to_node_collisionfree(world, tool_link, pose1, pose2):
    # set_joint_positions(world.robot, world.base_joints, conf1)
    # pose1 = get_pose(world.robot)
    # set_joint_positions(world.robot, world.base_joints, conf2)
    # pose2 = get_pose(world.robot)
    print(pose1)
    set_pose(world.robot, pose1)
    for pose in interpolate_poses(pose1, pose2, pos_step_size=0.01):
        set_pose(world.robot, pose)
        # conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        # wait_for_user()
        # for _, obstacle in world.static_obstacles:
        #     obs = next(iter(obstacle))
        #     if obs != world.gripper and obs != 2:
        #         if pairwise_collision(world.robot, obs):
        #             set_pose(world.robot, pose1)
        #             print("Collided with static object {}! ({})".format(get_body_info(obs), obs))
        #             wait_for_user()
        #             return False

        if pairwise_collision(world.robot, world.kitchen):
            set_pose(world.robot, pose1)
            print(pose1)
            print("Collided with  kitchen!")
            wait_for_user()
            return False

        # if conf is None:
        #     cur_pose = get_pose(world.robot)
        #     if not is_pose_close(cur_pose, pose2):
        #         print("Failure!")
        #         wait_for_user()
        #         set_pose(world.robot, pose1)
        #         return False
        #     else:
        #         break
        # else:
        #     set_pose(world.robot, pose)
        #     print("Pass!")
    print("Nodepath is collision free")
    return True

def node_collisionfree(world, pose):
    set_pose(world.robot, pose)
    if single_collision(world.robot):
        return False
    print("Node is collision free")
    return True

def project_pose(world, pose1, pose2):
    # set_joint_positions(world.robot, world.base_joints, conf1)
    # pose1 = get_pose(conf1)
    # set_joint_positions(world.robot, world.base_joints, conf2)
    # pose2 = get_pose(conf2)
    point1 = Point(pose1[0][0], pose1[0][1], pose1[0][2])
    point2 = Point(pose2[0][0], pose2[0][1], pose2[0][2])
    line = LineString([point2, point1])
    pose_point = line.interpolate(STEER_DISTANCE)
    # Defaulting to the yaw from the goal pose
    pose = ((pose_point.x, pose_point.y, pose_point.z), pose2[1])
    return pose

def set_pose_to_goal_bq(world):
    goal = world.goal_bq
    # goal_quat = quat_from_euler((0,0,goal.values[2]))
    # goal_pose = ((goal.values[0], goal.values[1], 0 + Z_OFFSET), goal_quat)   
    set_base_values(world.robot, goal.values) 

def pose_w_z_offset(pose):
    list_pose = [list(x) for x in pose]
    list_pose[0][2] = Z_OFFSET
    return tuple([tuple(x) for x in list_pose])

def highlight_collision(pose1, pose2=None):
    if not pose2:
        viz = create_visual_shape(geometry=get_box_geometry(0.05,0.05,0.05), pose=pose1)
        bod = create_body(visual_id=viz)
        wait_for_user()
        remove_body(bod)
        return None
    elif pose2:
        viz1 = create_visual_shape(geometry=get_box_geometry(0.05,0.05,0.05), pose=pose1)
        viz2 = create_visual_shape(geometry=get_box_geometry(0.05,0.05,0.05), pose=pose2)
        bod1 = create_body(visual_id=viz1)
        bod2 = create_body(visual_id=viz2)
        wait_for_user()
        remove_body(bod1)
        remove_body(bod2)
        return None

def name_static_objects(list, w):
    ids = [z for x, y in list for z in y]
    names = []
    for id in ids:
        try:
            names.append(get_body_info(id))
        except:
            try:
                names.append(get_joint_name(w.robot, id))
            except:
                pass
                try:
                    names.append(get_joint_name(0, id))
                except:
                    pass
                    try:
                        names.append("Invalid{}".format(id))
                    except:
                        continue
    return names

def name_objects(list, w):
    names = []
    for id in list:
        try:
            names.append(get_body_info(id))
        except:
            try:
                names.append(get_joint_name(w.robot, id))
            except:
                pass
                try:
                    names.append(get_joint_name(0, id))
                except:
                    pass
                    try:
                        names.append("Invalid{}".format(id))
                    except:
                        continue
    return names

def obstacles_of_interest(w):
    default_s_ids = set([z for x, y in w.static_obstacles for z in y])
    robot_ids = [w.arm_joints, w.base_link, w.gripper_joints, w.gripper_link, w.gripper, w.franka_link, w.tool_link]
    robot_set = set()
    for ids in robot_ids:
        if type(ids) is int:
            ids = [ids]
        robot_set.update(set(ids))
    
    s_ids = default_s_ids.difference(robot_set)
    return s_ids

def play_poses(world, poses):
    for pose in poses:
        set_pose(world.robot, pose)
        sleep(0.1)

def aabb_from_objpose(goal_pose, buff=BASE_BUFFER):
    goal_position, goal_quat = goal_pose
    goal_aabb = AABB(lower=[goal_position[0], goal_position[1] - ROBOT_LENGTH, FLOOR_PLANE],
                    upper=[goal_position[0] + BASE_WIDTH + KITCHEN_WIDTH,  goal_position[1] + ROBOT_LENGTH, FLOOR_PLANE])
    return goal_aabb

def limits_from_objpose(goal_pose, buff=BASE_BUFFER):
    goal_position, goal_quat = goal_pose
    goal_limits = {0: (goal_position[0], goal_position[0] + buff*1.5),
                   1: (goal_position[1] - buff, goal_position[1] + buff),
                   2: (-np.pi, np.pi)}
    return goal_limits
    
# Taken from ss-pybullet sample_placement_on_aabb, to extend interface
def sample_base_placement_on_aabb(world, top_body, bottom_aabb, top_pose=unit_pose(),
                             percent=0.8, max_attempts=50, epsilon=1e-3):
    # TODO: transform into the coordinate system of the bottom
    # TODO: maybe I should instead just require that already in correct frame
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = Euler(yaw=theta)
        set_pose(top_body, multiply(Pose(euler=rotation), top_pose))
        center = get_aabb_center(world.get_base_aabb())
        extent = get_aabb_extent(world.get_base_aabb())
        lower = (np.array(bottom_aabb[0]) + percent*extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent*extent/2)[:2]
        if np.less(upper, lower).any():
            continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2] + epsilon
        point = np.array([x, y, z]) + (get_point(top_body) - center)
        pose = multiply(Pose(point, rotation), top_pose)
        set_pose(top_body, pose)
        return pose
    return None

def base_rrt(world, tool_link, goal):
    # goal_quat = quat_from_euler((0,0,goal.values[2]))
    # goal_pose = ((goal.values[0], goal.values[1], 0 + Z_OFFSET), goal_quat)
    sample_fn = get_sample_fn(world.robot, world.base_joints, custom_limits=world.custom_limits)
    sample_max = 5000
    goal_pose = get_pose(world.get_body(goal))
    goal_aabb = aabb_from_objpose(goal_pose)
    draw_aabb(goal_aabb)
    # goal_limits = limits_from_objpose(goal_pose)
    # goal_sample = get_sample_fn(world.robot, world.base_joints, custom_limits=goal_limits)
    init_pose = get_pose(world.robot)
    tree_nodes = [init_pose]
    tree_edges = []
    for i in range(sample_max):
        if i % 5 == 0:
            # rand_base_pose = pose_w_z_offset(pose_from_base_values(goal_sample()))
            # set_pose(world.robot, rand_base_pose)
            # print(rand_base_pose)
            # wait_for_user()
            rand_base_pose = access_poses[goal]
            # set_pose(world.robot, rand_base_pose)
            set_base_values(world.robot, base_values_from_pose(rand_base_pose))
        else:
            rand_base_pose = pose_w_z_offset(pose_from_base_values(sample_fn()))
        # print("Rand Node:{}".format(rand_base_pose))
        if node_collisionfree(world, rand_base_pose):
            nearest_pose = nearest_base_pose(world, rand_base_pose, tree_nodes)
            # print("Nearest Node: {}".format(nearest_pose))
            # if i % 5 == 0:
            #     wait_for_user()
            pose = project_pose(world, rand_base_pose, nearest_pose)
            if base_to_node_collisionfree(world, tool_link, pose, nearest_pose):
                # print("New Node: {}".format(pose))
                set_base_values(world.robot, base_values_from_pose(pose))
                # if i % 5 == 0:
                #     wait_for_user()
                # set_pose(world.robot, pose)
                tree_nodes.append(pose)
                tree_edges.append((nearest_pose, pose))
                position, _ = pose
                goal_position, _ = goal_pose
                x1,y1,yaw1 = base_values_from_pose(pose)
                x2,y2,yaw2 = base_values_from_pose(goal_pose)
                pos_dist = math.dist([x1, y1], [x2, y2])
                # if i % 5 == 0:
                #     wait_for_user()
                # if pos_dist < 0.05:
                #     print("NEAR GOAL")
                #     return path_to_goal(tree_nodes[0], tree_edges)
                # elif aabb_contains_aabb(world.get_base_aabb(), goal_aabb):
                #     print("NEAR GOAL")
                #     return path_to_goal(tree_nodes[0], tree_edges)
                # # wait_for_user()
                if is_pose_close(pose, access_poses[goal]):
                    print("NEAR GOAL")
                    # wait_for_user()
                    return path_to_goal(tree_nodes[0], tree_edges)
        print(len(tree_nodes))
    return None, tree_nodes

def arm_rrt(world, goal):
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    sample_max = 5000
    init_conf = get_joint_positions(world.robot, world.arm_joints)
    tree_nodes = [init_conf]
    tree_edges = []
    for i in range(sample_max):
        new_conf = sample_fn()
        set_joint_positions(world.robot, world.arm_joints, new_conf)
        wait_for_user()

def path_to_goal(root_node, tree_edges):
    path = [tree_edges[-1][1], tree_edges[-1][0]]
    target_node = tree_edges[-1][0]
    while target_node != root_node:
        for parent, child in tree_edges:
            if child == target_node:
                path.append(parent)
                target_node = parent
    path.reverse()
    return path

def base_rrt_and_play(goal='sugar_box0'):
    w,t=build_panda_world()
    poses=base_rrt(w,t,goal)
    play_poses(w,poses)
    # arm_rrt(w, goal)
    return w

def arm_at_pose(world, target_pose):
    return plan_cartesian_motion(world.robot, world.arm_joints[0], world.tool_link, [target_pose])

def arm_at_pose_ik(world, target_pose):
    conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, world.tool_link, target_pose, max_time=0.05), None)
    return conf

def test_arm_at_pose(goal='sugar_box0', w=None):
    if not w:
        w,t=build_panda_world()
    set_pose(w.robot, access_poses[goal])
    w.open_gripper()
    tool_link_pose = get_link_pose(w.robot, w.tool_link)
    anc = get_link_ancestors(w.robot, w.tool_link)
    grasps = get_grasps(w, goal)
    obj_pose = get_pose(w.get_body(goal))
    grasp = next(grasps, None)
    draw_pose(parent=w.robot, parent_link=anc[-1], pose=tool_link_pose)
    init_config = get_configuration(w.robot)
    while not grasp is None:
        goal_pose = multiply(obj_pose, invert(grasp.pregrasp_pose))
        draw_pose(parent=w.robot, parent_link=anc[-1], pose=goal_pose)
        for pose in interpolate_poses(tool_link_pose, goal_pose, pos_step_size=0.01):
            conf = arm_at_pose(w,pose)
            if conf:
                set_configuration(w.robot, conf[0])
                sleep(0.1)
            else:
                new_tool_link_pose = get_link_pose(w.robot, w.tool_link)
                if is_pose_close(new_tool_link_pose, goal_pose):
                    print("Pose close!")
                    return w, get_configuration(w.robot)
                    
            # confs2 = arm_at_pose_ik(w,pose)
            # if confs2:
            #     print("Success2")
            #     return w, (confs2, goal_pose, grasp)
        grasp = next(grasps, None)
    return w, (conf, goal_pose, grasp)