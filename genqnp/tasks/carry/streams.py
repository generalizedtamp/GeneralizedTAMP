BASE_CONSTANT = 1
BASE_VELOCITY = 0.5
Z_EPSILON = 2.5e-3
SELF_COLLISIONS = False

import math
import os
import random
import re
from itertools import islice
from collections import namedtuple
from itertools import combinations
from utils import CloseAttach, CloseDetach, RelPose, QPose, create_bucket
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import Conf, Attach, Detach, Clean, \
    Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, APPROACH_DISTANCE, Grasp, GRASP_LENGTH, \
    iterate_approach_path, create_trajectory, Commands, Trajectory
import numpy as np
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_never_collisions import NEVER_COLLISIONS
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf, GET_GRASPS, \
    TOP_HOLDING_LEFT_ARM, MAX_GRASP_WIDTH, PR2_TOOL_FRAMES, learned_pose_generator, get_gripper_link, GRASP_LENGTH,\
    is_drake_pr2, compute_grasp_width

from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import multiply, get_link_pose, joint_from_name, \
    set_joint_position, joints_from_names, \
    set_joint_positions, get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, \
    set_base_values, \
    get_pose, euler_from_quat, link_from_name, has_link, point_from_pose, invert, Pose, \
    unit_pose, joints_from_names, PoseSaver, get_aabb, get_joint_limits, get_joints, \
    ConfSaver, get_bodies, create_mesh, remove_body, single_collision, unit_from_theta, angle_between, violates_limit, \
    violates_limits, add_line, get_body_name, get_num_joints, approximate_as_cylinder, \
    approximate_as_prism, unit_quat, unit_point, clip, get_joint_info, tform_point, get_yaw, \
    get_pitch, wait_for_user, quat_angle_between, sample_placement, angle_between, quat_from_pose, compute_jacobian, \
    movable_from_joints, quat_from_axis_angle, LockRenderer, Euler, get_links, get_link_name, \
    draw_point, draw_pose, get_extend_fn, get_moving_links, link_pairs_collision, draw_point, get_link_subtree, \
    clone_body, get_all_links, set_color, pairwise_collision, tform_point, create_attachment, BASE_LINK, body_collision, is_placement, \
    get_custom_limits, all_between, sub_inverse_kinematics, plan_direct_joint_motion, plan_joint_motion, \
    BodySaver, CIRCULAR_LIMITS, get_center_extent, get_point, get_unit_vector
from GeneralizedTAMP.pybullet_planning.pybullet_tools.ikfast.pr2.ik import is_ik_compiled, pr2_inverse_kinematics


TOOL_POSE = Pose(euler=Euler(pitch=np.pi / 2))  # l_gripper_tool_frame (+x out of gripper arm)

def get_motion_gen(problem, custom_limits={}, collisions=True, teleport=False):
    # TODO: include fluents
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    def fn(bq1, bq2, fluents=[]):
        saver.restore()
        bq1.assign()
        if teleport:
            path = [bq1, bq2]
        elif is_drake_pr2(robot):
            raw_path = plan_joint_motion(robot, bq2.joints, bq2.values, attachments=[],
                                         obstacles=obstacles, custom_limits=custom_limits, self_collisions=SELF_COLLISIONS,
                                         restarts=4, iterations=50, smooth=50)
            if raw_path is None:
                #set_renderer(True)
                #for bq in [bq1, bq2]:
                #    bq.assign()
                #    wait_if_gui()
                return None
            path = [Conf(robot, bq2.joints, q) for q in raw_path]
        else:
            goal_conf = base_values_from_pose(bq2.value)
            raw_path = plan_base_motion(robot, goal_conf, BASE_LIMITS, obstacles=obstacles)
            if raw_path is None:
                return None
            path = [Pose(robot, pose_from_base_values(q, bq1.value)) for q in raw_path]
        bt = Trajectory(path)
        cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
        return (cmd,)
    return fn

def get_top_tray_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                        max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)

    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    # translate_center = 
    grasps = []
    # for i in range(1 + under):
    #     rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
    #     grasps += [multiply(tool_pose, translate_z, rotate_z,
    #                         reflect_z, translate_center, body_pose)]
    # for i in range(1 + under):
    rotate_z = Pose(euler=[0, 0, 0])
    grasps += [multiply(tool_pose, translate_z, rotate_z,
                        Pose(euler=[0, math.pi, math.pi / 2.0]),
                        Pose(point=point_from_pose(body_pose) - center + [w / 2.0, 0, 0]), body_pose)]
    grasps += [multiply(tool_pose, translate_z, rotate_z,
                        Pose(euler=[0, math.pi, math.pi / 2.0]),
                        Pose(point=point_from_pose(body_pose) - center + [-w / 2.0, 0, 0]), body_pose)]
    grasps += [multiply(tool_pose, translate_z, rotate_z,
                        Pose(euler=[0, math.pi, 0]), Pose(point=point_from_pose(body_pose) - center + [0, w / 2.0, 0]),
                        body_pose)]
    grasps += [multiply(tool_pose, translate_z, rotate_z,
                        Pose(euler=[0, math.pi, 0]), Pose(point=point_from_pose(body_pose) - center + [0, -w / 2.0, 0]),
                        body_pose)]
    return grasps


def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose) - center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    return grasps


def get_ir_sampler(problem, custom_limits={}, max_attempts=25, collisions=True, learned=True, semantic_surface=None):
    robot = problem.robot
    arm = problem.arms[0]
    obstacles = problem.fixed if collisions else []
    gripper = problem.get_gripper()

    def gen_fn(obj, pose, grasp):
        pose.assign()
        approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        print(approach_obstacles)
        for _ in iterate_approach_path(robot, arm, gripper, pose, grasp, body=obj):
            if any(pairwise_collision(gripper, b) or pairwise_collision(obj, b) for b in approach_obstacles):
                print(":(")
                return
        gripper_pose = multiply(pose.value, invert(grasp.value))  # w_f_g = w_f_o * (g_f_o)^-1
        default_conf = arm_conf(arm, grasp.carry)
        arm_joints = get_arm_joints(robot, arm)
        base_joints = get_group_joints(robot, 'base')
        if learned:
            base_generator = learned_pose_generator(robot, gripper_pose, arm=arm, grasp_type=grasp.grasp_type)
        else:
            base_generator = uniform_pose_generator(robot, gripper_pose)
        lower_limits, upper_limits = get_custom_limits(robot, base_joints, custom_limits)
        while True:
            count = 0
            for base_conf in islice(base_generator, max_attempts):
                count += 1
                if not all_between(lower_limits, base_conf, upper_limits):
                    continue
                bq = Conf(robot, base_joints, base_conf)
                pose.assign()
                bq.assign()
                set_joint_positions(robot, arm_joints, default_conf)
                if any(pairwise_collision(robot, b) for b in obstacles + [obj]):
                    continue
                # print('IR attempts:', count)
                yield (bq,)
                break
            else:
                yield None

    return gen_fn


def get_ik_fn(problem, custom_limits={}, collisions=True, teleport=False):
    robot = problem.robot
    arm = problem.arms[0]
    obstacles = problem.fixed if collisions else []
    if is_ik_compiled():
        print('Using ikfast for inverse kinematics')
    else:
        print('Using pybullet for inverse kinematics')

    def fn(obj, pose, grasp, base_conf):
        approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        gripper_pose = multiply(pose.value, invert(grasp.value))  # w_f_g = w_f_o * (g_f_o)^-1
        # approach_pose = multiply(grasp.approach, gripper_pose)
        approach_pose = multiply(pose.value, invert(grasp.approach))
        arm_link = get_gripper_link(robot, arm)
        arm_joints = get_arm_joints(robot, arm)

        default_conf = arm_conf(arm, grasp.carry)
        # sample_fn = get_sample_fn(robot, arm_joints)
        pose.assign()
        base_conf.assign()
        open_arm(robot, arm)
        set_joint_positions(robot, arm_joints, default_conf)  # default_conf | sample_fn()
        grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose,
                                            custom_limits=custom_limits)  # , upper_limits=USE_CURRENT)
        # nearby_conf=USE_CURRENT) # upper_limits=USE_CURRENT,
        if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles):  # [obj]
            # print('Grasp IK failure', grasp_conf)
            # if grasp_conf is not None:
            #    print(grasp_conf)
            #    #wait_for_user()
            return None
        # approach_conf = pr2_inverse_kinematics(robot, arm, approach_pose, custom_limits=custom_limits,
        #                                       upper_limits=USE_CURRENT, nearby_conf=USE_CURRENT)
        approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose,
                                               custom_limits=custom_limits)
        if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles + [obj]):
            # print('Approach IK failure', approach_conf)
            # wait_for_user()
            return None
        approach_conf = get_joint_positions(robot, arm_joints)
        attachment = grasp.get_attachment(problem.robot, arm)
        attachments = {attachment.child: attachment}
        if teleport:
            path = [default_conf, approach_conf, grasp_conf]
        else:
            resolutions = 0.05 ** np.ones(len(arm_joints))
            grasp_path = plan_direct_joint_motion(robot, arm_joints, grasp_conf, attachments=attachments.values(),
                                                  obstacles=approach_obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits=custom_limits, resolutions=resolutions / 2.)
            if grasp_path is None:
                return None
            set_joint_positions(robot, arm_joints, default_conf)
            approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                              obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                              custom_limits=custom_limits, resolutions=resolutions,
                                              restarts=2, iterations=25, smooth=25)
            if approach_path is None:
                return None
            path = approach_path + grasp_path
        mt = create_trajectory(robot, arm_joints, path)
        cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
        return (cmd,)

    return fn


def get_ik_ir_gen(problem, max_attempts=1000, learned=True, teleport=False, semantic_surface=None, **kwargs):
    # TODO: compose using general fn
    ir_sampler = get_ir_sampler(problem, learned=learned, max_attempts=10, semantic_surface=semantic_surface, **kwargs)
    ik_fn = get_ik_fn(problem, teleport=teleport, **kwargs)

    def gen(*inputs):
        count=0
        ir_generator = ir_sampler(*inputs)
        attempts = 0
        while True:
            if max_attempts <= attempts:
                print("Max attempts reached")
                print(attempts)
                import sys
                sys.exit()

                return
                attempts = 0
                
                yield None
            attempts += 1
            try:
                ir_outputs = next(ir_generator)
            except StopIteration:
                print("StopIteration")
                import sys
                sys.exit()
                return
            if ir_outputs is None:
                continue
            ik_outputs = ik_fn(*(inputs + ir_outputs))

            if ik_outputs is None:
                continue
            yield ir_outputs + ik_outputs
            return

    return gen


def get_compute_pose_kin(problem):
    # obstacles = world.static_obstacles
    def fn(o1, rp, o2, wp2):
        if o1 == o2:
            return None

        o2p = get_pose(o2)
        set_pose(o2, wp2.get_world_from_body())
        p1 = RelPose(o1, None, rp.get_world_from_body())
        set_pose(o2, o2p)
        return (p1,)
    return fn


def test_cfree(problem):
    # obstacles = world.static_obstacles
    def fn(o1, p1, r, o2, p2):
        print("test_cfree")
        if (problem.tray in [o1, o2] or problem.sinks[0] in [o1, o2] or problem.stoves[0] in [o1, o2]):
            return True

        if(o1 == o2):
            return False
            
        pose1 = p1.get_world_from_body()
        pose2 = p2.get_world_from_body()

        set_pose(o1, pose1)
        set_pose(o2, pose2)

        if (body_collision(o1, o2)):
            print("boo")
            return False

        return True

    return fn
    

    return (offset_pos, offset_rot)

def create_relative_pose(body, surface, **kwargs):
    body_pose = get_pose(body)
    surface_pose = get_pose(surface)
    relpose = multiply(invert(surface_pose), body_pose)
    return RelPose(body, surface, relpose, **kwargs)

def get_stable_gen(problem, max_attempts=100, no_rotation=False, **kwargs):
    # TODO: remove fixed collisions with contained surfaces
    # TODO: place where currently standing
    def gen(body, surface):
        while True:
            for _ in range(max_attempts):
                if (surface in problem.object_types.keys() and problem.object_types[surface] == "tray"):
                    body_pose_world = sample_tray_placement(body, surface)
                    # set_pose(body, multiply(get_pose(surface), body_pose_world))
                    set_pose(body, body_pose_world)
                else:
                    body_pose_world = sample_placement(body, surface, **kwargs)
                    if(no_rotation):
                        body_pose_world = (body_pose_world[0], unit_quat())
                    set_pose(body, body_pose_world)

                rp = create_relative_pose(body, surface)
                yield (rp,)
                break
            else:
                yield None

    return gen


def sample_tray_placement_on_aabb(top_body, bottom_aabb, top_pose=unit_pose(),
                                  percent=1.0, max_attempts=50, epsilon=1e-3):
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = Euler(yaw=theta)
        set_pose(top_body, multiply(QPose(euler=rotation), top_pose))
        center, extent = get_center_extent(top_body)

        lower = (np.array(bottom_aabb[0]) + percent * extent / 2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent * extent / 2)[:2]

        if np.less(upper, lower).any():
            continue

        x, y = np.random.uniform(lower, upper)

        z = (bottom_aabb[1] + extent / 2.)[2] + epsilon
        point = np.array([x, y, z]) + (get_point(top_body) - center)
        pose = multiply(QPose(point, rotation), top_pose)
        set_pose(top_body, pose)
        return pose
    return None


def sample_tray_placement(top_body, bottom_body, bottom_link=None, **kwargs):
    bottom_aabb = get_aabb(bottom_body, link=bottom_link)
    return sample_tray_placement_on_aabb(top_body, bottom_aabb, **kwargs)


def get_grasp_gen(problem, collisions=False, randomize=True):
    for grasp_type in problem.grasp_types:
        if grasp_type not in GET_GRASPS:
            raise ValueError('Unexpected grasp type:', grasp_type)

    def fn(body):
        # TODO: max_grasps
        # TODO: return grasps one by one
        grasps = []
        arm = 'left'
        # carry_conf = get_carry_conf(arm, 'top')
        if 'top' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE * get_unit_vector([1, 0, 0])
            if (problem.object_types[body] == "box"):
                grasps.extend(
                    Grasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM) for g in
                    get_top_grasps(body, grasp_length=GRASP_LENGTH))
            elif (problem.object_types[body] == "tray"):
                grasps.extend(
                    Grasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM) for g in
                    get_top_tray_grasps(body, grasp_length=GRASP_LENGTH))

        if 'side' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE * get_unit_vector([2, 0, -1])
            grasps.extend(Grasp('side', body, g, multiply((approach_vector, unit_quat()), g), SIDE_HOLDING_LEFT_ARM)
                          for g in get_side_grasps(body, grasp_length=GRASP_LENGTH))
        filtered_grasps = []
        for grasp in grasps:
            grasp_width = compute_grasp_width(problem.robot, arm, body, grasp.value) if collisions else 0.0
            if grasp_width is not None:
                grasp.grasp_width = grasp_width
                filtered_grasps.append(grasp)
                if(problem.object_types[body] != "tray"):
                    break
        if randomize:
            random.shuffle(filtered_grasps)

        return [(g,) for g in filtered_grasps]

    # for g in filtered_grasps:
    #    yield (g,)
    return fn
