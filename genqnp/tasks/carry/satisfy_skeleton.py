# TINY TAMP: given a skeleton, find the satisfying assignments

import copy
import sys
import time
from collections import defaultdict

import numpy as np
import pybullet as p
import skvideo.io
from GeneralizedTAMP.genqnp.genqnp_utils import apply_commands, PolicyGraph
from GeneralizedTAMP.genqnp.tasks.carry.run import carry_problem, carry_post_process, carry_pddlstream_from_problem, \
    CloseAttach, CloseDetach
from GeneralizedTAMP.language.planner import find_plan, apply_action, task_from_domain_problem, get_action_instances, \
    parse_sequential_domain, parse_problem
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_ik_fn, \
    get_motion_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, State, APPROACH_DISTANCE, Grasp, get_grasp_gen, GRASP_LENGTH
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_problems import create_table
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf, GET_GRASPS, \
    TOP_HOLDING_LEFT_ARM, MAX_GRASP_WIDTH, PR2_TOOL_FRAMES
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import set_pose, connect, get_pose, is_placement, \
    point_from_pose, wait_for_duration, \
    disconnect, user_input, get_joint_positions, enable_gravity, save_state, restore_state, HideOutput, \
    get_distance, LockRenderer, get_min_limit, get_max_limit, load_model, load_pybullet, create_box, set_point, \
    get_bodies, create_plane, TAN, GREY, PI, joints_from_names, stable_z, body_collision, Euler, get_unit_vector, \
    multiply, unit_quat, get_box_geometry, STATIC_MASS, BROWN, unit_pose, create_shape_array, \
    create_body, get_closest_points, sample_placement, get_aabb, pairwise_collision, CIRCULAR_LIMITS, Point, Euler, \
    quat_from_euler, get_center_extent, get_point, create_attachment, link_from_name, stable_z, VideoSaver
from PIL import Image
from graphviz import render
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import Equal, AND, print_solution
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen
from pddlstream.language.stream import StreamInfo, PartialInputs, OptValue
from pddlstream.utils import read, INF, get_file_path, find_unique

satisfaction_type = "no_search"  # "mcts"
viewer = True

policy_graph = PolicyGraph("./genqnp/tasks/carry/")
print(policy_graph.G.nodes)

# Hardcoding some solutions
connect(use_gui=viewer)

problem_fn = carry_problem
problem = problem_fn()
p.stepSimulation()
pddlstream_problem, stream_info = carry_pddlstream_from_problem(problem, teleport=False)
state_id = save_state()
domain, _, _, stream_map, init, goal = pddlstream_problem

parsed_domain = parse_sequential_domain(domain)

current_state = init

print(init)

commands = []

q = problem.get_init_baseconf(init)
rp = Pose(problem.tray, get_pose(problem.tray))
sinkp = Pose(problem.sinks[0], get_pose(problem.sinks[0]))
stovep = Pose(problem.stoves[0], get_pose(problem.stoves[0]))

a = problem.arms[0]
# movable_moved = defaultdict(lambda: False)
block_poses = defaultdict(lambda: None)
block_attachments = defaultdict(lambda: None)
gripper_joint = get_gripper_joints(problem.robot, a)[0]
position = get_max_limit(problem.robot, gripper_joint)


def all_collision_test(b0, p0, blocks, poses):
    set_pose(b0, p0.value)
    for block, pose in zip(blocks, poses):
        set_pose(block, pose.value)
        if (b0 != block and body_collision(b0, block)):
            return True
    return False


MAX_TEST = 10
for b in problem.blocks:
    block_poses[b] = Pose(b, get_pose(b))
tray_pose = Pose(problem.tray, get_pose(problem.tray))

it = 0
capacity = 2

blocks_not_moved = {}
for b in problem.blocks:
    blocks_not_moved[b] = 1
loopnum = 0
while (sum(list(blocks_not_moved.values())) > 0):
    print("loopnum: " + str(loopnum))
    loopnum += 1
    num_success = 0
    tray_blocks = []
    set_pose(problem.tray, tray_pose.value)
    for b in problem.blocks:
        if (not blocks_not_moved[b]):
            continue
        valid = False
        successes = 0
        for _ in range(MAX_TEST):

            new_commands = []
            block_pose = block_poses[b]

            # Get grasp for object
            (g,) = next(stream_map['sample-grasp'](b))[0]

            # Get the robot configuration
            try:
                (q2, t1) = next(stream_map['inverse-kinematics'](a, b, block_pose, g))[0]
            except:
                print("FAILD IRIK 1")
                continue
            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q2))[0]
            q = q2

            new_commands += qt.commands

            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=True)
            attach = Attach(problem.robot, a, g, b)
            close_attach = CloseAttach(problem.robot, a, g, problem.movable, b)
            new_commands += [t1.commands[0], close_gripper, attach, close_attach, t1.commands[0].reverse()]

            # Place block on tray

            # Get the region pose
            (on_region_pose,) = next(stream_map['sample-pose'](b, problem.tray, rp))[0]

            # Place object on region
            try:
                (q3, t2) = next(stream_map['inverse-kinematics'](a, b, on_region_pose, g))[0]
            except:
                print("FAILD IRIK 2")
                continue

            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q3))[0]
            q = q3

            new_commands += qt.commands
            open_gripper = GripperCommand(problem.robot, a, position, teleport=True)
            detach = Detach(problem.robot, a, b)
            close_detach = CloseDetach(problem.robot, a, g, problem.movable, b)
            new_commands += [t2.commands[0], detach, open_gripper, close_detach, t2.commands[0].reverse()]

            if (not all_collision_test(b, on_region_pose, block_poses.keys(), block_poses.values())):
                valid = True
                set_pose(b, on_region_pose.value)
                block_attachments[b] = create_attachment(problem.tray, -1, b)
                print("FAILD collision test")
                break

        if (not valid):
            break

        blocks_not_moved[b] = 0
        num_success += 1
        tray_blocks.append(b)

        commands += new_commands
        if (num_success >= capacity):
            break

    # Move tray over
    for _ in range(MAX_TEST):
        new_commands = []

        # Pick Tray
        (g,) = next(stream_map['sample-grasp'](problem.tray))[0]

        # Get the robot configuration
        try:
            (q2, t1) = next(stream_map['inverse-kinematics'](a, problem.tray, tray_pose, g))[0]
        except:
            continue

        # Plan base motion
        (qt,) = next(stream_map['plan-base-motion'](q, q2))[0]
        q = q2

        new_commands += qt.commands

        close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=True)
        attach = Attach(problem.robot, a, g, problem.tray)
        close_attach = CloseAttach(problem.robot, a, g, problem.movable, problem.tray)
        new_commands += [t1.commands[0], close_gripper, attach, close_attach, t1.commands[0].reverse()]

        # Get the region pose
        (on_region_pose,) = next(stream_map['sample-pose'](problem.tray, problem.sinks[0], sinkp))[0]

        # Place object on region
        try:
            (q3, t2) = next(stream_map['inverse-kinematics'](a, problem.tray, on_region_pose, g))[0]
        except:
            continue

        # Plan base motion
        (qt,) = next(stream_map['plan-base-motion'](q, q3))[0]
        q = q3

        new_commands += qt.commands
        open_gripper = GripperCommand(problem.robot, a, position, teleport=True)
        detach = Detach(problem.robot, a, problem.tray)
        close_detach = CloseDetach(problem.robot, a, g, problem.movable, problem.tray)
        new_commands += [t2.commands[0], detach, open_gripper, close_detach, t2.commands[0].reverse()]
        tray_pose = on_region_pose
        break

    commands += new_commands

    for b in tray_blocks:
        set_pose(problem.tray, tray_pose.value)
        block_attachments[b].assign()
        block_poses[b] = Pose(b, get_pose(b))

    for b in tray_blocks:

        valid = False
        for _ in range(MAX_TEST):
            new_commands = []

            block_pose = block_poses[b]

            # Get grasp for object
            (g,) = next(stream_map['sample-grasp'](b))[0]

            # Get the robot configuration
            try:
                (q2, t1) = next(stream_map['inverse-kinematics'](a, b, block_pose, g))[0]
            except:
                continue

            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q2))[0]
            q = q2

            new_commands += qt.commands

            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=True)
            attach = Attach(problem.robot, a, g, b)
            close_attach = CloseAttach(problem.robot, a, g, problem.movable, b)
            new_commands += [t1.commands[0], close_gripper, attach, close_attach, t1.commands[0].reverse()]

            # Get the region pose
            (on_region_pose,) = next(stream_map['sample-pose'](b, problem.sinks[0], sinkp))[0]

            # Place object on region

            try:
                (q3, t2) = next(stream_map['inverse-kinematics'](a, b, on_region_pose, g))[0]
            except:
                continue

            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q3))[0]
            q = q3

            new_commands += qt.commands
            open_gripper = GripperCommand(problem.robot, a, position, teleport=True)
            detach = Detach(problem.robot, a, b)
            close_detach = CloseDetach(problem.robot, a, g, problem.movable, b)
            new_commands += [t2.commands[0], detach, open_gripper, close_detach, t2.commands[0].reverse()]

            if (not all_collision_test(b, on_region_pose, list(block_poses.keys()) + [problem.tray],
                                       list(block_poses.values()) + [tray_pose])):
                valid = True
                break

        if (not valid):
            break

        block_poses[b] = on_region_pose

        commands += new_commands

    if (sum(list(blocks_not_moved.values())) > 0):
        # Move tray back
        for _ in range(MAX_TEST):
            new_commands = []

            # Pick Tray
            (g,) = next(stream_map['sample-grasp'](problem.tray))[0]

            # Get the robot configuration
            try:
                (q2, t1) = next(stream_map['inverse-kinematics'](a, problem.tray, tray_pose, g))[0]
            except:
                continue

            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q2))[0]
            q = q2

            new_commands += qt.commands

            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=True)
            attach = Attach(problem.robot, a, g, problem.tray)
            close_attach = CloseAttach(problem.robot, a, g, problem.movable, problem.tray)
            new_commands += [t1.commands[0], close_gripper, attach, close_attach, t1.commands[0].reverse()]

            # Get the region pose
            (on_region_pose,) = next(stream_map['sample-pose'](problem.tray, problem.stoves[0], stovep))[0]

            # Place object on region
            try:
                (q3, t2) = next(stream_map['inverse-kinematics'](a, problem.tray, on_region_pose, g))[0]
            except:
                continue

            # Plan base motion
            (qt,) = next(stream_map['plan-base-motion'](q, q3))[0]
            q = q3

            new_commands += qt.commands
            open_gripper = GripperCommand(problem.robot, a, position, teleport=True)
            detach = Detach(problem.robot, a, problem.tray)
            close_detach = CloseDetach(problem.robot, a, g, problem.movable, problem.tray)
            new_commands += [t2.commands[0], detach, open_gripper, close_detach, t2.commands[0].reverse()]
            tray_pose = on_region_pose
            break

        commands += new_commands

restore_state(state_id)

print("Creating video ... ")

apply_commands(State(), commands, policy_graph, time_step=0.01, save_video=True)
