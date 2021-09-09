

from pddlstream.language.generator import from_gen_fn, from_list_fn,\
     from_test, from_fn

from pddlstream.utils import ensure_dir, safe_rm_dir, user_input, read, INF, \
     get_file_path, implies, inclusive_range

from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import invert, multiply, get_name, set_pose, get_link_pose, is_placement, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, waypoints_from_path, \
    unit_quat, plan_base_motion, plan_joint_motion, base_values_from_pose, pose_from_base_values, \
    uniform_pose_generator, sub_inverse_kinematics, add_fixed_constraint, remove_debug, remove_fixed_constraint, \
    disable_real_time, enable_gravity, joint_controller_hold, get_distance, \
    get_min_limit, user_input, step_simulation, get_body_name, get_bodies, BASE_LINK, \
    add_segments, get_max_limit, link_from_name, BodySaver, get_aabb, Attachment, interpolate_poses, \
    plan_direct_joint_motion, has_gui, create_attachment, wait_for_duration, get_extend_fn, set_renderer, \
    get_custom_limits, all_between, get_unit_vector, wait_if_gui, \
    set_base_values, euler_from_quat, INF, elapsed_time, get_moving_links, flatten_links, get_relative_pose, \
    any_link_pair_collision

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_stable_gen, Pose
from pddlstream.language.generator import from_fn, BoundedGenerator, \
    from_sampler

from GeneralizedTAMP.pybullet_planning.pybullet_tools.ikfast.pr2.ik import is_ik_compiled, pr2_inverse_kinematics

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, get_side_grasps,SIDE_HOLDING_LEFT_ARM,get_top_grasps, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf, GET_GRASPS, \
    TOP_HOLDING_LEFT_ARM, MAX_GRASP_WIDTH, PR2_TOOL_FRAMES, learned_pose_generator, get_gripper_link, GRASP_LENGTH, \
    compute_grasp_width
import numpy as np

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_motion_gen, Conf, Attach, Detach, Clean, \
    Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, APPROACH_DISTANCE, Grasp, GRASP_LENGTH, \
    iterate_approach_path, Commands, Command
from itertools import chain
import random
import pybullet as p

from utils import SplitTrajectory, create_trajectory
import time 
COLLISION_DISTANCE = 5e-3 # Distance from fixed obstacles
#MOVABLE_DISTANCE = 1e-2 # Distance from movable objects
MOVABLE_DISTANCE = COLLISION_DISTANCE



def get_grasp_gen(problem, grasp_types = ["side"], collisions=False, randomize=True):
    for grasp_type in grasp_types:
        if grasp_type not in GET_GRASPS:
            raise ValueError('Unexpected grasp type:', grasp_type)
    def fn(body):
        # TODO: max_grasps
        # TODO: return grasps one by one
        grasps = []
        arm = 'left'
        #carry_conf = get_carry_conf(arm, 'top')
        if 'top' in grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([1, 0, 0])
            grasps.extend(Grasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM)
                          for g in get_top_grasps(body, grasp_length=GRASP_LENGTH))
        if 'side' in grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([2, 0, -1])
            grasps.extend(Grasp('side', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM)
                          for g in [get_side_grasps(body, grasp_length=GRASP_LENGTH)[1]])
        filtered_grasps = []
        for grasp in list(grasps):
            grasp_width = compute_grasp_width(problem.robot, arm, body, grasp.value) if collisions else 0.0
            if grasp_width is not None:
                grasp.grasp_width = grasp_width
                filtered_grasps.append(grasp)
                if('side' in grasp_types):
                    break
        if randomize:
            random.shuffle(filtered_grasps)
        return [(g,) for g in filtered_grasps]
        #for g in filtered_grasps:
        #    yield (g,)
    return fn


def get_stable_gen(problem, collisions=True, **kwargs):
    obstacles = problem.fixed if collisions else []
    def gen(body, surface):
        # TODO: surface poses are being sampled in pr2_belief
        if surface is None:
            surfaces = problem.surfaces
        else:
            surfaces = [surface]
        while True:
            surface = random.choice(surfaces) # TODO: weight by area
            body_pose = sample_placement(body, surface, **kwargs)
            if body_pose is None:
                break

            p = Pose(body, body_pose, surface)
            p.assign()

            if not any(pairwise_collision(body, obst) for obst in obstacles if obst not in {body, surface}):
                # if(body_pose[0][0] < 0):
                yield (p,)
    # TODO: apply the acceleration technique here
    return gen


def get_cfree_traj_pose_test(problem):
    arm = problem.arms[0]
    def test(sequence, obj2, pose2):
        if (pose2 is None): # (obj1 == obj2) or
            return True

        pose2.assign()
        #state = State() # TODO: apply to the context
        for traj in sequence.commands:
            all_joints = []
            for el in traj.path:
                all_joints+=el.joints
            all_joints = list(set(all_joints))
            moving_links = get_moving_links(problem.robot, all_joints)
            #for _ in command.iterate(state=None):
            for conf in traj.path:
                set_joint_positions(problem.robot, all_joints, conf.values)
                if any_link_pair_collision(problem.robot, moving_links, obj2, max_distance=MOVABLE_DISTANCE):
                    return False
        return True
    return test


def get_ik_fn(problem, custom_limits={}, collisions=True, teleport=False, max_attempts=20):
    SELF_COLLISIONS = False
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    arm = problem.arms[0]
    if is_ik_compiled():
        print('Using ikfast for inverse kinematics')
    else:
        print('Using pybullet for inverse kinematics')

    def fn(obj, pose, grasp):
        for _ in range(max_attempts):
            approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
            gripper_pose = multiply(pose.value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
            #approach_pose = multiply(grasp.approach, gripper_pose)
            approach_pose = multiply(pose.value, invert(grasp.approach))
            arm_link = get_gripper_link(robot, arm)
            arm_joints = get_arm_joints(robot, arm)

            default_conf = arm_conf(arm, grasp.carry)
            #sample_fn = get_sample_fn(robot, arm_joints)
            pose.assign()
            problem.initial_bq.assign()
            open_arm(robot, arm)
            set_joint_positions(robot, arm_joints, default_conf) # default_conf | sample_fn()
            grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose, custom_limits=custom_limits) #, upper_limits=USE_CURRENT)
                                                #nearby_conf=USE_CURRENT) # upper_limits=USE_CURRENT,

            if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles): # [obj]
                print('Grasp IK failure', grasp_conf)
                #if grasp_conf is not None:
                #    print(grasp_conf)
                #    #wait_if_gui()
                continue
            #approach_conf = pr2_inverse_kinematics(robot, arm, approach_pose, custom_limits=custom_limits,
            #                                       upper_limits=USE_CURRENT, nearby_conf=USE_CURRENT)
            approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose, custom_limits=custom_limits, ori_tolerance=100, pos_tolerance=1)
            if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles + [obj]):
                print('Approach IK failure', approach_conf)
                #wait_if_gui()
                continue
            approach_conf = get_joint_positions(robot, arm_joints)
            attachment = grasp.get_attachment(problem.robot, arm)
            attachments = {attachment.child: attachment}
            if teleport:
                path = [default_conf, approach_conf, grasp_conf]
            else:
                resolutions = 0.05**np.ones(len(arm_joints))
                grasp_path = plan_direct_joint_motion(robot, arm_joints, grasp_conf, attachments=attachments.values(),
                                                      obstacles=approach_obstacles, self_collisions=SELF_COLLISIONS,
                                                      custom_limits=custom_limits, resolutions=resolutions/2.)
                if grasp_path is None:
                    print('Grasp path failure')
                    continue
                set_joint_positions(robot, arm_joints, default_conf)
                approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                                  obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits=custom_limits, resolutions=resolutions,
                                                  restarts=2, iterations=25, smooth=25)
                if approach_path is None:
                    print('Approach path failure')
                    continue

            mt = create_trajectory(robot, arm_joints, approach_path, grasp_path)
            cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
            print("Success!")
            return (cmd,)
        return None
    return fn