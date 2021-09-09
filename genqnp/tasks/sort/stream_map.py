

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
    set_base_values, euler_from_quat, INF, elapsed_time, get_moving_links, flatten_links, get_relative_pose

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_stable_gen, get_grasp_gen
from pddlstream.language.generator import from_fn, BoundedGenerator, \
    from_sampler

from GeneralizedTAMP.pybullet_planning.pybullet_tools.ikfast.pr2.ik import is_ik_compiled, pr2_inverse_kinematics

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf, PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf, GET_GRASPS, \
    TOP_HOLDING_LEFT_ARM, MAX_GRASP_WIDTH, PR2_TOOL_FRAMES, learned_pose_generator, get_gripper_link, GRASP_LENGTH
import numpy as np

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_motion_gen, Conf, Attach, Detach, Clean, \
    Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, APPROACH_DISTANCE, Grasp, get_grasp_gen, GRASP_LENGTH, \
    iterate_approach_path, create_trajectory, Commands


def get_ik_fn(problem, custom_limits={}, collisions=True, teleport=False, max_attempts=5):
    SELF_COLLISIONS = False
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    arm = problem.arms[0]
    if is_ik_compiled():
        print('Using ikfast for inverse kinematics')
    else:
        print('Using pybullet for inverse kinematics')

    def fn(obj, pose, grasp, base_conf):
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
            base_conf.assign()
            open_arm(robot, arm)
            set_joint_positions(robot, arm_joints, default_conf) # default_conf | sample_fn()
            grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose, custom_limits=custom_limits) #, upper_limits=USE_CURRENT)
                                                #nearby_conf=USE_CURRENT) # upper_limits=USE_CURRENT,
            if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles): # [obj]
                #if grasp_conf is not None:
                #    print(grasp_conf)
                #    #wait_if_gui()
                continue
            #approach_conf = pr2_inverse_kinematics(robot, arm, approach_pose, custom_limits=custom_limits,
            #                                       upper_limits=USE_CURRENT, nearby_conf=USE_CURRENT)
            approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose, custom_limits=custom_limits)
            if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles + [obj]):
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
                    continue
                set_joint_positions(robot, arm_joints, default_conf)
                approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                                  obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits=custom_limits, resolutions=resolutions,
                                                  restarts=2, iterations=25, smooth=25)
                if approach_path is None:
                    continue
                path = approach_path + grasp_path
            mt = create_trajectory(robot, arm_joints, path)
            cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
            return (cmd,)
        return None
    return fn

def get_stream_map(problem):
    return {
            'sample-pose': from_gen_fn(get_stable_gen(problem)),
            'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=True)),
            'inverse-kinematics': from_sampler(get_ik_fn(problem,
                                                         teleport=False))
        }