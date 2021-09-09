#!/usr/bin/env python

from __future__ import print_function

import cProfile
import os
import random
from collections import namedtuple

import pybullet as p
from tasks.gentask import GenTask, ProblemSpec
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import \
    Pose, Conf, Attach, Detach, \
    get_gripper_joints, GripperCommand, apply_commands, State
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_problems import \
    create_table
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import \
    rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf,\
    PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, \
    get_group_joints, get_group_conf
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import set_pose,\
    connect, get_pose, is_placement, \
    disconnect, user_input, get_joint_positions, \
    HideOutput, get_distance, LockRenderer, get_max_limit, load_model, \
    load_pybullet, get_bodies, create_plane, TAN, GREY, PI, joints_from_names,\
    stable_z, body_collision
from search import solve_mcts
from pddlstream.language.constants import AND, print_solution
from pddlstream.language.generator import from_fn, BoundedGenerator, \
    from_sampler
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, INF, get_file_path
from tasks.sort.stream_map import get_stream_map
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST
from utils import get_arg_string
import math


BASE_CONSTANT = 1
BASE_VELOCITY = 0.5
MODEL_PATH = './models'
LTAMP_PR2 = os.path.join(MODEL_PATH, 'pr2_description/pr2.urdf')
RED_REGION_URDF = "../models/recycle/red_region.urdf"
BLUE_REGION_URDF = "../models/recycle/blue_region.urdf"
RED_CAN_URDF = "../models/recycle/red_can.urdf"
BLUE_CAN_URDF = "../models/recycle/blue_can.urdf"


class SortProblemSpec(ProblemSpec):
    def __init__(self, robot, arms=tuple(), movable=tuple(),
                 grasp_types=tuple(),
                 surfaces=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(),
                 goal_cleaned=tuple(), costs=False,
                 table=None,
                 body_names={}, body_types=[], base_limits=None,
                 task_problem=None,
                 red_cans=[], blue_cans=[], red_goal=None, blue_goal=None,
                 blue_poses=[], red_poses=[]):
        self.robot = robot
        self.arms = arms
        self.red_cans = red_cans
        self.blue_cans = blue_cans
        self.blue_poses = blue_poses
        self.red_poses = red_poses
        self.movable = movable
        self.table = table
        self.grasp_types = grasp_types
        self.surfaces = surfaces
        self.red_goal = red_goal
        self.blue_goal = blue_goal
        self.goal_conf = goal_conf
        self.goal_holding = goal_holding
        self.goal_cleaned = goal_cleaned
        self.body_names = body_names
        self.body_types = body_types
        self.base_limits = base_limits
        self.task_problem = task_problem
        self.cost = INF
        all_movable = [self.robot] + list(self.movable)
        self.fixed = list(filter(lambda b: b not in all_movable, get_bodies()))
        self.gripper = None
        super(SortProblemSpec, self).__init__()

        self.nonfluents = ["atraj", "grasp", "graspable", "kin", "pose", "supported", "stackable", "traj_cfree"]


    def get_gripper(self, arm='left'):
        if self.gripper is None:
            self.gripper = create_gripper(self.robot, arm=arm)
        return self.gripper

    def __repr__(self):
        return repr(self.__dict__)


def from_list_fn(list_fn):
    # return lambda *args, **kwargs: iter([list_fn(*args, **kwargs)])

    def list_gen_fn(*args, **kwargs):
        return BoundedGenerator(iter([list_fn(*args, **kwargs)]),
                                max_calls=1)

    return list_gen_fn


def from_gen_fn(gen_fn):
    def list_gen_fn(*args, **kwargs):
        return ([] if (ov is None or ov is {}) else [ov] for ov in
                gen_fn(*args, **kwargs))

    return list_gen_fn


def set_group_positions(pr2, group_positions):
    for group, positions in group_positions.items():
        joints = joints_from_names(pr2, PR2_GROUPS[group])
        print(len(joints), len(positions))
        assert len(joints) == len(positions)
        set_joint_positions(pr2, joints, positions)


def move_cost_fn(c):
    [t] = c.commands
    distance = t.distance(distance_fn=lambda q1, q2:
                          get_distance(q1[:2], q2[:2]))
    return BASE_CONSTANT + distance / BASE_VELOCITY


def opt_pose_fn(o, r):
    CustomValue = namedtuple('OptValue', ['stream', 'values'])
    p = CustomValue('p-sp', (r,))
    return p,


def opt_ik_fn(o, p, g, q):
    CustomValue = namedtuple('OptValue', ['stream', 'values'])
    t = CustomValue('t-ik', tuple())
    return t,


def opt_motion_fn(q1, q2):
    CustomValue = namedtuple('OptValue', ['stream', 'values'])
    t = CustomValue('t-pbm', (q1, q2))
    return t,


class Sort(GenTask):

    def __init__(self, *args, **kwargs):
        super(Sort, self).__init__(*args, **kwargs)

    def place_movable(certified):
        for literal in certified:
            if literal[0] != 'not':
                continue
            fact = literal[1]
            if fact[0] == 'trajposecollision':
                _, b, p = fact[1:]
                p.assign()
            if fact[0] == 'trajarmcollision':
                _, a, q = fact[1:]
                q.assign()
            if fact[0] == 'trajgraspcollision':
                _, a, o, g = fact[1:]
                # TODO: finish this

    def pddlstream_from_spec(self, spec):
        robot = spec.robot
        domain_pddl = read(self.domain)
        stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
        constant_map = {}

        # initial_bq = Pose(robot, get_pose(robot))
        initial_bq = Conf(robot, get_group_joints(robot, 'base'),
                          get_group_conf(robot, 'base'))
        init = [
            ('bconf', initial_bq),
            ('atbconf', initial_bq)
        ]
        arm = ARM_NAMES[0]
         # for arm i]n problem.arms:
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('aconf', conf),
                 ('handempty', ),
                 ('ataconf', conf)]

        init += [('red_region', spec.red_goal), ('blue_region', spec.blue_goal)]

        for body in spec.red_cans:
            pose = Pose(body, get_pose(body))
            init += [('graspable', body), ('pose', body, pose), ('atpose', body, pose), ('red', body)]
            for surface in spec.surfaces:
                init += [('stackable', body, surface)]
                if is_placement(body, surface, below_epsilon=1e-2):
                    init += [('supported', body, pose, surface)]

        for body in spec.blue_cans:
            pose = Pose(body, get_pose(body))
            init += [('graspable', body), ('pose', body, pose), ('atpose', body, pose), ('blue', body)]
            for surface in spec.surfaces:
                init += [('stackable', body, surface)]
                if is_placement(body, surface, below_epsilon=1e-2):
                    init += [('supported', body, pose, surface)]
        init += [('table', spec.table)]

        red_goal_pose = Pose(spec.red_goal, get_pose(spec.red_goal))
        init += [('supported', spec.red_goal, red_goal_pose, spec.table), ('pose', spec.red_goal, red_goal_pose), ('atpose', spec.red_goal, red_goal_pose)]

        blue_goal_pose = Pose(spec.blue_goal, get_pose(spec.blue_goal))
        init += [('supported', spec.blue_goal, blue_goal_pose, spec.table), ('pose', spec.blue_goal, blue_goal_pose), ('atpose', spec.blue_goal, blue_goal_pose)]

        table_pose = Pose(spec.table, get_pose(spec.table))
        init += [('pose', spec.table, table_pose), ('atpose', spec.table, table_pose)]

        goal = [AND]
        for body in spec.blue_cans:
            goal += [('on', body, spec.blue_goal)]

        for body in spec.red_cans:
            goal += [('on', body, spec.red_goal)]
    
        stream_map = get_stream_map(spec)
        

        return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,
                init, goal)

    def create_floor():
        return load_pybullet("plane.urdf")

    def post_process(self, problem, init, plan, teleport=False):
        a = problem.arms[0]

        if plan is None:
            return None
        commands = []

        for i, (node) in enumerate(plan):
            if(node.action_name is not None):
                name, args = node.action_name, node.action_args
                if name == 'pick':
                    b, p, g, _, c = args
                    [t] = c.commands
                    close_gripper = GripperCommand(problem.robot, a, g.grasp_width,
                                                   teleport=teleport)
                    attach = Attach(problem.robot, a, g, b)
                    new_commands = [t, close_gripper, attach, t.reverse()]
                elif name == 'place':
                    b, p, g, _, c = args
                    [t] = c.commands
                    gripper_joint = get_gripper_joints(problem.robot, a)[0]
                    position = get_max_limit(problem.robot, gripper_joint)
                    open_gripper = GripperCommand(problem.robot, a, position,
                                                  teleport=teleport)
                    detach = Detach(problem.robot, a, b)
                    new_commands = [t, detach, open_gripper, t.reverse()]
                else:
                    raise ValueError(name)
                commands += new_commands
        return commands, {}

    #######################################################

    def create_pr2(use_drake=True, fixed_base=True, torso=0.2):
        pr2_path = DRAKE_PR2_URDF if use_drake else PR2_URDF
        with LockRenderer():
            with HideOutput():
                pr2 = load_model(pr2_path, fixed_base=fixed_base)
            set_group_conf(pr2, 'torso', [torso])
        return pr2

    def terminate(self):
        disconnect()

    def check_body_collision(self, b1, b2, thresh = 0.1):
        poseb1 = get_pose(b1)[0]
        poseb2 = get_pose(b2)[0]
        return math.sqrt((poseb1[0]-poseb2[0])**2+(poseb1[1]-poseb2[1])**2) <= thresh

    def place_cans_randomly(self, surface, red=3, blue=3,
                            table_width=1.2, region_width=0.4):
        random.seed(self.seed)
        sample_rad = (table_width - 2 * region_width) / 2.5
        red_cans = []
        red_can_poses = []
        for _ in range(red):
            in_collision = True
            red_cans.append(load_model(RED_CAN_URDF))
            while (in_collision):
                in_collision = False
                x_pos, y_pos = random.uniform(-0.13, 0.13), \
                    random.uniform(-sample_rad, sample_rad)

                cstable_z = stable_z(red_cans[-1], surface)
                red_can_poses.append(([x_pos, y_pos, cstable_z],
                                      p.getQuaternionFromEuler([0, 0, 0])))
                set_pose(red_cans[-1], red_can_poses[-1])

                for existing_can in red_cans[:-1]:
                    if (self.check_body_collision(existing_can, red_cans[-1])):
                        in_collision = True
                        break

        blue_cans = []
        blue_can_poses = []
        for _ in range(blue):
            in_collision = True
            blue_cans.append(load_model(BLUE_CAN_URDF))
            while (in_collision):
                in_collision = False
                x_pos, y_pos = random.uniform(-0.13, 0.13),\
                    random.uniform(-sample_rad, sample_rad)

                cstable_z = stable_z(blue_cans[-1], surface)
                blue_can_poses.append(([x_pos, y_pos, cstable_z],
                                      p.getQuaternionFromEuler([0, 0, 0])))
                set_pose(blue_cans[-1], blue_can_poses[-1])

                for existing_can in red_cans+blue_cans[:-1]:
                    if (body_collision(existing_can, blue_cans[-1])):
                        in_collision = True
                        break

        return blue_cans, red_cans, blue_can_poses, red_can_poses

    def get_problem_spec(self, vis=False, problem=None):

        connect(use_gui=vis)

        arm = "left"
        grasp_type = "top"
        other_arm = get_other_arm(arm)
        initial_conf = get_carry_conf(arm, grasp_type)
        initial_other_conf = get_carry_conf(other_arm, grasp_type)

        pr2 = load_pybullet(LTAMP_PR2)
        set_arm_conf(pr2, arm, initial_conf)
        open_arm(pr2, arm)
        set_arm_conf(pr2, other_arm, arm_conf(other_arm, initial_other_conf))
        close_arm(pr2, other_arm)

        group_positions = {
            'base': [-0.5, 0, 0],
            'left_arm': REST_LEFT_ARM,
            'right_arm': rightarm_from_leftarm(REST_LEFT_ARM),
            'torso': [0.2],
            'head': [0, PI / 4],
        }
        set_group_positions(pr2, group_positions)

        # TODO
        # floor = p.loadURDF('./models/kitchen/floor/floor.urdf',useFixedBase=True,  globalScaling=0.8)
        # kitchen = p.loadURDF("./models/kitchen/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf",[-5,0,1.477], useFixedBase=True, globalScaling=0.8)
        # table = p.loadURDF('./models/kitchen/table/table.urdf',[0.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True,  globalScaling=0.8)

        table = create_table(width=0.6, length=1.2, height=0.73,
                                     leg_color=GREY, cylinder=False)
        # Create the red region object
        red_region = load_model(RED_REGION_URDF)
        set_pose(red_region, ([0, 0.3, stable_z(red_region, table)],
                 p.getQuaternionFromEuler([0, 0, 0])))

        # Create the blue region object
        blue_region = load_model(BLUE_REGION_URDF)
        set_pose(blue_region, ([0, -0.3, stable_z(blue_region, table)],
                 p.getQuaternionFromEuler([0, 0, 0])))

        num_red = 0
        num_blue = 0
        for literal in self.task_problem.init:
            if literal.predicate == "red":
                num_red += 1
            elif literal.predicate == "blue":
                num_blue += 1

        print("num red num blue")
        print(num_red, num_blue)

        blue_cans, red_cans, blue_poses, red_poses = \
            self.place_cans_randomly(table, red=num_red, blue=num_blue)

        # Step in simulation to set everything up
        # import time
        for _ in range(5):
            p.stepSimulation()
            # time.sleep(0.1)

        return SortProblemSpec(robot=pr2,
                               red_cans=red_cans, blue_cans=blue_cans,
                               blue_poses=blue_poses,
                               red_poses=red_poses,
                               movable=red_cans + blue_cans,
                               arms=["left"],
                               grasp_types=["top"],
                               surfaces=[table, red_region, blue_region],
                               table= table,
                               red_goal=red_region,
                               blue_goal=blue_region,
                               task_problem=self.task_problem)

