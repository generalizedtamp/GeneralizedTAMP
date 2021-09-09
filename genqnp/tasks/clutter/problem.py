#!/usr/bin/env python

from __future__ import print_function

import cProfile
import os
import random
from collections import namedtuple

import pybullet as p
from tasks.gentask import GenTask, ProblemSpec
from tasks.clutter.stream_map import get_ik_fn, get_cfree_traj_pose_test,\
    get_stable_gen, get_grasp_gen
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
    connect, get_pose, is_placement, remove_body, \
    disconnect, user_input, get_joint_positions, \
    HideOutput, get_distance, LockRenderer, get_max_limit, load_model, \
    load_pybullet, get_bodies, create_plane, TAN, GREY, PI, joints_from_names,\
    stable_z, body_collision, set_default_camera, remove_body
from pddlstream.language.constants import AND, print_solution
from pddlstream.language.generator import from_fn, BoundedGenerator, \
    from_sampler
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, INF, get_file_path
from search import solve_mcts
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST
from pddlstream.language.generator import from_test
import time
import math

BASE_CONSTANT = 1
BASE_VELOCITY = 0.5
MODEL_PATH = './models'
LTAMP_PR2 = os.path.join(MODEL_PATH, 'pr2_description/pr2.urdf')
RED_CAN_URDF = "../models/recycle/red_can.urdf"
BLUE_CAN_URDF = "../models/recycle/blue_can_tall.urdf"


class ClutterProblemSpec(ProblemSpec):
    def __init__(self, robot, arms=tuple(), movable=tuple(),
                 grasp_types=tuple(),
                 surfaces=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(),
                 goal_cleaned=tuple(), costs=False,
                 body_names={}, body_types=[], base_limits=None,
                 task_problem=None,
                 red_can=None, blue_cans=[],
                 initial_bq=None,
                 blue_poses=[], red_pose=None):


        super(ClutterProblemSpec, self).__init__()
        self.robot = robot
        self.arms = arms
        self.red_can = red_can
        self.blue_cans = blue_cans
        self.blue_poses = blue_poses
        self.red_pose = red_pose
        self.movable = movable
        self.initial_bq = initial_bq
        self.grasp_types = grasp_types
        self.surfaces = surfaces
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
        self.nonfluents = ["stackable", "grasp", "kin", "basemotion", "armmotion", "traj_cfree", "atraj", "supported", "pose", "graspable", "trajto"]


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


class Clutter(GenTask):

    def __init__(self, *args, **kwargs):
        super(Clutter, self).__init__(*args, **kwargs)

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

    def pddlstream_from_spec(self, problem):
        robot = problem.robot
        domain_pddl = read(self.domain)
        stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
        constant_map = {}
        init = []
        arm = ARM_NAMES[0]
        # for arm in problem.arms:
        joints = get_arm_joints(robot, arm)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        init += [('aconf', conf),
                 ('handempty', ),
                 ('ataconf', conf)]

        pose = Pose(problem.red_can, get_pose(problem.red_can))
        init += [('graspable', problem.red_can), ('pose', problem.red_can, pose),
                 ('atpose', problem.red_can, pose), ('target', problem.red_can)]

        blue_can_poses = []
        for surface in problem.surfaces:
            init += [('stackable', problem.red_can, surface)]
            if is_placement(problem.red_can, surface):
                init += [('supported', problem.red_can, pose, surface)]


            for body in problem.blue_cans:
                pose = Pose(body, get_pose(body))
                blue_can_poses.append(pose)
                init += [('graspable', body), ('pose', body, pose),
                         ('atpose', body, pose), ('nontarget', body)]
                for surface in problem.surfaces:
                    init += [('stackable', body, surface)]
                    if is_placement(body, surface):
                        init += [('supported', body, pose, surface)]

        goal = [AND]
        goal += [('holding', problem.red_can)]
        # goal += [('handempty', )]
        # goal += [('notatpose', problem.blue_cans[0], )]

        print(init)
        print(goal)

        stream_map = {
            'sample-pose': from_gen_fn(get_stable_gen(problem)),
            'sample-grasp': from_list_fn(get_grasp_gen(problem, collisions=True, grasp_types = ["top"])),
            'sample-target-grasp': from_list_fn(get_grasp_gen(problem, collisions=True, grasp_types = ["side"])),
            'inverse-kinematics': from_sampler(get_ik_fn(problem, teleport=False)),
            'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(problem)),
        }


        return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,
                init, goal)

    def create_floor():
        return load_pybullet("plane.urdf")

    def post_process(self, problem, init, plan, teleport=False):
        if plan is None:
            return None
        commands = []
        a = problem.arms[0]

        for i, (node) in enumerate(plan):
            if(node.action_name is not None):
                name, args = node.action_name, node.action_args

                if name == 'pick':
                    b, p, g, c = args
                    [t] = c.commands
                    close_gripper = GripperCommand(problem.robot, a, g.grasp_width,
                                                   teleport=teleport)
                    attach = Attach(problem.robot, a, g, b)
                    new_commands = [t, close_gripper, attach, t.reverse()]
                elif name == 'place':
                    b, p, g, c = args
                    [t] = c.commands
                    gripper_joint = get_gripper_joints(problem.robot, a)[0]
                    position = get_max_limit(problem.robot, gripper_joint)
                    open_gripper = GripperCommand(problem.robot, a, position,
                                                  teleport=teleport)
                    detach = Detach(problem.robot, a, b)
                    new_commands = [t, detach, open_gripper, t.reverse()]
                else:
                    raise ValueError(name)
                print(i, name, args, new_commands)
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

    def check_body_collision(self, b1, b2, thresh = 0.12):
        poseb1 = get_pose(b1)[0]
        poseb2 = get_pose(b2)[0]
        return math.sqrt((poseb1[0]-poseb2[0])**2+(poseb1[1]-poseb2[1])**2) <= thresh

    def place_cans_randomly(self, surface, blue=3,
                            table_width=1.2, region_width=0.4):
        random.seed(self.seed+2)
        sample_rad = (table_width/3.5)
        red_can = []

        red_can = load_model(RED_CAN_URDF)

        # x_pos, y_pos = random.uniform(-0.5, 0.5), \
        #     random.uniform(-sample_rad, sample_rad)

        cstable_z = stable_z(red_can, surface)
        red_can_pose = ([0, 0, cstable_z],
                        p.getQuaternionFromEuler([0, 0, 0]))

        set_pose(red_can, red_can_pose)

        maxit=1000
        restart=True
        while(restart):
            restart=False
            blue_cans = []
            blue_can_poses = []
            it=0
            for _ in range(blue):
                in_collision = True
                blue_cans.append(load_model(BLUE_CAN_URDF))

                while (in_collision):
                    it += 1
                    in_collision = False
                    x_pos, y_pos = random.uniform(-0.2, 0.2),\
                        random.uniform(-sample_rad, sample_rad)

                    cstable_z = stable_z(blue_cans[-1], surface)
                    blue_can_poses.append(([x_pos, y_pos, cstable_z],
                                          p.getQuaternionFromEuler([0, 0, 0])))
                    set_pose(blue_cans[-1], blue_can_poses[-1])

                    for existing_can in [red_can]+blue_cans[:-1]:
                        if (self.check_body_collision(existing_can, blue_cans[-1])):
                            in_collision = True
                            break
                    if(it>maxit):
                        restart=True
                        break
                if(it>maxit):
                    for existing_can in blue_cans:
                        remove_body(existing_can)
                        p.stepSimulation()
                    print("Restarting")
                    restart=True
                    break

        for _ in range(5):
            # time.sleep(0.1)
            p.stepSimulation()

        return blue_cans, red_can, blue_can_poses, red_can_pose

    def terminate(self):
        disconnect()

    def get_problem_spec(self, problem=None, vis=False):
        connect(use_gui=vis)

        arm = "left"
        grasp_type = "side"
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

        # floor = p.loadURDF('./models/kitchen/floor/floor.urdf',useFixedBase=True,  globalScaling=0.8)
        # kitchen = p.loadURDF("./models/kitchen/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf",[-5,0,1.477],useFixedBase=True,  globalScaling=0.8)
        # table = p.loadURDF('./models/kitchen/table/table.urdf',[0.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True,  globalScaling=0.8)
        table = create_table(width=0.6, length=1.2, height=0.73,
                             leg_color=GREY, cylinder=False)
        num_blue = 0
        for literal in self.task_problem.init:
            if literal.predicate == "blue":
                num_blue += 1

        initial_bq = Conf(pr2, get_group_joints(pr2, 'base'),
                              get_group_conf(pr2, 'base'))

        restart=True
        while (restart):
            restart=False


            blue_cans, red_can, blue_poses, red_pose = \
                self.place_cans_randomly(table, blue=num_blue)

           

            # Make generators
            problem = ClutterProblemSpec(robot=pr2,
                                      red_can=red_can,
                                      blue_cans=blue_cans,
                                      blue_poses=blue_poses,
                                      red_pose=red_pose,
                                      initial_bq = initial_bq,
                                      movable=[red_can] + blue_cans,
                                      arms=["left"],
                                      grasp_types=[grasp_type],
                                      surfaces=[table],
                                      task_problem=self.task_problem)

            grasp_gen = get_grasp_gen(problem, collisions=True, grasp_types = ["side"])
            ik_gen = get_ik_fn(problem, teleport=False)
            coll_test = get_cfree_traj_pose_test(problem)

            for _ in range(10):
                set_group_positions(pr2, group_positions)
                p.stepSimulation()
                set_group_positions(pr2, group_positions)

            pso = Pose(red_can, get_pose(red_can))
            # Get a grasp
            total_collision = 0
            for (g, ) in grasp_gen(red_can):
                print(red_can, pso, g)
                t=None
                while(t is None):
                    t=ik_gen(red_can, pso, g)
       
                for blue_can in blue_cans:
                    total_collision += 1-int(coll_test(t[0], blue_can, Pose(blue_can, get_pose(blue_can))))

            print(math.floor(len(blue_cans)/3))
            print(total_collision)
            print()
# python genqnp/profile_policy.py --test-problem=Clutter_14  --mode=skeleton --exp-name=clutter_0 --write-external --seed=3 --run-name=realrun10
# python genqnp/profile_policy.py --test-problem=Clutter_14  --mode=skeleton --exp-name=clutter_0 --write-external --seed=5 --run-name=realrun10
# python genqnp/profile_policy.py --test-problem=Clutter_16  --mode=skeleton --exp-name=clutter_0 --write-external --seed=0 --run-name=realrun10
# python genqnp/profile_policy.py --test-problem=Clutter_16  --mode=skeleton --exp-name=clutter_0 --write-external --seed=3 --run-name=realrun10
# python genqnp/profile_policy.py --test-problem=Clutter_16  --mode=skeleton --exp-name=clutter_0 --write-external --seed=5 --run-name=realrun10

            if(total_collision!=max(2, math.floor((len(blue_cans))/3))):
                self.seed += 100
                restart=True
                for can in [red_can] + blue_cans:
                    remove_body(can)

            for _ in range(10):
                set_group_positions(pr2, group_positions)
                p.stepSimulation()
                set_group_positions(pr2, group_positions)

           

        # Get a trajectory from grasp

        # Test trajectory with every object and count intersec
        return problem

