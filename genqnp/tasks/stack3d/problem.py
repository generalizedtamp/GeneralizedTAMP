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
    stable_z, body_collision, get_aabb
from search import solve_mcts
from pddlstream.language.constants import AND, print_solution
from pddlstream.language.generator import from_fn, BoundedGenerator, \
    from_sampler
from pddlstream.language.stream import StreamInfo
from pddlstream.utils import read, INF, get_file_path
from tasks.stack3d.stream_map import get_stream_map
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST
from utils import get_arg_string, create_ycb
from urdf_utils import get_ycb_objects, get_color_blocks, scales, block_heights
import time
import math


BASE_CONSTANT = 1
BASE_VELOCITY = 0.5
MODEL_PATH = './models'
LTAMP_PR2 = os.path.join(MODEL_PATH, 'pr2_description/pr2.urdf')
RED_REGION_URDF = "../models/recycle/red_region.urdf"
BLUE_REGION_URDF = "../models/recycle/blue_region.urdf"


class Stack3dProblemSpec(ProblemSpec):
    def __init__(self, robot, arms=tuple(), movable=tuple(),
                 grasp_types=tuple(),
                 surfaces=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(),
                 goal_cleaned=tuple(), costs=False,
                 table=None,
                 init_regions={},
                 body_names={}, body_types=[], base_limits=None,
                 task_problem=None,
                 blocks=[],
                 poses=[]):
        self.robot = robot
        self.arms = arms
        self.blocks = blocks
        self.poses = poses
        self.movable = movable
        self.init_regions = init_regions
        self.table = table
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
        super(Stack3dProblemSpec, self).__init__()
        self.skeleton = []

        for _ in range(len(blocks)):
            self.skeleton.append(('pick', {}))
            self.skeleton.append(('place', {}))

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


class Stack3d(GenTask):

    def __init__(self, *args, **kwargs):
        super(Stack3d, self).__init__(*args, **kwargs)

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

        body_poses = {}

        for body in spec.blocks:
            pose = Pose(body, get_pose(body))
            body_poses[body] = pose
            init += [('graspable', body), ('pose', body, pose),
                     ('atpose', body, pose)]

            init += [('stackable', body, spec.table)]
            init += [('supported', body, pose, spec.init_regions[body])]

        # for body1 in spec.blocks:
        #     for body2 in spec.blocks:
        #         if(body1 != body2):
        #             init+=[('stackable', body1, body2)]


        init += [('table', spec.table)]

        goal = [AND]

        for block in spec.blocks:
            goal += [('on', block, spec.table)]

        # goal+=[('holding', spec.blocks[0])]
    
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

    def get_num_blocks(self):
        block_counter = 0
        for obj in self.task_problem.objects:
            if(obj.name.startswith("obj")):
                block_counter+=1
        return block_counter

    def check_body_collision(self, b1, b2, thresh = 0.15):
        poseb1 = get_pose(b1)[0]
        poseb2 = get_pose(b2)[0]
        return math.sqrt((poseb1[0]-poseb2[0])**2+(poseb1[1]-poseb2[1])**2) <= thresh

    def place_new_block(self, blocks, new_urdf, surface, surface_upper, table_width=0.8, region_width=0.4):
        sample_rad = (table_width - 2 * region_width) / 2.0
        new_block = create_ycb(new_urdf)
        in_collision = True
        while (in_collision):
            in_collision = False
            x_pos, y_pos = random.uniform(-0.2, 0.2), \
                random.uniform(0, table_width/2.0)

            if(new_urdf in block_heights):
                new_object_height = block_heights[new_urdf]
            else:
                new_object_height = get_aabb(new_block).upper[2]-get_aabb(new_block).lower[2]
            cstable_z = surface_upper + new_object_height/2.0
            new_pose = ((x_pos, y_pos, cstable_z), p.getQuaternionFromEuler([0, 0, 0]))
            set_pose(new_block, new_pose)
            for existing_block in blocks:
                if (self.check_body_collision(existing_block, new_block)):
                    in_collision = True
                    break

        return new_block


    def get_problem_spec(self, vis=False, problem=None):
        connect(use_gui=vis)
        random.seed(self.seed+1)

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

        floor = p.loadURDF('./models/kitchen/floor/floor.urdf',useFixedBase=True,  globalScaling=0.8)
        # kitchen = p.loadURDF("./models/kitchen/kitchen_description/urdf/kitchen_part_right_gen_convex.urdf",[-5,0,1.477],useFixedBase=True,  globalScaling=0.8)
        table = p.loadURDF('./models/kitchen/table/table.urdf',[0.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True,  globalScaling=0.8)


        for _ in range(5):
            p.stepSimulation()
            # time.sleep(0.1)

        num_blocks = self.get_num_blocks()
        # blocks, poses = \
        #     self.place_blocks_randomly(table, num_blocks=self.get_num_blocks())

        mode = "plain"
        if(mode == "plain"):
            color_cubes = get_color_blocks()
        elif(mode == "ycb"):
            color_cubes = get_ycb_objects()

        # random.shuffle(color_cubes)
        poses = []
        removal_order = []
        gr = [table]
        total_regions = [(table, 0)]
        on_tuples = []
        blocks = []

        body_urdf_map = {}
        for b in range(num_blocks):
            ccb_urdf = color_cubes[b%len(color_cubes)]
            (place_region, place_height) = total_regions.pop(0)
            while(place_height >= 3):
                place_region, place_height = total_regions.pop(0)

            if (place_region == table):
                total_regions.append((table, 0))              
                surface_upper  =  get_aabb(table).upper[2]                
                new_block = self.place_new_block(blocks, ccb_urdf, table, surface_upper)
                on_tuples.append((new_block, table))
                blocks.append(new_block)
            else:
                new_block = create_ycb(color_cubes[b%len(color_cubes)])
                if(body_urdf_map[place_region] in block_heights):
                    surface_height = block_heights[body_urdf_map[place_region]]
                else:
                    surface_height = get_aabb(place_region).upper[2]-get_aabb(place_region).lower[2]

                if(ccb_urdf in block_heights):
                    body_height = block_heights[ccb_urdf]
                else:
                    body_height = get_aabb(new_block).upper[2]-get_aabb(new_block).lower[2]
                
                place_pose = get_pose(place_region)
                set_pose(new_block, ((place_pose[0][0], place_pose[0][1], place_pose[0][2]+surface_height/2.0+body_height/2.0 ), place_pose[1]))
                blocks.append(new_block)
                on_tuples.append((new_block, place_region))
                removal_order.append(new_block)

            body_urdf_map[new_block] = ccb_urdf
            total_regions.append((new_block, place_height+1))
            poses.append(get_pose(new_block))

        init_regions = {block: table for block in blocks}
        for on_block, on_region in on_tuples:
            init_regions[on_block] = on_region


        # Step in simulation to set everything up
        for _ in range(5):
            p.stepSimulation()
            # time.sleep(0.1)

        return Stack3dProblemSpec(robot = pr2,
                                  blocks = blocks,
                                  poses = poses,
                                  movable = blocks,
                                  init_regions = init_regions,
                                  arms = ["left"],
                                  grasp_types = [grasp_type],
                                  surfaces = [table],
                                  table = table,
                                  task_problem = self.task_problem)

