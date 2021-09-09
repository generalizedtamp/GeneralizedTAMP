#!/usr/bin/env python

from __future__ import print_function

import math
import random
from collections import namedtuple
import pybullet as p
from tasks.carry.streams import get_compute_pose_kin,\
    get_stable_gen, get_ik_ir_gen, get_motion_gen, test_cfree,\
    create_relative_pose, Conf, get_grasp_gen
from tasks.gentask import GenTask, ProblemSpec
from utils import CloseAttach, CloseDetach, RelPose, \
    create_bucket
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives \
    import Attach, Detach, get_gripper_joints, GripperCommand, apply_commands, State
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import \
    set_arm_conf, REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf,\
    PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import connect,\
    is_placement, disconnect, user_input, get_joint_positions, HideOutput,\
    LockRenderer, get_max_limit, load_model, load_pybullet, create_box,\
    set_point, get_bodies, create_plane, TAN, stable_z, body_collision, get_pose, set_pose
from pddlstream.language.constants import AND, print_solution
from pddlstream.language.external import defer_shared
from pddlstream.language.generator import from_gen_fn, from_list_fn,\
     from_fn, from_test
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.utils import read, INF, get_file_path
from pddlstream.language.constants import And, Equal, PDDLProblem, TOTAL_COST
import time
from utils import get_arg_string
from collections import defaultdict


class CarryProblemSpec(ProblemSpec):
    def __init__(self, robot, arms=tuple(), movable=tuple(), cans=tuple(),
                 grasp_types=tuple(),
                 surfaces=tuple(), sinks=tuple(), stoves=tuple(),
                 buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(), goal_on=tuple(),
                 goal_cleaned=tuple(), blocks=tuple(), costs=False, cost=INF,
                 body_names={}, body_types=[], object_types={},
                 base_limits=None, tray=None,
                 num_slots=0):
        self.robot = robot
        self.arms = arms
        self.movable = movable
        self.cost = cost
        self.grasp_types = grasp_types
        self.cans = cans
        self.surfaces = surfaces
        self.sinks = sinks
        self.stoves = stoves
        self.buttons = buttons
        self.num_slots = num_slots
        self.goal_conf = goal_conf
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cleaned = goal_cleaned
        self.blocks = blocks
        self.tray = tray
        self.costs = costs
        self.body_names = body_names
        self.object_types = object_types

        block_index = 0
        self.skeleton = []

        # For quicky collecting training data 
        while True:
            carrying_blocks =[]
            for slot_index in range(self.num_slots):
                self.skeleton.append(("pick_at_stove", {"?o": str(self.cans[block_index])}))
                self.skeleton.append(("place_at_stove", {"?o": str(self.cans[block_index])}))
                carrying_blocks.append(self.cans[block_index])
                block_index += 1
                if(block_index == len(self.cans)):
                    break
            self.skeleton.append(("pick_at_stove", {"?o": str(self.tray)}))
            self.skeleton.append(("move_to_sink", {}))
            self.skeleton.append(("place_at_sink", {"?o": str(self.tray)}))
            for carrying_block in carrying_blocks:
                self.skeleton.append(("pick_at_sink", {"?o": str(carrying_block)}))
                self.skeleton.append(("place_at_sink", {"?o": str(carrying_block)}))

            self.skeleton.append(("pick_at_sink", {"?o": str(self.tray)}))
            self.skeleton.append(("move_to_stove", {}))
            self.skeleton.append(("place_at_stove", {"?o": str(self.tray)}))
            if(block_index == len(self.cans)):
                break


        self.nonfluents = [ "stackable", "sink", "stove", "grasp", "posekin",
                            "computedposekin", "basemotion", "armmotion", "worldposebasestove", "worldposebasesink",
                            "worldpose", "relpose", "can", "unsafeworldpose", "cfree", "btraj", "atstove", "atsink", 
                            "bconf", "graspable", "atraj"]

        self.base_limits = base_limits
        all_movable = [self.robot] + list(self.movable)
        self.fixed = list(filter(lambda b: b not in all_movable, get_bodies()))
        self.gripper = None

    def get_init_baseconf(self, inits):
        for init in inits:
            if (init[0] == "atbconf"):
                return init[1]


    def get_gripper(self, arm='left'):
        # upper = get_max_limit(problem.robot, get_gripper_joints(problem.robot, 'left')[0])
        # set_configuration(gripper, [0]*4)
        # dump_body(gripper)
        if self.gripper is None:
            self.gripper = create_gripper(self.robot, arm=arm)
        return self.gripper

    def __repr__(self):
        return repr(self.__dict__)


def create_pr2(use_drake=True, fixed_base=True, torso=0.2):
    pr2_path = DRAKE_PR2_URDF if use_drake else PR2_URDF
    with LockRenderer():
        with HideOutput():
            pr2 = load_model(pr2_path, fixed_base=fixed_base)
        set_group_conf(pr2, 'torso', [torso])
    return pr2


def create_floor():
    return load_pybullet("plane.urdf")


#######################################################

CustomValue = namedtuple('OptValue', ['stream', 'values'])


def opt_pose_fn(o, r, rp):
    p = CustomValue('p-sp', (r,))
    return p,


def opt_ik_fn(a, o, p, r, g):
    q = CustomValue('q-ik', (p,))
    t = CustomValue('t-ik', tuple())
    return q, t


def opt_motion_fn(q1, q2):
    t = CustomValue('t-pbm', (q1, q2))
    return t,


class Carry(GenTask):


    def __init__(self, *args, **kwargs):
        super(Carry, self).__init__(*args, **kwargs)

        self.action_costs = {
            "move_to_sink": 100,
            "move_to_stove": 100
        }

    def check_body_collision(self, b1, b2, thresh = 0.1):
        poseb1 = b1[0]
        poseb2 = b2[0]
        return math.sqrt((poseb1[0]-poseb2[0])**2+(poseb1[1]-poseb2[1])**2) <= thresh


    def terminate(self):
        disconnect()

    def pddlstream_from_spec(self, problem, teleport=False):
        robot = problem.robot

        domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
        stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
        constant_map = {}
        init = []
        tray_gen = get_stable_gen(problem)
        

        block_index_counter = 0

        # Create the base of the stack of relposes
        surface_relpose = {}
        for sink in problem.sinks:
            # surface_relpose[sink] = RelPose(sink, None, get_link_pose(problem.kitchen, sink))
            surface_relpose[sink] = RelPose(sink, None, get_pose(sink))
            init += [('worldpose', sink, surface_relpose[sink]), 
                     ('atworldpose', sink, surface_relpose[sink]),
                     ('worldposebasesink', surface_relpose[sink])]
            break
        for stove in problem.stoves:
            surface_relpose[stove] = RelPose(stove, None, get_pose(stove))
            init += [('worldpose', stove, surface_relpose[stove]), 
                     ('atworldpose', stove, surface_relpose[stove]),
                     ('worldposebasestove', surface_relpose[stove])]
            break

        tray_gen = get_stable_gen(problem)
        compute_kin = get_compute_pose_kin(problem)
   
        tray_relpose = None
        for surf in problem.sinks:
            obj_pose = get_pose(problem.tray)
            for (relpose,) in tray_gen(problem.tray, surf):

                wp, = compute_kin(problem.tray, relpose, surf, surface_relpose[surf])
                init += [('relpose', problem.tray, relpose, surf),
                         # ('worldpose', problem.tray, wp),
                         # ('computedposekin', problem.tray, wp, relpose, surf, surface_relpose[surf]),
                         # ('worldposebasesink', wp)
                         ]
                tray_relpose = relpose

                set_pose(problem.tray, obj_pose)
                break


        for surf in problem.sinks:
            it = 0
            maxit = 1000
            restart=True
            while(restart):
                sink_block_relposes = []
                restart=False
                for obj in problem.blocks:
                    obj_pose = get_pose(obj)
                    incollision=True
                    while(incollision):
                        it+=1
                        print(it)
                        for (relpose,) in tray_gen(obj, surf):
                            incollision=False
                            for old_relpose in sink_block_relposes:
                                if(self.check_body_collision(relpose.rel_pose, old_relpose.rel_pose)):
                                    incollision=True

                            if(self.check_body_collision(relpose.rel_pose, tray_relpose.rel_pose, thresh=0.15)):
                                incollision = True

                            if(not incollision):
                                init += [('relpose', obj, relpose, surf)]
                                sink_block_relposes.append(relpose)
                                set_pose(obj, obj_pose)
                            break

                        if(it>=maxit):
                            it=0
                            restart=True
                            break
                    if(it>=maxit):
                        it=0
                        restart=True
                        break


        tray_slot_rel_poses = []
        obj_pose = get_pose(problem.blocks[0])
        for _ in range(problem.num_slots):
            incollision = True
            while(incollision):
                for (relpose,) in tray_gen(problem.blocks[0], problem.tray):
                    incollision = False
                    for old_relpose in tray_slot_rel_poses:
                        if(self.check_body_collision(relpose.rel_pose, old_relpose)):
                            incollision = True
                    if(not incollision):
                        tray_slot_rel_poses.append(relpose.rel_pose)
                    break


        set_pose(problem.blocks[0], obj_pose)

        for surf in [problem.tray]:
            while(block_index_counter<len(problem.blocks)):
                for rel_pose in tray_slot_rel_poses:
                    relpose = RelPose(problem.blocks[block_index_counter], problem.tray, rel_pose)
                    init += [('relpose', problem.blocks[block_index_counter], relpose, surf), ('tray_relpose', problem.blocks[block_index_counter], relpose)]
                    block_index_counter+=1
                    if(block_index_counter == len(problem.blocks)):
                        break

        # Initial robot base settings
        initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
        init += [
                   ('bconf', initial_bq),
                   ('atstove', initial_bq),
                   ('atbconf', initial_bq)
               ] + [('sink', s) for s in problem.sinks] + [('stove', s) for s in problem.stoves]\
                 + [('table', s) for s in problem.sinks] + [('table', s) for s in problem.stoves]\
                 + [('tray', problem.tray)]

        # Set the arms as controllable and set their current configuration
        for arm in ARM_NAMES:
            joints = get_arm_joints(robot, arm)
            conf = Conf(robot, joints, get_joint_positions(robot, joints))
            init += [('aconf', conf), ('handempty', ), ('ataconf', conf)]
            if arm in problem.arms:
                init += [('controllable', )]

        # Every movable body is graspable
        for body in problem.movable:
            init += [('graspable', body)]

        stove_can_relposes = []
        for body in problem.movable:
            for surface in problem.surfaces:
                if (body != surface):
                    init += [('stackable', body, surface)]
                    print(body, surface)
                    if is_placement(body, surface, below_epsilon=1e-2):
                        print(is_placement)
                        relpose = create_relative_pose(body, surface)
                        wp, = compute_kin(body, relpose, surface, surface_relpose[surface])
                        if(surface in problem.stoves and body in problem.blocks):
                            stove_can_relposes.append(relpose)
                        init += [('relpose', body, relpose, surface), 
                                 ('atrelpose', body, relpose),
                                 ('worldpose', body, wp), 
                                 ('atworldpose', body, wp),
                                 ('computedposekin', body, wp, relpose, surface, surface_relpose[surface])
                                 ]
                        if (surface == problem.sinks[0]):
                            init += [('worldposebasesink', wp)]
                        elif (surface == problem.stoves[0]):
                            init += [('worldposebasestove', wp)]


        tray_gen = get_stable_gen(problem)
        compute_kin = get_compute_pose_kin(problem)
        # for _ in range(problem.num_slots):
        for surf in problem.stoves:
            obj_pose = get_pose(problem.tray)
            in_collision = True
            while (in_collision): 
                in_collision = False   
                for (relpose,) in tray_gen(problem.tray, surf):
                    for old_relpose in stove_can_relposes:
                        if(self.check_body_collision(relpose.rel_pose, old_relpose.rel_pose, thresh=0.15)):
                            in_collision = True
                    if(not in_collision):
                        tray_stove_relpose = relpose
                    break

        wp, = compute_kin(problem.tray, tray_stove_relpose, surf, surface_relpose[surf])
        init += [('relpose', problem.tray, tray_stove_relpose, surf),
                 # ('worldpose', problem.tray, wp),
                 # ('computedposekin', problem.tray, wp, tray_stove_relpose, surf, surface_relpose[surf]),
                 # ('worldposebasestove', wp)
                 ]

        set_pose(problem.tray, obj_pose)
        # Set up the goal
        goal = [AND]

        # print("Tray: "+str(problem.tray))
        for can in problem.cans:
            goal += [('on', can, problem.sinks[0])]

        defer_fn = defer_shared

 
        stream_map = {
            # 'sample-pose': from_gen_fn(get_stable_gen(problem)),
            'sample-grasp': from_list_fn(get_grasp_gen(problem)),
            'inverse-kinematics-stove': from_gen_fn(
                get_ik_ir_gen(problem,
                              teleport=teleport,
                              semantic_surface=problem.stoves[0])),
            "inverse-kinematics-sink": from_gen_fn(
                get_ik_ir_gen(problem,
                              teleport=teleport,
                              semantic_surface=problem.sinks[0])),
            'compute-kin-stove': from_fn(get_compute_pose_kin(problem)),
            'compute-kin-sink': from_fn(get_compute_pose_kin(problem)),
            'test-cfree': from_test(test_cfree(problem)),
        }

        # get_press_gen(problem, teleport=teleport)
        print("Init: ", init)

        return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

    def place_cans(self, w=.8, h=.8, num_cans=2, ):
        random.seed(self.seed+3)
        w = 1.0
        create_plane(color=TAN)
        can_width = 0.25
        colors = [
            [64 / 255., 224 / 255., 208 / 255., 1],
            [255 / 255., 191 / 255., 0 / 255., 1],
            [255 / 255., 127 / 255., 80 / 255., 1],
            [222 / 255., 49 / 255., 99 / 255., 1],
            [159 / 255., 226 / 255., 191 / 255., 1],
            [204 / 255., 204 / 255., 255 / 255., 1],
        ]
        r = random.randint(0, 5)
        table = create_box(w, w, h, color=colors[r])
        tp = random.choice([2, -2])
        table_point = ((tp), 0, h / 2)
        set_point(table, table_point)

        mass = 1
        # mass = 0.01
        # mass = 1e-6

        existing_cans = []
        tw = 0.4
        tray = create_bucket(tw, tw, 0.05)

        rw = w/2 - can_width/2.0
        margin = 0.01
        tray_offset = -w / 2.0 + tw / 2.0 + margin
        set_point(tray, (table_point[0] + tray_offset, tray_offset,
                  stable_z(tray, table)))

        it = 0
        maxit = 1000
        restart=True
        while(restart):
            restart = False
            for i in range(num_cans):
                existing_can = create_box(.06, .06, .1, mass=mass,
                                          color=(0, 1, 0, 1))
                in_collision = True
                while (in_collision):
                    it+=1
                    in_collision = False
                    tp1 = random.uniform(-rw, rw)
                    tp2 = random.uniform(-rw, rw)
                    existing_can_pose = (table_point[0] +
                                         tp1,
                                         tp2,
                                         stable_z(existing_can, table))
                    set_point(existing_can, existing_can_pose)
                    for can in existing_cans + [tray]:
                        in_collision = in_collision or \
                                        self.check_body_collision(get_pose(existing_can), get_pose(can))
                    in_collision = in_collision or \
                        self.check_body_collision(get_pose(tray), get_pose(existing_can), thresh=tw)

                    if(it > maxit):
                        it=0
                        break

                existing_cans.append(existing_can)
                if(it > maxit):
                    restart=True
                    print("Restarting...")
                    for existing_can in existing_cans:
                        remove_body(existing_can)
                    break
        print()
        print("Done")

        r = random.randint(0, 5)
        w = random.uniform(0.6, 0.9)
        sink = create_box(w, w, h, color=colors[r])
        ss = random.choice([2, -2])
        set_point(sink, (0, ss, h / 2))
        
        for i in range(5):
            # time.sleep(0.1)  
            p.stepSimulation()

        return table, existing_cans, sink, tray

    def is_world_pose(self, arg):
        return isinstance(arg, RelPose) and (not arg.parent_body is None)

    def post_process(self, problem, init, plan, teleport=False):
        motion_gen = get_motion_gen(problem, teleport=teleport)
        world_poses = {}
        for evaluation in init:
            args = evaluation[1:]
            for arg in args:
                if self.is_world_pose(arg):
                    world_poses[str(arg)] = arg.get_world_from_body()

        commands = []
        a = problem.arms[0]
        for i, (node) in enumerate(plan):
            if(node.action_name is not None):
                name, args = node.action_name, node.action_args
                if  name == "move_to_sink" or name == "move_to_stove":
                    q1, q2 = args
                    [tb] = motion_gen(q1, q2)
                    new_commands = tb.commands

                elif name == 'pick_at_stove' or name == 'pick_at_sink':
                    o, wp, _, _, _, g, q, c, q2 = args
                    
                    world_poses[str(wp)] = wp.get_world_from_body()
                    [t] = c.commands
                    close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
                    attach = Attach(problem.robot, a, g, o)
                    close_attach = CloseAttach(problem.robot, a, g, problem.movable, o)
                    [tb] = motion_gen(q, q2)
                    new_commands = [tb, t, close_gripper, attach, close_attach, t.reverse()]

                elif name == 'place_at_stove' or name == 'place_at_sink':
                    o, wp, _, _, _, g, q, c, q2 = args
                    world_poses[str(wp)] = wp.get_world_from_body()

                    [t] = c.commands
                    gripper_joint = get_gripper_joints(problem.robot, a)[0]
                    position = get_max_limit(problem.robot, gripper_joint)
                    open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
                    detach = Detach(problem.robot, a, o)

                    # Search for all objects that are within a certain radius
                    close_detach = CloseDetach(problem.robot, a, g, problem.movable, o)
                    [tb] = motion_gen(q, q2)
                    new_commands = [tb, t, detach, open_gripper, close_detach, t.reverse()]
                else:
                    raise ValueError(name)
                print(i, name, args, new_commands)
                commands += new_commands

        problem.remove_gripper()
        print("World Poses", world_poses)
        return commands, world_poses

    def get_problem_spec(self, vis=False):
        connect(use_gui=vis)

        # Load in the problem file
        num_cans = 0
        num_slots = 0
        for obj in self.task_problem.objects:
            if (obj.name[:3] == "obj"):
                num_cans += 1
            elif (obj.name[:4] == "slot"):
                num_slots += 1

        arm = "left"
        grasp_type = "top"
        other_arm = get_other_arm(arm)
        initial_conf = get_carry_conf(arm, grasp_type)

        pr2 = create_pr2()
        set_arm_conf(pr2, arm, initial_conf)
        open_arm(pr2, arm)
        set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
        close_arm(pr2, other_arm)

        num_trips = math.ceil(num_cans / num_slots)
        cost = (num_trips * 2 - 1) * 100 + 150

        table, cans, sink, tray = self.place_cans(num_cans=num_cans)
        movable = cans + [tray]

        object_types = {}
        for k, v in zip(movable, ["box" for _ in cans] + ["tray"]):
            object_types[k] = v

        goal_on = [(c, sink) for c in cans]

        time.sleep(0.1)
        p.stepSimulation()
        return CarryProblemSpec(robot=pr2, movable=movable, cans=cans,
                                goal_on=goal_on, arms=[arm],
                                grasp_types=[grasp_type],
                                surfaces=[table, sink, tray], sinks=[sink],
                                stoves=[table], object_types=object_types,
                                blocks=cans, tray=tray, cost=cost,
                                num_slots=num_slots)

    def compile_plan(self, tplan):
        raise NotImplementedError