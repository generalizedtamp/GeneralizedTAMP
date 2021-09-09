import yaml
import os
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import invert, \
    is_placement, Euler, multiply, unit_quat, get_box_geometry, STATIC_MASS,\
    BROWN, unit_pose, create_shape_array, create_body, Point, \
    quat_from_euler, create_attachment, link_from_name, BASE_LINK, \
    get_link_pose, get_pose, set_pose, get_mesh_geometry, create_collision_shape, \
    create_visual_shape, CLIENT

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import \
    Pose, Conf, get_grasp_gen, Attach, Detach, \
    get_gripper_joints, GripperCommand, apply_commands, State

from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_problems import \
    create_table
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import \
    rightarm_from_leftarm, set_arm_conf, \
    REST_LEFT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm, set_group_conf,\
    PR2_URDF, DRAKE_PR2_URDF, create_gripper, \
    PR2_GROUPS, set_joint_positions, get_arm_joints, ARM_NAMES, \
    get_group_joints, get_group_conf, GET_GRASPS, TOP_HOLDING_LEFT_ARM, \
    compute_grasp_width
import random
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import set_pose,\
    connect, get_pose, is_placement, \
    disconnect, user_input, get_joint_positions, \
    HideOutput, get_distance, LockRenderer, get_max_limit, load_model, \
    load_pybullet, get_bodies, create_plane, TAN, GREY, PI, joints_from_names,\
    stable_z, body_collision, set_default_camera, WHITE, get_unit_vector, apply_affine, \
    implies, get_num_links, vertices_from_link, OBJ_MESH_CACHE, read_obj, aabb_from_points, \
    get_aabb_center, get_aabb_extent, point_from_pose, mesh_from_points, create_mesh, set_texture,\
    create_obj

import math

import pybullet as p
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_motion_gen, Conf, Attach, Detach, Clean, \
    Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, APPROACH_DISTANCE, Grasp, get_grasp_gen, GRASP_LENGTH, \
    iterate_approach_path, Commands, Command

from itertools import chain
from sympy import symbols, simplify_logic
from sympy import Not as SymNot
from sympy import Or as SymOr
from sympy import And as SymAnd
import trimesh
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_utils import \
    PR2_TOOL_FRAMES, TOOL_POSE,MAX_GRASP_WIDTH
from pddlstream.language.object import Object
from urdf_utils import scales
from collections import namedtuple
from pddl import Atom, NegatedAtom, Conjunction, UniversalCondition, \
    ExistentialCondition, TypedObject, Disjunction, Effect, conditions, Action
import pickle
import numpy as np
from pyeda.inter import Or, Not, And, exprvar
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import get_motion_gen, Conf, Pose, Grasp, Commands, Trajectory
import time
import pyswip
import itertools
import errno
import os
import signal
import functools
from minio import Minio
import shutil

from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import Pose as Pose2

GenTAMPProblem = namedtuple("GenTAMPProblem",
                            ["features", "quantified_concept_map", "complexity_map"])


Domain = namedtuple('Domain', ['name', 'requirements', 'types', 'type_dict',
                               'constants', 'predicates', 'predicate_dict',
                               'functions', 'actions', 'axioms'])

Feature = namedtuple('Feature', ['concept', 'free_arg', 'uni_set', 'ext_set'])

BOOLEAN = 0
NUMERIC = 1

sign = lambda x: -1 if x < 0 else (1 if x > 0 else 0)

class QualitativeAction(Action):
    def __init__(self, source=None, decs = [], incs = [], order=[], *args, **kwargs):
        if(source is not None):
            self.__dict__.update(source.__dict__)
        self.decs = decs
        self.incs = incs
        self.order = order


class SplitTrajectory(Command):
    _draw = False
    def __init__(self, approach_path, grasp_path):
        self.approach_path = tuple(approach_path)
        self.grasp_path = tuple(grasp_path)
        self.path = tuple(list(self.approach_path )+list(self.grasp_path))
        # TODO: constructor that takes in this info
    def apply(self, state, sample=1):
        handles = add_segments(self.to_points()) if self._draw and has_gui() else []
        for conf in self.path[::sample]:
            conf.assign()
            yield
        end_conf = self.path[-1]
        if isinstance(end_conf, Pose):
            state.poses[end_conf.body] = end_conf
        for handle in handles:
            remove_debug(handle)
    def control(self, dt=0, **kwargs):
        # TODO: just waypoints
        for conf in self.path:
            if isinstance(conf, Pose):
                conf = conf.to_base_conf()
            for _ in joint_controller_hold(conf.body, conf.joints, conf.values):
                step_simulation()
                time.sleep(dt)
    def to_points(self, link=BASE_LINK):
        # TODO: this is computationally expensive
        points = []
        for conf in self.path:
            with BodySaver(conf.body):
                conf.assign()
                #point = np.array(point_from_pose(get_link_pose(conf.body, link)))
                point = np.array(get_group_conf(conf.body, 'base'))
                point[2] = 0
                point += 1e-2*np.array([0, 0, 1])
                if not (points and np.allclose(points[-1], point, atol=1e-3, rtol=0)):
                    points.append(point)
        points = get_target_path(self)
        return waypoints_from_path(points)
    def distance(self, distance_fn=get_distance):
        total = 0.
        for q1, q2 in zip(self.path, self.path[1:]):
            total += distance_fn(q1.values, q2.values)
        return total
    def iterate(self):
        for conf in self.path:
            yield conf
    def reverse(self):
        return Trajectory(reversed(self.path))
    #def __repr__(self):
    #    return 't{}'.format(id(self) % 1000)
    def __repr__(self):
        d = 0
        if self.path:
            conf = self.path[0]
            d = 3 if isinstance(conf, Pose) else len(conf.joints)
        return 't({},{})'.format(d, len(self.path))

def create_trajectory(robot, joints, approach_path, grasp_path):
    return SplitTrajectory( (Conf(robot, joints, q) for q in approach_path), (Conf(robot, joints, q) for q in grasp_path))

class Implication():
    def __init__(self, body, head):
        self.body = body
        self.head = head
        self.parts = tuple([body, head])
        self.hash = hash((self.__class__, self.parts))

    def __hash__(self):
        return self.hash


def get_query(concept, transition, depth=0, nf_pred_extension=0, nonfluents=[]):
    if(isinstance(concept, UniversalCondition)):
        bound_args = []
        forall_lists = []
        while(isinstance(concept, UniversalCondition)):
            for bound_arg in  concept.parameters:
                bound_args.append(bound_arg)
                forall_lists.append(["'"+np_print_repr(get_arg_string(to))+"'" for to in transition.typed_objects[bound_arg.type_name]])
            concept = concept.parts[0]

        inner_query = get_query(concept,
                                transition, 
                                nf_pred_extension = nf_pred_extension, 
                                depth = depth+1, 
                                nonfluents = nonfluents)
        bound_vars = [bound_arg.name.replace("?", "_") for bound_arg in bound_args]
        forall_list_strs = [", ".join(forall_list) for forall_list in forall_lists]
        member_strs = ["member({}, [{}])".format(bound_var, forall_list_str) for (bound_var, forall_list_str) in zip(bound_vars, forall_list_strs)]
        return "forall(({}), ({}) )".format(",".join(member_strs), inner_query)
    elif(isinstance(concept, ExistentialCondition)):
        bound_args = []
        forall_lists = []
        while(isinstance(concept, ExistentialCondition)):
            for bound_arg in  concept.parameters: 
                bound_args.append(bound_arg)
                forall_lists.append(["'"+np_print_repr(get_arg_string(to))+"'" for to in transition.typed_objects[bound_arg.type_name]])
            concept = concept.parts[0]

        inner_query = get_query(concept, 
                                transition, 
                                nf_pred_extension = nf_pred_extension, 
                                depth = depth+1, 
                                nonfluents = nonfluents)

        bound_vars = [bound_arg.name.replace("?", "_") for bound_arg in bound_args]
        forall_list_strs = [", ".join(forall_list) for forall_list in forall_lists]
        member_strs = ["member({}, [{}])".format(bound_var, forall_list_str) for (bound_var, forall_list_str) in zip(bound_vars, forall_list_strs)]
        bound_vars_tuple_str = "({})".format(",".join(bound_vars))
        return "findall({}, ({}, {}), Results{}), length(Results{}, N), N > 0".format(bound_vars_tuple_str, 
                                                                                      ",".join(member_strs),
                                                                                      inner_query, 
                                                                                      depth, 
                                                                                      depth)
    else:
        ext = "_{}".format(transition.index)
        return convert_to_prolog(concept, pred_extension = ext, nf_pred_extension=nf_pred_extension, nonfluents=nonfluents)
        
def infer_object_type(task_domain, obj, states):
    for inits in states:
        for init in inits:
            for ai, arg in enumerate(init.args):
                if(arg == obj):
                    # Search for predicate in task domain
                    for predicate in task_domain.predicates:
                        if(predicate.name == init.predicate):
                            return predicate.arguments[ai].type_name

def format_pl_statement(atom, index):
    if(isinstance(atom, tuple)):
        arg_strings = ["'{}'".format(a) for a in atom[1:]]
        formatted_pl = "{}_{}({}).".format(atom[0], index, ", ".join(arg_strings))
        return formatted_pl.replace(" ", "")
    else:
        arg_strings = []
        for a in atom.args:
            arg_strings.append("'"+str(a.name).lower()+"'")
        formatted_pl = "{}_{}({}).".format(atom.predicate, index, ", ".join(arg_strings))
        return formatted_pl.replace(" ", "")

def findall_query_raw(free_args, transition, base_concept, kb, nf_pred_extension=0, nonfluents=[], verbose=False):
    
    if(verbose):
        print("Querying")
        print(print_concept(base_concept))
    results = []
    findall_list = []
    valmap = {}
    for to_tup in itertools.product(*[transition.typed_objects[free_arg.type_name] for free_arg in free_args]):
        for to in to_tup:
            valmap[to] = np_print_repr( get_arg_string(to) ).replace(" ", "")
        findall_list.append("({})".format(",".join(["'{}'".format(valmap[to]) for to in to_tup])))

    free_strs = [free_arg.name.replace("?", "_") for free_arg in free_args]
    free_vars = "({})".format(",".join(free_strs))
    bq = get_query(base_concept, transition, nf_pred_extension=nf_pred_extension, nonfluents=nonfluents)
    findall_list_str = ",".join(findall_list)
    if(len(free_args)>0):
        query = "findall({}, (member({}, [{}]), {}), Results).".format(free_vars, free_vars, findall_list_str, bq)
        if(verbose):
            print(query)
        ress = list(kb.query(query))
        for out in ress:
            for result in out['Results']:
                if(len(free_strs)>1):
                    results.append(result[2:-1].split(", "))
                else:
                    results.append([result])
        if(verbose):
            print("Results")
            print(results)
        return results, {v:k for k, v in valmap.items()}
    else:
        ress = list(kb.query("{}.".format(bq)))
        for out in ress:
            if('Results0' in out.keys()):
                return len(out['Results0'])>0, {}
            else:
                return True, {}
        return False, {}

def is_continuous(obj_str):
    return ("[" in obj_str and "]" in obj_str) or ("(" in obj_str and ")" in obj_str)

def create_kb(kb_lines):

    """
        We encode everything into one knowledge base and differentiate plans/states using
        different predicates for different plans states predicate_planidx_transitionidx
    """
    if(len(kb_lines)>0):
        kb_filename = './temp/'+str(time.time())+'.pl' 
        with open(kb_filename, 'w') as h:
            for line in kb_lines:
                h.write(line)

        kb = pyswip.Prolog()
        kb.consult(kb_filename)
    else:
        kb = pyswip.Prolog()
    return kb

def clear_temp():
    dirpath = "./temp"
    if os.path.exists(dirpath) and os.path.isdir(dirpath):
        shutil.rmtree(dirpath)
    os.mkdir(dirpath)

def populate_state_prolog(task_domain, index, encode_nonfluents=False, nf_pred_extension=0):

    """
        This function introduces dynamic variables for every possible predicate
        to avoid prolog errors when a predicate doesn't exist in a state
    """
    kb_lines = []
    for predicate in task_domain.predicates:
        numargs = len(predicate.arguments)
        if(predicate.name.islower() and "-" not in predicate.name):
            if(encode_nonfluents):
                kb_lines.append(":- dynamic({}_{}/{}).\n".format(predicate.name, nf_pred_extension, numargs))
            kb_lines.append(":- dynamic({}_{}/{}).\n".format(predicate.name, index, numargs))
    return kb_lines

def get_atom_head(atom):
    if(isinstance(atom, Atom) or isinstance(atom, NegatedAtom)):
        return atom.predicate
    else:
        return atom[0]

def encode_state_kb(state, index, nonfluents=[], encode_nonfluents=True, nf_pred_extension=0):
    # print(state)
    # Open the file
    kb_lines = []
    pl_string = []
    for atom in state:
        if(get_atom_head(atom) not in nonfluents or encode_nonfluents):
            if(get_atom_head(atom) in nonfluents):
                pl_string.append(format_pl_statement(atom, nf_pred_extension))
            else:
                pl_string.append(format_pl_statement(atom, index))
    for pl_statement in pl_string:
        kb_lines.append('%s\n' % pl_statement)
    return kb_lines

def powerset(s):
    x = len(s)
    masks = [1 << i for i in range(x)]
    for i in range(1 << x):
        yield [ss for mask, ss in zip(masks, s) if i & mask]

def to_typed(lst):
    return [TypedObject(a, "object") for a in lst]



def get_type_constraints(concept):
    if(isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return [Atom(typed_object.type_name, (to_typed([typed_object.name])[0], )) for typed_object in concept.args]
    elif(isinstance(concept, TypedObject)):
        return [Atom(concept.type_name, (to_typed([concept.name])[0], ))]
    else:
        returns = [] 
        for part in concept.parts:
            returns = returns+get_type_constraints(part)
        return returns

def add_type_constraints(concept):
    if(isinstance(concept, UniversalCondition)):
        type_atoms = [Atom(obj.type_name, [TypedObject(obj.name, "object")]) for obj in concept.parameters]
        return UniversalCondition(concept.parameters, [Implication(Conjunction(type_atoms), Conjunction([add_type_constraints(c_part) for c_part in concept.parts]))])
    elif(isinstance(concept, ExistentialCondition)):
        type_atoms = [Atom(obj.type_name, [TypedObject(obj.name, "object")]) for obj in concept.parameters]
        return ExistentialCondition(concept.parameters, [Conjunction(type_atoms+[add_type_constraints(c_part) for c_part in concept.parts])])
    elif (isinstance(concept, Conjunction)):
        return Conjunction([add_type_constraints(c_part) for c_part in concept.parts])
    elif (isinstance(concept, Disjunction)):
        return Disjunction([add_type_constraints(c_part) for c_part in concept.parts])
    elif (isinstance(concept, Implication)):
        return Implication(add_type_constraints(concept.body), add_type_constraints(concept.head))
    return concept

def quantify_feature(feature):
    if(len(feature.ext_set) > 0 and len(feature.uni_set) > 0):
        return UniversalCondition(list(feature.uni_set), 
                                    [ExistentialCondition(
                                        list(feature.ext_set), [feature.concept])] )
    elif(len(feature.ext_set) > 0 and len(feature.uni_set) == 0):
        return ExistentialCondition(list(feature.ext_set), [feature.concept] )
    elif(len(feature.ext_set) == 0 and len(feature.uni_set) > 0):
        return UniversalCondition(list(feature.uni_set), [feature.concept] )
    elif(len(feature.ext_set) == 0 and len(feature.uni_set) == 0):
        return feature.concept

def remove_implication(concept):
    if(isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return concept 
    elif(isinstance(concept, Conjunction)):
        return Conjunction([remove_implication(c_part) for c_part in concept.parts])
    elif(isinstance(concept, Disjunction)):
        return Disjunction([remove_implication(c_part) for c_part in concept.parts])        
    elif(isinstance(concept, Implication)):
        head = remove_implication(concept.head)
        body = remove_implication(concept.body)
        return Disjunction( [body.negate(), head] )
    elif(isinstance(concept, UniversalCondition)):
        return UniversalCondition(concept.parameters, [remove_implication(c_part) for c_part in concept.parts])
    elif(isinstance(concept, ExistentialCondition)):
        return ExistentialCondition(concept.parameters, [remove_implication(c_part) for c_part in concept.parts])
    else:
        raise NotImplementedError


def get_additional_init(max_step=50):
    initial_state = [Atom("state0", [])]
    return initial_state
def untype_arg(arg):
    return arg.name


def untype_concept(concept):
    if(isinstance(concept, Atom)):
        return Atom(concept.predicate, [untype_arg(a) for a in concept.args] )
    elif(isinstance(concept, NegatedAtom)):
        return NegatedAtom(concept.predicate, [untype_arg(a) for a in concept.args] )
    elif(isinstance(concept, Conjunction)):
        return Conjunction([untype_concept(c_part) for c_part in concept.parts])
    elif(isinstance(concept, Implication)):
        head = untype_concept(concept.head)
        body = untype_concept(concept.body)
        return Disjunction( [body.negate(), head] )
    elif(isinstance(concept, UniversalCondition)):
        return UniversalCondition(to_typed([(p.name) for p in concept.parameters]), 
                                  [untype_concept(c_part) for c_part in concept.parts])
    elif(isinstance(concept, ExistentialCondition)):
        return ExistentialCondition(to_typed([(p.name) for p in concept.parameters]), 
                                    [untype_concept(c_part) for c_part in concept.parts])
   
    else:
        raise NotImplementedError 

def to_generic_typed_atom(atom):
    return Atom(atom[0],  [TypedObject(arg, "object") for arg in atom[1:]])


def get_arg_string(arg):
    if(isinstance(arg, list)):
        return "[{}]".format(", ".join(get_arg_string(a) for a in arg))
    elif(isinstance(arg, str) or isinstance(arg, int) or isinstance(arg, float)):
        return str(arg).lower()
    elif(isinstance(arg, tuple)):
        return get_arg_string(list(arg))
    elif(isinstance(arg, np.ndarray)):
        return str(arg).lower()
    elif(isinstance(arg, Conf)):
        return str(arg.values).lower()
    elif(isinstance(arg, Pose) or isinstance(arg, Grasp)):
        return str(arg.value).lower()
    elif(isinstance(arg, Commands)):
        return get_arg_string(arg.commands)
    elif(isinstance(arg, Trajectory) or isinstance(arg, SplitTrajectory)):
        return get_arg_string([get_arg_string(arg.path[0]), get_arg_string(arg.path[-1])])
    elif(isinstance(arg, RelPose)):
        return "rp:{}-{}-{}".format(str(arg.body).lower(), str(arg.parent_body).lower(), get_arg_string(arg.rel_pose))
    elif(isinstance(arg, Object)):
        return get_arg_string(arg.value)
    elif(isinstance(arg, TypedObject)):
        return get_arg_string(arg.name)
    else:
        print(type(arg))
        raise NotImplementedError


def count_negations(concept):
    if(isinstance(concept, Atom)):
        return 0
    elif(isinstance(concept, NegatedAtom)):
        return 1
    elif(isinstance(concept, Conjunction)):
        return sum([count_negations(c_part) for c_part in concept.parts])
    elif(isinstance(concept, Implication)):
        return sum([count_negations(concept.body), count_negations(concept.head)])
    elif(isinstance(concept, UniversalCondition) or isinstance(concept, ExistentialCondition)):
        return sum([count_negations(c_part) for c_part in concept.parts])
    else:
        raise NotImplementedError 

def np_print_repr(n):
    return str(n).lower().replace(" ", "")


def print_feature(feature):
    return "{} / {}".format(print_concept(feature.free_arg), print_concept(quantify_feature(feature)))

def print_concept(concept, typed=False):
    if (isinstance(concept, UniversalCondition) or
            isinstance(concept, ExistentialCondition)):
        concept_type = {"UniversalCondition": "forall", "ExistentialCondition": "exists"}[type(concept).__name__]
        if(typed):
            return "({} ({}) {})".format(concept_type, " ".join([print_concept("{} - {}".format(arg.name, arg.type_name), typed=typed) for arg in concept.parameters]), print_concept(concept.parts[0], typed=typed))
        else:
            return "({} ({}) {})".format(concept_type, " ".join([print_concept(arg, typed=typed) for arg in concept.parameters]), print_concept(concept.parts[0], typed=typed))    
                    
    elif (isinstance(concept, Conjunction)):
        if (len(concept.parts) > 1):
            return "(and " + " ".join([print_concept(part, typed=typed) for part in concept.parts]) + ")"
        else:
            return print_concept(concept.parts[0], typed=typed)
    elif (isinstance(concept, Disjunction)):
        if (len(concept.parts) > 1):
            return "(or " + " ".join([print_concept(part, typed=typed) for part in concept.parts]) + ")"
        else:
            return print_concept(concept.parts[0], typed=typed)

    elif (isinstance(concept, Implication)):
        if (len(concept.parts) > 1):
            return "(implies " + " ".join([print_concept(part, typed=typed) for part in concept.parts]) + ")"
        else:
            return print_concept(concept.parts[0], typed=typed)
    elif (isinstance(concept, NumericAtom)):
        return "(num_" + concept.predicate + " " + " ".join([print_concept(arg, typed=typed) for arg in concept.args]) + ")"        
    elif (isinstance(concept, Atom)):
        return "(" + concept.predicate + " " + " ".join([print_concept(arg, typed=typed) for arg in concept.args]) + ")"
    elif (isinstance(concept, NegatedAtom)):
        return "(" + "not " + print_concept(concept.negate(), typed=typed) + ")"
    elif (isinstance(concept, TypedObject)):
        return str(concept.name)
    elif (isinstance(concept, Effect)):
        print(concept.condition)
        if concept.parameters:
            if concept.condition != conditions.Truth() and not isinstance(concept.condition, bool):
                return "(forall ({}) (when {} {}))".format(" ".join([p.name for p in concept.parameters]), print_concept(concept.condition), print_concept(concept.literal))
            else:
                return "(forall ({}) {})".format(" ".join([p.name for p in concept.parameters]), print_concept(concept.literal))
        else:
            if concept.condition != conditions.Truth() and not isinstance(concept.condition, bool):
                return "(when {} {})".format(print_concept(concept.condition), print_concept(concept.literal))
            else:
                return "{}".format(print_concept(concept.literal))
        return ""
    elif(isinstance(concept, RelPose)):
        return (concept, )
    else:
        return str(concept)


def get_subsumption(pt, index, selected_features, quantified_concept_map):
    initial_state = None
    for problem in pt:
        descs = []
        start_features = []
        for concept in selected_features:
            f = concept
            init_c = int(len(quantified_concept_map[problem[index].index][concept]) > 0)
            descs.append(print_concept(concept))
            start_features.append((f, init_c))
        
        initial_state = start_features
    assert initial_state is not None
    return initial_state


def create_new_task_domain(task_domain, 
                           predicates = None, 
                           axioms = None,
                           actions = None,
                           predicate_dict = None):
    return Domain(task_domain.name,
                  task_domain.requirements,
                  task_domain.types,
                  task_domain.type_dict,
                  task_domain.constants,
                  predicates if predicates is not None else task_domain.predicates,
                  predicate_dict if predicate_dict is not None else task_domain.predicate_dict,
                  task_domain.functions,
                  actions if actions is not None else task_domain.actions,
                  axioms if axioms is not None else task_domain.axioms)


def get_description(selected_features):
    descs = []
    for selected_feature in selected_features:
        descs.append(print_concept(selected_feature.concept)+" / "+str(selected_feature.free_arg))
    return descs


def save_features(features, directory="./"):
    descs = get_description(features)

    # Save human-readable feature descriptions
    f = open(os.path.join(directory, "features.txt"), "w")
    for fi, feature_desc in enumerate(descs):
        f.write("f" + str(fi) + ": " + str(feature_desc) + "\n")
    f.close()

    # Save feature objects in a separate file
    with open(os.path.join(directory, 'features.pkl'), 'wb') as handle:
        pickle.dump(features, handle, protocol=pickle.HIGHEST_PROTOCOL)


def save_qnp_file(init, goal, actions, selected_features,
                  boolean_numeric_mapping, filename="./qnp_file.txt"):

    # Feature to name map
    feature_to_name_map = {}
    for fi, feature in enumerate(selected_features):
        feature_to_name_map[feature] = "f" + str(fi)

    f = open(filename, "w")
    bn_line = []

    f.write("qnp_domain\n")
    for feature in selected_features:
        bn_line += [str(feature_to_name_map[feature]),
                    str(boolean_numeric_mapping[feature])]

    f.write(str(int(len(bn_line) / 2.0)) + " " + " ".join(bn_line) + "\n")

    init_line = []
    for feature, v in init:
        init_line += [feature_to_name_map[feature], str(int(v))]

    f.write(str(int(len(init_line) / 2.0)) + " " + " ".join(init_line) + "\n")

    goal_line = []
    for feature, v in goal:
        goal_line += [feature_to_name_map[feature], str(int(v))]
    f.write(str(int(len(goal_line) / 2.0)) + " " + " ".join(goal_line) + "\n")

    # Number of actions
    f.write(str(int(len(actions))) + "\n")
    for ai, action in enumerate(actions):
        action_name_line = "a" + str(ai)
        f.write(action_name_line + "\n")
        action_precondition_line = []
        for feature, v in action["precondition"]:
            action_precondition_line += [feature_to_name_map[feature], str(int(v))]

        action_effect_line = []
        for feature, v in action["effect"]:
            action_effect_line += [feature_to_name_map[feature], str(int(v))]

        f.write(str(int(len(action_precondition_line) / 2.0)) + " " + " ".join(action_precondition_line) + "\n")
        f.write(str(int(len(action_effect_line) / 2.0)) + " " + " ".join(action_effect_line) + "\n")
    f.close()


class NumericAtom(Atom):
    def __init__(self, *args, **kwargs):
        super(NumericAtom, self).__init__(*args, **kwargs)

def qnp_line_to_dict(unparsed_line):
    line = unparsed_line.replace("\n", "").split(" ")
    eldict = {}
    for el in range(int(line[0])):
        eldict[line[el*2+1]] = int(line[el*2+2])
    return eldict

def parse_qnp_file(filename):
    f = open(filename, "r")
    lines = f.readlines()
    counts = qnp_line_to_dict(lines[1])
    init = qnp_line_to_dict(lines[2])
    goal = qnp_line_to_dict(lines[3])
    num_actions = int(lines[4])
    actions = []
    for action in range(num_actions):
        precondition = lines[5+action*3+1]
        effect = lines[5+action*3+2]
        actions.append((qnp_line_to_dict(precondition), 
                        qnp_line_to_dict(effect)))
    return counts, init, goal, actions

    
def get_experiment_dict(experiment):
    with open("./genqnp/experiments.yaml", 'r') as experiments_file:
        try:
            all_exps_dict = yaml.safe_load(experiments_file)[0]
            return all_exps_dict[experiment]
        except yaml.YAMLError as exc:
            print("Error: experiment doesn't exist")


class CloseAttach:
    def __init__(self, robot, arm, grasp, movable, obj, max_dist=0.01):
        self.movable = movable
        self.obj = obj
        self.grasp = grasp
        self.robot = robot
        self.max_dist = max_dist
        self.arm = arm
        self.link = link_from_name(self.robot, PR2_TOOL_FRAMES.get(self.arm,
                                                                   self.arm))

    # self.attachment = None

    def apply(self, state, **kwargs):
        for mobj in self.movable:
            if (mobj != self.obj):
                if (is_placement(mobj, self.obj)):
                    state.attachments[mobj] = create_attachment(self.robot,
                                                                self.link,
                                                                mobj)
                    state.grasps[mobj] = self.grasp
                    del state.poses[mobj]
        yield


class CloseDetach:
    def __init__(self, robot, arm, grasp, movable, obj, max_dist=0.01):
        self.movable = movable
        self.obj = obj
        self.grasp = grasp
        self.robot = robot
        self.max_dist = max_dist
        self.arm = arm
        self.link = link_from_name(self.robot,
                                   PR2_TOOL_FRAMES.get(self.arm, self.arm))

    # self.attachment = None

    def apply(self, state, **kwargs):
        for mobj in self.movable:
            if (mobj != self.obj):
                if (is_placement(mobj, self.obj)):
                    if (mobj in state.attachments.keys()):
                        del state.attachments[mobj]
                        state.poses[mobj] = RelPose(mobj, None, get_pose(mobj))
                        del state.grasps[mobj]
        yield


class RelPose(object):
    def __init__(self, body, parent_body, rel_pose, base=None):
        self.body = body
        self.parent_body = parent_body
        self.rel_pose = rel_pose

    @property
    def value(self):
        value = self.get_world_from_body()
        return value

    def assign(self):
        set_pose(self.body, self.get_world_from_body())

    def get_world_from_body(self):
        if(self.parent_body is None):
            orig_pose = unit_pose()
        else:
            orig_pose = get_pose(self.parent_body)
        return multiply(orig_pose, self.rel_pose)

    def __repr__(self):
        if self.parent_body is None:
            return 'wp{}'.format(id(self) % 1000)
        return 'rp{}'.format(id(self) % 1000)


def QPose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return point, quat_from_euler(euler)


def atom_to_symbol_pyeda(atom):
    args = [arg.name for arg in atom.args]
    return atom.predicate+"".join(args).replace("?", "")


def atom_to_symbol_sympy(atom):
    args = [arg.name for arg in atom.args]
    return atom.predicate+"".join(args)


def convert_to_sympy(concept):
    if(isinstance(concept, Atom)):
        return symbols(atom_to_symbol_sympy(concept))
    elif(isinstance(concept, NegatedAtom)):
        return SymNot(symbols(atom_to_symbol_sympy(concept)))
    elif(isinstance(concept, Conjunction)):
        return SymAnd(*[convert_to_sympy(c_part) for c_part in concept.parts])
    elif(isinstance(concept, Implication)):
        head = convert_to_sympy(concept.head)
        body = convert_to_sympy(concept.body)
        return SymOr(SymNot(body), head)
    else:
        raise NotImplementedError


def get_parts_flat(concept):
    if (isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return [concept]
    else:
        total_parts = []
        for c in concept.parts:
            total_parts += list(get_parts_flat(c))
        return total_parts

def objectify_expression(concept):
    if(isinstance(concept, Atom)):
        return Atom(concept.predicate, [objectify_expression(arg) for arg in concept.args])
    elif(isinstance(concept, NegatedAtom)):
        return NegatedAtom(concept.predicate, [objectify_expression(arg) for arg in concept.args])
    elif(isinstance(concept, Conjunction)):
        return Conjunction([objectify_expression(cp) for cp in concept.parts])
    elif(isinstance(concept, Disjunction)):
        return Disjunction([objectify_expression(cp) for cp in concept.parts])
    elif(isinstance(concept, str)):
        return TypedObject(concept, "object")
    elif(isinstance(concept, UniversalCondition)):
        return UniversalCondition(concept.parameters, [objectify_expression(cp) for cp in concept.parts])
    elif(isinstance(concept, ExistentialCondition)):
        return ExistentialCondition(concept.parameters, [objectify_expression(cp) for cp in concept.parts])
    else:
        raise NotImplementedError


def convert_to_prolog(concept, pred_extension="", nf_pred_extension=0, nonfluents=[]):
    if(isinstance(concept, Atom)):
        if(concept.predicate in nonfluents):
            pred = concept.predicate + "_" + str(nf_pred_extension)
        else:
            pred = concept.predicate + pred_extension
        return pred+"("+", ".join([a.name.replace("?", "_") for a in concept.args])+")"
    elif(isinstance(concept, NegatedAtom)):
        if(concept.predicate in nonfluents):
            pred = concept.predicate + "_" + str(nf_pred_extension)
        else:
            pred = concept.predicate + pred_extension
        return "not(("+pred+"("+", ".join([a.name.replace("?", "_") for a in concept.args])+"))"+")"
    elif(isinstance(concept, Conjunction)):
        return "("+",".join([convert_to_prolog(c, pred_extension=pred_extension, 
                                                  nf_pred_extension = nf_pred_extension, 
                                                  nonfluents=nonfluents) for c in concept.parts])+")"
    elif(isinstance(concept, Implication)):
        head = convert_to_prolog(concept.head, 
                                 pred_extension=pred_extension, 
                                 nf_pred_extension = nf_pred_extension, 
                                 nonfluents=nonfluents)
        body = convert_to_prolog(concept.body, 
                                 pred_extension = pred_extension, 
                                 nf_pred_extension = nf_pred_extension, 
                                 nonfluents = nonfluents)
        return "not(({}));{}".format(body, head)
    elif(isinstance(concept, Disjunction)):
        return "("+";".join([convert_to_prolog(c, pred_extension = pred_extension, 
                                                  nf_pred_extension = nf_pred_extension, 
                                                  nonfluents = nonfluents) for c in concept.parts])+")"
    else:
        raise NotImplementedError


def convert_to_pyeda(concept):
    if(isinstance(concept, Atom)):
        return exprvar(atom_to_symbol_pyeda(concept))
    elif(isinstance(concept, NegatedAtom)):
        return Not(exprvar(atom_to_symbol_pyeda(concept)))
    elif(isinstance(concept, Conjunction)):
        return And(*[convert_to_pyeda(c_part) for c_part in concept.parts])
    elif(isinstance(concept, Implication)):
        head = convert_to_pyeda(concept.head)
        body = convert_to_pyeda(concept.body)
        return Or(Not(body), head)
    else:
        raise NotImplementedError



# TODO: Move this function to task
def create_bucket(w, le, h, t=0.01, mass=STATIC_MASS, color=BROWN):
    shape_array = []
    pose_array = []
    color_array = []

    # Bottom Face
    shape_array.append(get_box_geometry(w, le, t))
    pose_array.append(unit_pose())
    color_array.append(color)

    # Side faces
    shape_array.append(get_box_geometry(w, t, h))
    pose_array.append(((0, le / 2.0, h / 2.0), unit_quat()))
    color_array.append(color)

    shape_array.append(get_box_geometry(w, t, h))
    pose_array.append(((0, -le / 2.0, h / 2.0), unit_quat()))
    color_array.append(color)

    shape_array.append(get_box_geometry(t, le, h))
    pose_array.append(((w / 2.0, 0, h / 2.0), unit_quat()))
    color_array.append(color)

    shape_array.append(get_box_geometry(t, le, h))
    pose_array.append(((-w / 2.0, 0, h / 2.0), unit_quat()))
    color_array.append(color)

    collision_id, visual_id = create_shape_array(shape_array,
                                                 pose_array,
                                                 colors=color_array)

    return create_body(collision_id, visual_id, mass=mass)


def read_from_minio(file):

    minioClient = Minio('s3.amazonaws.com',
                    access_key=os.environ["S3ACCESS_CUST"],
                    secret_key=os.environ["S3SECRET_CUST"],
                    secure=True)

    tmp_folder = "./tmp_minio/"
    if(not os.path.isdir(tmp_folder)):
        os.mkdir(tmp_folder)
  
    tmp_path = tmp_folder + file


    # s to get objects in folder
    minioClient.fget_object('gentamp', "gentamp/"+str(file), tmp_path)

    # Write data to a pickle file
    with open(tmp_path, 'rb') as handle:
        b = pickle.load(handle)
    return b

    # data = read_from_minio("003_cracker_box.pkl")


def write_to_minio(obj, name):
    print("Saving minio: "+str(name))
    # plot grasps in mayavi

    minioClient = Minio('s3.amazonaws.com',
                        access_key=os.environ["S3ACCESS_CUST"],
                        secret_key=os.environ["S3SECRET_CUST"],
                        secure=True)

    tmp_folder = "./tmp_minio/"
    if(not os.path.isdir(tmp_folder)):
        os.mkdir(tmp_folder)
    tmp_filename = str(name)+".pkl"
    tmp_path = tmp_folder + tmp_filename
    # Write data to a pickle file
    with open(str(tmp_path), 'wb') as handle:
        pickle.dump(obj, handle, protocol=pickle.HIGHEST_PROTOCOL)

    # Write file to bucket
    minioClient.fput_object('gentamp', "gentamp/"
                            + str(tmp_filename), str(tmp_path))
    shutil.rmtree(tmp_folder)

INFO_FROM_BODY = {}

ModelInfo = namedtuple('URDFInfo', ['name', 'path', 'fixed_base', 'scale'])


def get_heuristic(Q, mode=None):
    # TODO take from pyperplan
    if(mode == "policy"):
        random.shuffle(Q)
        return sorted(Q, key=lambda k: (k.h, -int(k.depth)) )
    else:
        return sorted(Q, key=lambda k: (k.h))

def create_ycb(ycb_path, mesh_type='centered', project_base=False, z_threshold=1e-2, mass_scale=1e-1, use_concave=False, **kwargs): 
    if(ycb_path in scales):
        scale = scales[ycb_path]
    else:
        scale=1
    ycb_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ycb_path)
    body = load_pybullet(ycb_path)
    # body = create_obj(ycb_path, color=WHITE, scale = scale, **kwargs)
    INFO_FROM_BODY[CLIENT, body] = ModelInfo(None, ycb_path, False, scale)
    return body

def get_model_info(body):
    key = (CLIENT, body)
    return INFO_FROM_BODY.get(key, None)

def vertices_from_rigid(body, link=BASE_LINK):
    assert implies(link == BASE_LINK, get_num_links(body) == 0)
    try:
        vertices = vertices_from_link(body, link)
    except RuntimeError:
        info = get_model_info(body)
        assert info is not None
        _, ext = os.path.splitext(info.path)
        if ext == '.obj':
            if info.path not in OBJ_MESH_CACHE:
                OBJ_MESH_CACHE[info.path] = read_obj(info.path, decompose=False)
            mesh = OBJ_MESH_CACHE[info.path]
            vertices = mesh.vertices
        else:
            raise NotImplementedError(ext)
    return vertices

def approximate_as_prism(body, body_pose=unit_pose(), **kwargs):
    # TODO: make it just orientation
    vertices = apply_affine(body_pose, vertices_from_rigid(body, **kwargs))
    aabb = aabb_from_points(vertices)
    return get_aabb_center(aabb), get_aabb_extent(aabb)
    #with PoseSaver(body):
    #    set_pose(body, body_pose)
    #    set_velocity(body, linear=np.zeros(3), angular=np.zeros(3))
    #    return get_center_extent(body, **kwargs)

def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose2(euler=[0, math.pi, 0])
    translate_z = Pose2(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose2(point=point_from_pose(body_pose)-center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose2(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose2(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    return grasps


def get_grasp_gen(problem, collisions=False, randomize=True):
    for grasp_type in problem.grasp_types:
        if grasp_type not in GET_GRASPS:
            raise ValueError('Unexpected grasp type:', grasp_type)
    def fn(body):
        # TODO: max_grasps
        # TODO: return grasps one by one
        grasps = []
        arm = 'left'
        #carry_conf = get_carry_conf(arm, 'top')
        if 'top' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([1, 0, 0])
            grasps.extend(Grasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM)
                          for g in get_top_grasps(body, grasp_length=GRASP_LENGTH))
        if 'side' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([2, 0, -1])
            grasps.extend(Grasp('side', body, g, multiply((approach_vector, unit_quat()), g), SIDE_HOLDING_LEFT_ARM)
                          for g in get_side_grasps(body, grasp_length=GRASP_LENGTH))
        filtered_grasps = []
        for grasp in grasps:
            grasp_width = compute_grasp_width(problem.robot, arm, body, grasp.value) if collisions else 0.0
            if grasp_width is not None:
                grasp.grasp_width = grasp_width
                filtered_grasps.append(grasp)
        if randomize:
            random.shuffle(filtered_grasps)
        return [(g,) for g in filtered_grasps]
        #for g in filtered_grasps:
        #    yield (g,)
    return fn



