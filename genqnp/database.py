import os
import pickle
import numpy as np
from os import listdir
from os.path import isfile, join
import copy
import time
from collections import defaultdict
import itertools
import pickle
import argparse
import pyswip
import importlib
from pyswip import Prolog
from sympy import symbols, simplify_logic
from sympy import Not as SymNot
from sympy import Or as SymOr
from sympy import And as SymAnd
from pyeda.inter import Or, Not, And, exprvar

from GeneralizedTAMP.pddlstream.pddlstream.utils import read
from GeneralizedTAMP.pddlstream.pddlstream.algorithms.algorithm import parse_stream_pddl
from GeneralizedTAMP.pddlstream.pddlstream.language.attachments import has_attachments, \
    compile_fluents_as_attachments, solve_pyplanners
from GeneralizedTAMP.pddlstream.pddlstream.algorithms.common import evaluations_from_init, \
    add_certified, add_facts, UNKNOWN_EVALUATION
from GeneralizedTAMP.pddlstream.pddlstream.language.conversion import value_from_evaluation
from GeneralizedTAMP.pddlstream.pddlstream.algorithms.instantiation import Instantiator
from pddl import Atom, NegatedAtom, Conjunction, UniversalCondition, \
    ExistentialCondition, TypedObject

from language.planner import parse_sequential_domain,\
    get_streams_certified_pddl
from opensat_interface import solve_sat
from utils import save_features, save_qnp_file, get_experiment_dict,\
    create_new_task_domain, get_subsumption, print_concept, Implication,\
    GenTAMPProblem, powerset, np_print_repr, get_arg_string, \
    to_generic_typed_atom, convert_to_sympy, convert_to_prolog, \
    convert_to_pyeda, BOOLEAN, NUMERIC, get_parts_flat, atom_to_symbol_pyeda, \
    sign
from stream_utils import process_instance, process_stream_queue


###### Generator storage objects #######

class TState(object):
    def __init__(self, atoms, value_map):
        self.atoms = atoms
        self.value_map = value_map


class TAction(object):
    def __init__(self, action):
        self.action = action


class TPlan(object):
    def __init__(self, tstates, tactions, evaluation_map, spec):
        self.spec = spec
        self.evaluation_map = evaluation_map
        self.tstates = tstates
        self.tactions = tactions


class TDatabase(object):
    def __init__(self, expdir):
        self.expdir = expdir
        self.savedir = self.expdir['gen_tamp_results_folder']

        if (not os.path.isdir(self.savedir)):
            os.mkdir(self.savedir)

        self.plans = {}


    def load(self):
        # Get list of files in the save directory
        for train_problem in self.expdir['training_problems']:
            with open("{}/{}.pkl".format(self.savedir, train_problem), 'rb') as handle:
                self.plans[train_problem] = pickle.load(handle)

    def add_plan(self, tname, tplan):
        self.plans[tname] = tplan

    def save(self):
        #  Check if the save folder exists
        if not os.path.isdir(self.savedir):
            os.mkdir(self.savedir)



        for plan_name, tplan in self.plans.items():
            print("saving: "+str("{}/{}.pkl".format(self.savedir, plan_name)))
            with open("{}/{}.pkl".format(self.savedir, plan_name), 'wb') as handle:
                pickle.dump(tplan, handle)



###### GenTAMP Structures #######


class Plan():
    def __init__(self, task_domain, problem_name):
        self.transitions = []
        self.stream_facts = []
        self.stream_value_map = {}
        self.task_domain = task_domain
        self.problem_name = problem_name

    def add_transition(self, action, state, value_map, objects, goal=False, index=0):
        value_map.update(self.stream_value_map)
        self.transitions.append(Transition(action, 
                                           list(set(state + self.stream_facts)), 
                                           value_map,
                                           objects, 
                                           self.task_domain, 
                                           goal=goal, 
                                           index=index))

    def __iter__(self):
        for transition in self.transitions:
            yield transition

    def __len__(self):
        return len(self.transitions)

    def __getitem__(self, i):
        return self.transitions[i]


    def prune_redundant_types(self, task_domain):
        # Step 1: Collect all the objects
        total_objects = []
        for transition in self:
            total_objects += transition.objects

        # Step 2: Find all the single arity predicates
        sa_predicates = []
        for predicate in task_domain.predicates:
            if(len(predicate.arguments) == 1):
                sa_predicates.append(predicate)

        # Step 3: For each predicate, remove it from the task domain under the following conditions 
        #         if the predicate holds true in every state for every object of a particular type
        removing_predicates = []
        for sa_predicate in sa_predicates:
            # Get the predicate argument type
            sa_type = sa_predicate.arguments[0].type_name
            is_type_sa_predicate = True
            for transition in self:
                for obj in transition.typed_objects[sa_type]:
                    in_state = any([(state_atom.predicate == sa_predicate.name and state_atom.args[0].name == obj.name) for state_atom in list(transition.state)])
                    if(not in_state):
                        is_type_sa_predicate = False

            if(is_type_sa_predicate):
                removing_predicates.append(sa_predicate)

        # Remove all unused predicates
        unused_predicates = []
        for predicate in task_domain.predicates:
            used = False
            for transition in self:
                for state_atom in list(transition.state):
                    if (state_atom.predicate == predicate.name):
                        used = True

            if(not used or predicate.name == "="):
                unused_predicates.append(predicate)

        all_removing_predicates = []
        for sa_predicate in task_domain.predicates:
            if (sa_predicate not in unused_predicates + removing_predicates):
                all_removing_predicates.append(sa_predicate)


        new_task_domain = create_new_task_domain(task_domain, predicates=all_removing_predicates)

        # Use the new domain to overwrite the transitions without the unused predicates
        print("Overwrite transitions")
        new_transitions = []
        for transition in self:
            cleaned_state = []
            for dstate_atom in list(set(transition.state + self.stream_facts)):
              
                if(dstate_atom.predicate in [p.name for p in all_removing_predicates] ):
                    cleaned_state.append(dstate_atom)

            cleaned_state = sorted(cleaned_state, key = lambda x: x.predicate)            
            new_transitions.append(Transition(transition.action, 
                                              cleaned_state, 
                                              transition.value_map,
                                              transition.objects, 
                                              new_task_domain, 
                                              goal=transition.goal, 
                                              index=transition.index))

        self.transitions = new_transitions
        self.task_domain = new_task_domain
        return new_task_domain



class Transition():
    def __init__(self, action, state, value_map, objects, domain, goal=False, index=0):
        self.action = action
        self.index = index
        self.value_map = value_map
        self.state = state
        self.goal = goal
        self.objects = objects
        self.typed_objects = self.get_typed(self.objects)

        self.flat_atoms = defaultdict(lambda: False)
        for atom in state:
            if(isinstance(atom, tuple)):
                self.flat_atoms[atom] = True
            else:
                self.flat_atoms[tuple([atom.predicate]+[arg.name for arg in list(atom.args)])] = True

        self.flat_atoms = dict(self.flat_atoms)

        self.domain = domain
        self.preconditions = {}
        self.effects = {}
        self.constants = []

    def get_value_state(self):
        value_state = []
        for atom in self.state:
            value_args = []
            for arg in atom.args:
                value_args.append(self.value_map[arg.name])

            value_state.append(tuple([atom.predicate]+value_args))
        return value_state

    def get_typed(self, objects):
        typed = defaultdict(lambda: [])
        for obj in objects:
            typed[obj.type_name].append(obj)
        return dict(typed)

    def map_atoms(self, atoms):
        atom_map = defaultdict(lambda: defaultdict(list))
        for atom in atoms:
            if (len(atom.args) == 0):
                self.constants.append(atom.predicate)
            for arg in atom.args:
                atom_map[arg][atom.predicate].append(atom)
        return dict(atom_map)

    def set_precondition(self, concept, bool_or_num, value):
        self.preconditions[concept] = (bool_or_num, value)

    @property
    def effects_tuple(self):
        return tuple(self.effects.items())

    def set_effect(self, concept, bool_or_num, value):
        if (value != 0):
            self.effects[concept] = (bool_or_num, sign(value))


    def feature_check(self, args, concept):
        concept_parts = get_parts_flat(concept)
        # TODO: Factorized object placement


        arg_dict = {}
       

        for ai, primitive_arg in enumerate(args):
            primitive_arg_type = primitive_arg.type_name
            arg_dict[primitive_arg.name] = self.typed_objects[primitive_arg_type]

        
        assignments = []
        concept_parts = sorted(concept_parts, key=lambda x:len(x.args))
        for part in concept_parts:
            vardict = {k.name:v for k, v in zip(args, tuple(arg_list))}
            if(self.check_valid_assignment(vardict, formula, concept)):
                assignments.append(vardict)
        return set(assignments)

