from collections import Counter

import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.common import add_facts, add_certified, SolutionStore, UNKNOWN_EVALUATION
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.downward import get_problem, task_from_domain_problem
from pddlstream.algorithms.instantiate_task import sas_from_pddl, instantiate_task
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.search import abstrips_solve_from_task
from pddlstream.language.constants import is_plan
from pddlstream.language.conversion import obj_from_pddl_plan
from pddlstream.language.attachments import has_attachments, compile_fluents_as_attachments, solve_pyplanners
from pddlstream.language.statistics import load_stream_statistics, write_stream_statistics
from pddlstream.language.temporal import solve_tfd, SimplifiedDomain
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.utils import INF, Verbose, str_from_object, elapsed_time
from database import Transition, TState
from utils import objectify_expression, encode_state_kb, populate_state_prolog, \
    create_kb, findall_query_raw, infer_object_type
from pddl import Atom, NegatedAtom, Conjunction, UniversalCondition, \
    ExistentialCondition, TypedObject
from collections import defaultdict
from language.planner import t_get_action_instance
import itertools
from utils import get_arg_string, is_continuous, get_heuristic, NumericAtom, clear_temp
from language.planner import literal_holds
from pddl import Atom, NegatedAtom, Conjunction, UniversalCondition, \
    ExistentialCondition, TypedObject, Disjunction, Effect, conditions
import copy
import pddl
import importlib
import pyswip
import random
from collections import namedtuple

Certificate = namedtuple('Certificate', ['all_facts', 'preimage_facts', 'plan_graph'])

UPDATE_STATISTICS = False

def solve_temporal(evaluations, goal_exp, domain, debug=False, **kwargs):
    assert isinstance(domain, SimplifiedDomain)
    problem = get_problem_pddl(evaluations, goal_exp, domain.pddl)
    return solve_tfd(domain.pddl, problem, debug=debug)

def solve_sequential(evaluations, goal_exp, domain, unit_costs=False, debug=False, **search_args):
    problem = get_problem(evaluations, goal_exp, domain, unit_costs)
    task = task_from_domain_problem(domain, problem)
    if has_attachments(domain):
        with Verbose(debug):
            instantiated = instantiate_task(task)
        return solve_pyplanners(instantiated, **search_args)
    sas_task = sas_from_pddl(task, debug=debug)
    return abstrips_solve_from_task(sas_task, debug=debug, **search_args)

def solve_finite(evaluations, goal_exp, domain, **kwargs):
    if isinstance(domain, SimplifiedDomain):
        pddl_plan, cost = solve_temporal(evaluations, goal_exp, domain, **kwargs)
    else:
        pddl_plan, cost = solve_sequential(evaluations, goal_exp, domain, **kwargs)
    plan = obj_from_pddl_plan(pddl_plan)
    return plan, cost

##################################################


def facts_from_evaluations(evaluations):
    return [tuple([ev.head.function]+[a.value for a in list(ev.head.args)]) for ev in list(evaluations.keys())]

def process_instance(instantiator, store, instance, verbose=False): #, **complexity_args):
    tuples = []
    if instance.enumerated:
        return []
    start_time = time.time()
    new_results, new_facts = instance.next_results(verbose=verbose)
    store.sample_time += elapsed_time(start_time)

    evaluations = store.evaluations
    #remove_blocked(evaluations, instance, new_results)
    for result in new_results:
        complexity = result.compute_complexity(evaluations)
        #complexity = instantiator.compute_complexity(instance)
        for evaluation in add_certified(evaluations, result):
            tuples.append((evaluation, complexity))
            instantiator.add_atom(evaluation, complexity)
    fact_complexity = 0 # TODO: record the instance or treat as initial?
    for evaluation in add_facts(evaluations, new_facts, result=UNKNOWN_EVALUATION, complexity=fact_complexity):
        instantiator.add_atom(evaluation, fact_complexity)
    if not instance.enumerated:
        instantiator.push_instance(instance)   
    return new_results, tuples

def process_stream_queue(instantiator, store, complexity_limit=INF, verbose=False):
    instances = []
    results = []
    tuples = []
    num_successes = 0
    while not store.is_terminated() and instantiator and (instantiator.min_complexity() <= complexity_limit):
        instance = instantiator.pop_stream()
        if instance.enumerated:
            continue
        instances.append(instance)
        new_results, new_tuples = process_instance(instantiator, store, instance, verbose=verbose)
        tuples += new_tuples

        results.extend(new_results)
        num_successes += bool(new_results) # TODO: max_results?
    if verbose:
        print('Eager Calls: {} | Successes: {} | Results: {} | Counts: {}'.format(
            len(instances), num_successes, len(results),
            str_from_object(Counter(instance.external.name for instance in instances))))

    # tuples = sorted(tuples, key=lambda x: x[0].head.function)
    # for tup in tuples:
    #     evaluation = tup[0]
    #     print(tuple([evaluation.head.function] + [get_arg_string(a) for a in evaluation.head.args] ), tup[1])
    return len(instances)

##################################################


def row_join(row1, row2):

    # Join the two rows, returns none if they cannot be joined
    new_row1 = {}
    new_row1.update(row1)
    new_row1.update(row2)
    new_row2 = {}
    new_row2.update(row2)
    new_row2.update(row1)
    if(new_row1 == new_row2):
        return new_row1
    else:
        return None

def join(current_db, joining_db, negated=True):
    new_db = []
    # Todo: can be more efficient
    if (current_db is None):
        return joining_db
    elif (joining_db is None):
        return current_db
    else:
        for row1, row2 in itertools.product(current_db, joining_db):
            joined_row = row_join(row1, row2)
            if (joined_row != None):
                new_db.append(joined_row)
    return new_db


def get_successor_gen(ground_atoms, parameters, preconditions, current_db=None, verbose=False, nonfluents=[]):
    
    if(isinstance(preconditions, Atom) or isinstance(preconditions, NegatedAtom)):
        preconditions = Conjunction([preconditions])

    st = time.time()
    ground_atoms_dict = defaultdict(list)
    for ground_atom in ground_atoms:
        ground_atoms_dict[ground_atom[0]].append(ground_atom[1:])

    preconditions.parts = sorted(list(preconditions.parts), key=lambda x: (int(x.predicate not in nonfluents), len(x.args)), reverse=False)
    variable_map = defaultdict(list)

    # Constants
    empty = False
    for precondition_part in reversed(list(preconditions.parts)):
        if(len(precondition_part.args) == 0):
            if (isinstance(precondition_part, Atom)):
                if(len(ground_atoms_dict[precondition_part.predicate])==0):
                    if(verbose):
                        print("None1: {}".format(precondition_part.predicate))
                    empty = True
            elif(isinstance(precondition_part, NegatedAtom)):
                if(len(ground_atoms_dict[precondition_part.predicate])>0):
                    if(verbose):
                        print("None2: {}".format(precondition_part.predicate))
                    empty = True

    if(empty):
        if(verbose):
            print("Empty Successor Gen")
        current_db = []
        def successor_gen():
            for row in current_db:
                yield tuple([row[p.name] for p in parameters])

        # print("Successor function time: "+str(time.time()-st))
        return successor_gen
    else:
        for precondition_part in reversed(list(preconditions.parts)):

            if verbose:
                print("=============================")
                print(precondition_part)

            # For this precondition part, find every object combination which exists in the ground atoms
            if (isinstance(precondition_part, Atom)):
                pre_part_db = []
                for args in ground_atoms_dict[precondition_part.predicate]:
                    if(len(precondition_part.args) > 0 and isinstance(precondition_part.args[0], str) ):
                        pre_part_db.append({precondition_part.args[i]: args[i] for i in range(len(args))})
                    else:
                        pre_part_db.append({precondition_part.args[i].name: args[i] for i in range(len(args))})
            elif (isinstance(precondition_part, NegatedAtom)):
                continue
            else:
                raise NotImplementedError

            if(verbose):
                print("==predb==")
                for row in pre_part_db:
                    print(row)

            st2 = time.time()

            current_db = join(current_db, pre_part_db)

            if(verbose):
                print("==Current db==")
                for row in current_db:
                    print(row)
                

        def successor_gen():
            for row in current_db:
                yield tuple([row[p.name] for p in parameters])

        # print("Successor function time: "+str(time.time()-st))
        return successor_gen



def apply_action(state, stringified_atoms, stringify_dict, action, action_params, io2s, nonfluents=[]):

    del_effects = []
    add_effects = []
    new_state = []
    del_state = []
    new_stringify_state = []
    successor_action_simp = {k.name: v for k, v in zip(action.parameters, action_params) }
    for effect in action.effects:
        if(effect.condition == conditions.Truth()):
            if( isinstance(effect.literal, Atom) ):
                add_effects.append(effect.literal)
            elif( isinstance(effect.literal, NegatedAtom) ):
                del_effects.append(effect.literal)
            else:
                raise NotImplementedError
        else:
            # print("processing conditional effect")
            # print(effect.literal)
            current_db = [successor_action_simp]

            # print(successor_action_simp)
            # print(current_db)
            successor_fn = get_successor_gen(stringified_atoms, effect.parameters, effect.condition, current_db=current_db, nonfluents=nonfluents)
            successor_gen = successor_fn()
            successor_list = list(successor_gen)

            print(successor_list)
            for successor in successor_list:
                new_successor_action_simp = {k: v for k, v in zip(effect.literal.args, successor) }
                if( isinstance(effect.literal, Atom)):
                    new_state.append(Atom(effect.literal.predicate, [io2s[new_successor_action_simp[a]] for a in effect.literal.args] ))
                    new_stringify_state.append(tuple([effect.literal.predicate] + [get_arg_string(new_successor_action_simp[a]) for a in effect.literal.args] ))
                    stringify_dict[new_stringify_state[-1]] = new_state[-1]

                elif( isinstance(effect.literal, NegatedAtom) ):
                    del_state.append(tuple([effect.literal.predicate]+[new_successor_action_simp[a] for a in effect.literal.args]))
                else:
                    raise NotImplementedError
 
 
    # assert (isinstance(action, pddl.PropositionalAction))
    # # TODO: signed literals
    for effect in del_effects:
        del_state.append(tuple([effect.predicate]+[successor_action_simp[a] for a in list(effect.args)] ))

    for (orig_state, state_el) in zip(list(state), list(stringified_atoms)):
        if state_el not in del_state:
            new_state.append(orig_state)
            new_stringify_state.append(state_el)

    # print("stringify state new: "+str(time.time()-sta))
    # sta = time.time()

    # # TODO: This loops through every possibility for for conditions which takes forever
    # # We need to replace this with successor generation

    for effect in add_effects:
        new_state.append( Atom(effect.predicate, [io2s[successor_action_simp[a]] for a in effect.args] ) )
        new_stringify_state.append(tuple([effect.predicate] + [get_arg_string(successor_action_simp[a]) for a in effect.args] ))
        stringify_dict[new_stringify_state[-1]] = new_state[-1]

    # print("Adding: ", str(set(new_state)-set(state)))
    # print("Del: ", str(set(state)-set(new_state)))

    return set(new_state), set(new_stringify_state), stringify_dict

def remove_new_axiom(cond):
    if(isinstance(cond, Conjunction)):
        new_parts = []
        for cond_part in cond.parts:
            if(not cond_part.predicate.startswith("new-axiom")):
                new_parts.append(cond_part)
        return Conjunction(new_parts)
    else:
        return remove_new_axiom(Conjunction([cond]))

class MCTSNode():
    def __init__(self,
                 atoms,
                 stringified_atoms,
                 axiom_atoms,
                 stringified_axiom_atoms,
                 action,
                 action_args,
                 successor_action,
                 h,
                 cost,
                 depth):
        self.h = h
        self.stringified_atoms = stringified_atoms
        self.successor_action = successor_action
        self.cost = cost
        self.atoms = atoms
        self.axiom_atoms = axiom_atoms
        self.stringified_axiom_atoms = stringified_axiom_atoms
        self.action = action
        if(action is not None):
            self.action_name = action.name
        else:
            self.action_name = None
        self.action_args = action_args
        self.depth = depth

class MCTSGraph():
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges


def evaluate_axiom(axiom, 
                   stringified_atoms, 
                   domain, 
                   state_index, 
                   typed_object_set, 
                   value_map, nonfluents, 
                   encode_nonfluents=False,
                   add_dynamic = True,
                   nf_pred_extension=0):
    # st = time.time()
    stringified_atoms = sorted(stringified_atoms, key=lambda x: x[0])
    kb_lines = encode_state_kb(stringified_atoms, state_index, 
                               nonfluents=nonfluents, encode_nonfluents=encode_nonfluents,
                               nf_pred_extension=nf_pred_extension)
    if(add_dynamic):
        kb_lines = populate_state_prolog(domain, state_index, 
                                         encode_nonfluents=encode_nonfluents,
                                         nf_pred_extension=nf_pred_extension) + kb_lines

    
    kb = create_kb(kb_lines)
    
    transition = Transition(None, stringified_atoms, value_map, typed_object_set, domain, index=state_index) # TODO atoms is the state  
    axiom_atoms = []
    c = objectify_expression(axiom.condition)
    results, valmap = findall_query_raw(axiom.parameters, 
                                        transition, 
                                        c, 
                                        kb, 
                                        nonfluents=nonfluents, 
                                        nf_pred_extension=nf_pred_extension)
    if(isinstance(results, list)):
        for result in results:
            axiom_atom = Atom(axiom.name, [valmap[obj] for obj in result])
            axiom_atoms.append(axiom_atom)
    else:
        if(results):
            axiom_atom = Atom(axiom.name, [])
            axiom_atoms.append(axiom_atom)

    return axiom_atoms

def conditions_hold(state, conditions):
    return all(literal_holds(state, cond) for cond in conditions)


def check_goal(atom_tuples, goal_atoms):
    # First convert to hashable format
    goal_tuples = [tuple([at.predicate]+[get_arg_string(a) for a in at.args]) for at in goal_atoms]
    non_goals = len([gt for gt in goal_tuples if gt not in atom_tuples])
    return non_goals

def check_inc_dec(incs, decs, old_atom_tuples, new_atom_tuples):
    
    
    
    # First convert to hashable format
    for inc in incs:
        # print(inc)
        # print(inc.literal)
        old_inc = len([o for o in old_atom_tuples if o[0] == inc.literal.predicate])
        new_inc = len([o for o in new_atom_tuples if o[0] == inc.literal.predicate])
        # print("old_inc")
        # print(old_inc)
        # print("new_inc")
        # print(new_inc)
        if(isinstance(inc.literal, NumericAtom)):
            # print("Numeric")
            # Numeric
            if(new_inc <= old_inc):
                return False
        else:
            # print("Boolean")
            # Boolean
            if(new_inc == 0):
                return False

    for dec in decs:
        old_dec = len([o for o in old_atom_tuples if o[0] == dec.literal.predicate])
        new_dec = len([o for o in new_atom_tuples if o[0] == dec.literal.predicate])
        if(isinstance(dec.literal, NumericAtom)):
            # Numeric
            if(new_dec >= old_dec):
                return False
        else:
            # Boolean
            if(new_dec > 0):
                return False
        
    return True


def get_stringified_atoms(atoms, cache, o2s):
    stringified_atoms=[]
    for a in atoms:
        if(a in cache):
            stringified_atoms.append(cache[a])
        else:
            stringified_atoms.append(tuple([a.predicate]+[o2s[arg] for arg in a.args]))
            cache[a] = stringified_atoms[-1]
    return stringified_atoms, cache


def get_action_cost(action_name, default_action, action_costs):
    current_action_cost = default_action
    for name, cost in action_costs.items():
        if(action_name.startswith(name)):
            return cost
    return current_action_cost

def disc(state_str_atoms, cont_strs):
    st = time.time()
    new_atoms = []
    for atom in state_str_atoms:
        new_atom = tuple([obj for obj in atom if (obj not in cont_strs) and not is_continuous(obj)])
        new_atoms.append(new_atom)
    return new_atoms
   

def solve_mcts(problem, 
               constraints=PlanConstraints(),
               unit_costs=False, 
               success_cost=INF,
               parsed_domain=None,
               max_iterations=INF, 
               max_time=60*10, 
               max_memory=INF,
               initial_complexity=0, 
               complexity_step=1, 
               max_complexity=INF,
               action_costs = {},
               default_action_cost = 0,
               nonfluents = [],
               skeleton=None,
               heuristic=None,
               verbose = False, 
               **search_kwargs):
    """
    Solves a PDDLStream problem by alternating between applying all possible streams and searching
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions

    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: the exclusive (strict) upper bound on plan cost to successfully terminate

    :param max_time: the maximum runtime
    :param max_iterations: the maximum number of search iterations
    :param max_memory: the maximum amount of memory

    :param initial_complexity: the initial stream complexity limit
    :param complexity_step: the increase in the stream complexity limit per iteration
    :param max_complexity: the maximum stream complexity limit

    :param verbose: if True, print the result of each stream application
    :param search_kwargs: keyword args for the search subroutine

    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan (INF if no plan), and evaluations is init expanded
        using stream applications
    """
    # max_complexity = 0 => current
    # complexity_step = INF => exhaustive
    # success_cost = terminate_cost = decision_cost
    # TODO: warning if optimizers are present


    print("Skeleton: {}".format(skeleton))
    print("Nonfluents: {}".format(nonfluents))

    clear_temp()
    
    evaluations, goal_expression, domain, externals = parse_problem(problem, constraints=constraints, unit_costs=unit_costs)

    problem = get_problem(evaluations, goal_expression, domain, unit_costs)
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory) # TODO: include other info here?

    static_externals = compile_fluents_as_attachments(domain, externals)

    # Create the state in prolog
    disc_atoms = []
    for evaluation in evaluations:
        if(evaluation.head.function.islower() and "-" not in evaluation.head.function):
            disc_atoms.append(Atom(evaluation.head.function, [TypedObject(a, "object") for a in evaluation.head.args]))

    disc_object_set = []
    for atom in disc_atoms:
        disc_object_set = list(set(disc_object_set+list(atom.args)))
    disc_typed_object_set = [TypedObject(obj.name, infer_object_type(parsed_domain, obj, [disc_atoms])) for obj in list(set(disc_object_set))]    

    instantiator = Instantiator(static_externals, evaluations) 


    complexity = max_complexity
    print("Complexity: {}".format(complexity))
    process_stream_queue(instantiator, store, complexity, verbose=verbose)

    # Create the state in prolog
    atoms = []
    for evaluation in evaluations:
        if(evaluation.head.function.islower() and "-" not in evaluation.head.function):
            atoms.append(Atom(evaluation.head.function, [TypedObject(a, "object") for a in evaluation.head.args]))

    # goal expresion to formula
    goal_atoms = []
    assert goal_expression[0] == "and"
    for goal_tuple in goal_expression[1:]:
        if(goal_tuple[0].islower() and "-" not in goal_tuple[0]):
            goal_atoms.append(Atom(goal_tuple[0], [TypedObject(a, "object") for a in goal_tuple[1:]]))
    
    atoms = sorted(atoms, key = lambda x: x.predicate) 

    state_index = int(time.time()*100)
    
    # Evaluate the preconditions of each action for this state in prolog
    task = task_from_domain_problem(domain, problem)

    value_map = None
    tstate = TState(atoms, value_map)
    object_set = []
    for atom in atoms:
        object_set = list(set(object_set+list(atom.args)))

    typed_object_set = [TypedObject(obj.name, infer_object_type(parsed_domain, obj, [atoms])) for obj in list(set(object_set))]
  
    # Objects to strings and strings to objects
    o2s = {obj: get_arg_string(obj) for obj in list(set(typed_object_set))+list(set(object_set))}
    io2s = {get_arg_string(obj): obj for obj in list(set(object_set))}

    # Objects to strings and strings to objects (discrete version)
    disc_o2s = {obj: get_arg_string(obj) for obj in list(set(disc_typed_object_set))+list(set(disc_object_set))}
    disc_io2s = {get_arg_string(obj): obj for obj in list(set(disc_object_set))}

    cont_object_set = set(list(io2s.keys()))-set(list(disc_io2s.keys()))

    stringified_atoms, stringify_dict = get_stringified_atoms(set(list(atoms)), {}, o2s)

    axioms_atoms = []
    encode_nonfluents = True
    nf_pred_extension=state_index
    state_index+=1
    stringified_new_axiom_atoms = stringified_atoms
    add_dynamic=True
    s_axiom_atoms = stringified_new_axiom_atoms
    for axiom in parsed_domain.axioms:
        print(axiom)
        axiom_atoms = evaluate_axiom(axiom, s_axiom_atoms, parsed_domain, state_index, 
                                     typed_object_set, value_map, nonfluents, 
                                     encode_nonfluents=encode_nonfluents,
                                     nf_pred_extension=nf_pred_extension,
                                     add_dynamic=add_dynamic)
        s_axiom_atoms, stringify_dict = get_stringified_atoms(axiom_atoms, stringify_dict, o2s)
        add_dynamic=False
        axioms_atoms+=axiom_atoms
        stringified_new_axiom_atoms, stringify_dict = get_stringified_atoms(set(list(atoms)+list(axioms_atoms)), stringify_dict, o2s)
        encode_nonfluents = False
    state_index+=1

    non_goal = check_goal(stringified_new_axiom_atoms, goal_atoms)
    start_time = time.time()

    # Check if goal already achieved
    if(non_goal == 0):
        return ([], 0, Certificate(all_facts=facts_from_evaluations(evaluations), preimage_facts=None, plan_graph=MCTSGraph([], {})), time.time()-start_time), None

    parent = defaultdict(lambda: None)
    Q = [(MCTSNode(atoms, stringified_atoms, axioms_atoms, stringified_new_axiom_atoms, None, None, None, non_goal, 0, 0))]

    QNodes = copy.deepcopy(Q)
    evaluated = [frozenset(stringified_atoms)]
    disc_evaluated = []
    iteration = 0
    max_depth = 0
    goal_atoms_rem = non_goal
    successor_action_simp = None
    while(len(Q) > 0):
        if(time.time()-start_time>=max_time):
            print("Max Time Exceeded")
            return ([], 0, None, time.time()-start_time), None

        print("Iteration {}".format(iteration))
        iteration+=1
        Q = get_heuristic(Q, mode=heuristic)
        print([q.h for q in Q[:20]])
        print([q.depth for q in Q[:20]])
        # print([q.depth for q in Q])

        q = Q.pop(0)
        print(q.action)
        print(q.successor_action)

        if(q.action is not None):
            # instance = t_get_action_instance(task, action.name, [successor_action_simp[param.name] for param in action.parameters])
            new_atoms, new_stringified_atoms, stringify_dict = apply_action(q.atoms, q.stringified_atoms, stringify_dict, q.action, q.successor_action, io2s, nonfluents=nonfluents)

            new_axioms_atoms = []
            stringified_new_axiom_atoms = new_stringified_atoms
            st = time.time()
            add_dynamic = True
            s_axiom_atoms = stringified_new_axiom_atoms

            exists = frozenset(new_stringified_atoms) in evaluated
            if(not exists):
                evaluated.append(frozenset(new_stringified_atoms))
            else:
                print("Exists")
                continue

            print("Evaluating axioms")
            for axiom in parsed_domain.axioms:
                print(axiom.name)
                st = time.time()
                new_axiom_atoms = evaluate_axiom(axiom, s_axiom_atoms, parsed_domain, 
                                                 state_index, typed_object_set, value_map, nonfluents, 
                                                 nf_pred_extension=nf_pred_extension,
                                                 add_dynamic=add_dynamic)
                s_axiom_atoms, stringify_dict = get_stringified_atoms(new_axiom_atoms, stringify_dict, o2s)
                add_dynamic = False
                new_axioms_atoms += new_axiom_atoms
                stringified_new_axiom_atoms, stringify_dict = get_stringified_atoms(set(list(new_atoms)+list(new_axioms_atoms)), stringify_dict, o2s)
                print(time.time()-st)

            state_index += 1

            #Finally evaluate the order
            order_axiom_names = [a.predicate for a in new_axioms_atoms]
            for effect in q.action.order:
                

                if((isinstance(effect.condition, Atom) ) and effect.condition.predicate in order_axiom_names  or 
                        (isinstance(effect.condition, NegatedAtom) and effect.condition.predicate not in order_axiom_names)):
                    print("Adding order:"+str(effect.literal))
                    new_atoms.add(effect.literal)
                elif(isinstance(effect.condition, Conjunction)):
                    valid=True
                    for cpart in effect.condition.parts:
                        if(  isinstance(cpart, Atom) and cpart.predicate not in order_axiom_names):
                            valid=False
                        elif(isinstance(cpart, NegatedAtom) and cpart.predicate in order_axiom_names):
                            valid=False
                    if(valid):
                        new_atoms.add(effect.literal)


            new_stringified_atoms, stringify_dict = get_stringified_atoms(new_atoms, stringify_dict, o2s)

           
            goal_atoms_rem = check_goal(stringified_new_axiom_atoms, goal_atoms)
            inc_decs_valid = True
            if(q.action is not None):
                inc_decs_valid = check_inc_dec(q.action.incs, q.action.decs, q.stringified_axiom_atoms, stringified_new_axiom_atoms)
                
                if(not inc_decs_valid):
                    print("Inc dec invalid")
                    continue
       
            # print(goal_atoms_rem)
            if(goal_atoms_rem == 0 and q.cost+action_cost <= success_cost):

                goal_node = MCTSNode(new_atoms,
                                     stringified_atoms,
                                     new_axioms_atoms,
                                     new_stringified_axiom_atoms,
                                     None, 
                                     None,
                                     None,
                                     goal_atoms_rem, 
                                     q.cost+action_cost, 
                                     q.depth+1)
                # GOAL
                plan  = [goal_node]
                parent_node = q
                while(parent_node != None):
                    plan.append(parent_node)
                    parent_node = parent[parent_node]

                facts = facts_from_evaluations(evaluations)

                for node in list(reversed(plan)):
                    if(node.action_name is not None):
                        print(node.action_name)
                        print([get_arg_string(a) for a in  node.action_args])
                return (list(reversed(plan))[1:], q.cost+action_cost, Certificate(all_facts=facts, preimage_facts=None, plan_graph=MCTSGraph(QNodes, dict(parent))), time.time()-start_time), None
        
        else:
            new_atoms = q.atoms
            new_axioms_atoms = q.axiom_atoms


        # Evaluate the preconditions of each action for this state in prolog
        for action in parsed_domain.actions:
            print(action.name)
            if(skeleton is None or q.depth<len(skeleton) and skeleton[q.depth][0]==action.name):
                c = objectify_expression(action.precondition)
                c = remove_new_axiom(c)

                # Need to stringify
                new_stringified_axiom_atoms, stringify_dict = get_stringified_atoms(set(list(new_atoms)+list(new_axioms_atoms)), stringify_dict, o2s)
                st = time.time()
                if(skeleton is None):
                    successors = get_successor_gen(new_stringified_axiom_atoms, action.parameters, c, nonfluents = nonfluents, verbose=True)()

                else:
                    successors = get_successor_gen(new_stringified_axiom_atoms, action.parameters, c, nonfluents = nonfluents, current_db = [skeleton[q.depth][1]], verbose=False)()

                # Remove axioms
                stringified_atoms, stringify_dict = get_stringified_atoms(new_atoms, stringify_dict, o2s)

                successors_list = list(frozenset(list(successors)))
                action_cost = get_action_cost(action.name, default_action_cost, action_costs)
                print(len(successors_list))
                if(q.cost + action_cost <= success_cost):
                    # Evaluate node
                    for successor_i, successor_action in enumerate(successors_list):
                        # print(successor_action)
                        successor_action_simp = {k.name: v for k, v in zip(action.parameters, successor_action) }
                        node_params = [io2s[successor_action_simp[param.name]].name.value for param in action.parameters]

                        new_node = MCTSNode(new_atoms,
                                            stringified_atoms,
                                            new_axioms_atoms,
                                            new_stringified_axiom_atoms,
                                            action, 
                                            node_params,
                                            successor_action,
                                            goal_atoms_rem, 
                                            q.cost+action_cost, 
                                            q.depth+1)
                        Q.append(new_node)
                        QNodes.append(new_node)
                        parent[new_node] = q

    print("No plan found")
    return ([], 0, None, max_time), None



