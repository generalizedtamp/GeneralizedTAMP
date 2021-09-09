import copy
import os
import time
from collections import defaultdict
import itertools
import pickle 
import pyswip
import importlib
from collections import namedtuple

from sympy import symbols, simplify_logic
from sympy import Not as SymNot
from sympy import Or as SymOr
from sympy import And as SymAnd

from GeneralizedTAMP.pddlstream.pddlstream.algorithms.algorithm import parse_stream_pddl
from GeneralizedTAMP.pddlstream.pddlstream.language.attachments import has_attachments, \
    compile_fluents_as_attachments, solve_pyplanners
from GeneralizedTAMP.pddlstream.pddlstream.algorithms.common import evaluations_from_init, \
    add_certified, add_facts, UNKNOWN_EVALUATION
from GeneralizedTAMP.pddlstream.pddlstream.language.conversion import value_from_evaluation
from GeneralizedTAMP.pddlstream.pddlstream.algorithms.instantiation import Instantiator
from pddl import Atom, NegatedAtom, Conjunction, UniversalCondition, \
    ExistentialCondition, TypedObject
from GeneralizedTAMP.pddlstream.pddlstream.utils import read

from language.planner import parse_sequential_domain,\
    get_streams_certified_pddl
from database import TDatabase, Transition, Plan
from opensat_interface import solve_sat
from pyeda.inter import Or, Not, And, exprvar
import argparse
from utils import save_features, save_qnp_file, get_experiment_dict,\
    create_new_task_domain, get_subsumption, print_concept, Implication,\
    GenTAMPProblem, powerset, np_print_repr, convert_to_sympy, convert_to_prolog, \
    convert_to_pyeda, BOOLEAN, NUMERIC, Feature, create_new_task_domain, print_feature, \
    get_parts_flat, encode_state_kb, populate_state_prolog, \
    create_kb, findall_query_raw, infer_object_type, get_arg_string, \
    to_generic_typed_atom, count_negations, sign
    
from stream_utils import process_instance, process_stream_queue
from tasks.carry.problem import Carry
from tasks.sort.problem import Sort
from tasks.clutter.problem import Clutter
import numpy as np
import errno



def primitive_negation(concepts, complexity, negation_complexity=1, nonfluents=[]):
    for concept in concepts:
        if (isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
            if(concept.predicate not in nonfluents):
                yield concept.negate()
    return


def get_params(c):
    if (isinstance(c, UniversalCondition) or
            isinstance(c, ExistentialCondition)):
        return list(set(c.parameters))
    else:
        return []


def get_name_index(obj_name):
    return int(obj_name.split("y")[1])


def get_concept_types(concept):
    total_args = []
    concept_types = defaultdict(lambda: [])
    max_name = -1
    for a in get_parts_flat(concept):
        for arg in a.args:
            total_args.append(arg.name)
            concept_types[arg.type_name].append(arg.name)
            if(get_name_index(arg.name) > max_name):
                max_name = get_name_index(arg.name)
    return set(total_args), concept_types, max_name

def get_concept_nonfluents(concept, nonfluents):
    print("NONFLUENTS")
    print(nonfluents)

    if (isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        print("Concept predicate")
        print(concept.predicate)
        print(type(concept.predicate))
        return [int(concept.predicate in nonfluents)]
    else:
        args = []
        for part in concept.parts:
            args+=get_concept_nonfluents(part, nonfluents)
        return args

def num_concept_nonfluents(concept, nonfluents):
    return sum(get_concept_nonfluents(concept, nonfluents))

def get_concept_variables(concept):
    if (isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return concept.args
    else:
        args = []
        for part in concept.parts:
            args.append(num_concept_variables(part))
        return args

def num_concept_variables(concept):
    return len(get_concept_variables(concept))

def populate_concept_variables(concept, variable_dict):

    if (isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        new_arg_list = [TypedObject(variable_dict[arg.name], arg.type_name)
                        for arg in concept.args]
        if(isinstance(concept, Atom)):
            return Atom(concept.predicate,  new_arg_list)
        elif(isinstance(concept, NegatedAtom)):
            return NegatedAtom(concept.predicate,  new_arg_list)
    else:
        assert isinstance(concept, Conjunction)
        new_parts = []
        for part in concept.parts:
            new_parts.append(populate_concept_variables(part, variable_dict))
        return Conjunction(new_parts)

def argument_alignment(concept):
    """
        Returns the placeholder name aligned concept the number
            of unique arguments
        TODO: make funcitonal
    """
    concept = copy.deepcopy(concept)
    flat_parts = get_parts_flat(concept)
    name_map = {}
    for cpart in flat_parts:
        for arg in cpart.args:
            if(arg.name not in name_map.keys()):
                name_map[arg.name] = "?y"+str(len(name_map.keys()))
            arg.name = name_map[arg.name]
    return concept, len(name_map.keys())


def make_logical_formula(concept1, concept2):

    MAX_SHARED_TYPE = 2

    concept1_args, c1_concept_types, max_name = get_concept_types(concept1)
    _, c2_concept_types, _ = get_concept_types(concept2)

    # Enforce maximum number of shared types
    combined_concept_types = copy.deepcopy(c1_concept_types)
    for k, v in c2_concept_types.items():
        combined_concept_types[k] += c2_concept_types[k]

    product_list = []
    # Encode object placeholders for the flattened concept2 tree
    arg_name_list = []
    for atom in get_parts_flat(concept2):
        for ai, primitive_arg in enumerate(atom.args):
            if(primitive_arg.name not in arg_name_list):
                arg_name_list.append(primitive_arg.name)
                shared_type = combined_concept_types[primitive_arg.type_name]
                if(len(shared_type) > MAX_SHARED_TYPE):
                    product_list.append(list(set(c1_concept_types[primitive_arg.type_name])))
                else:
                    max_name = max_name + 1
                    new_name = "?y" + str(max_name)
                    product_list.append(list(set(c1_concept_types[primitive_arg.type_name])) + [new_name])

    # Recreate the concept2 tree with new object placeholders
    for arg_names in itertools.product(*product_list):
        # Enforce at least one intersection point for new logical formulas
        if(len(set(concept1_args).intersection(set(arg_names))) > 0 or \
                len(concept1_args) == 0 or len(arg_names) == 0): 
            vardict = {old: arg_names[i] for i, old in enumerate(arg_name_list)}
            new_concept2 = populate_concept_variables(concept2, vardict)
            new_concept = [concept1, new_concept2]
            yield new_concept


def concept_conjunction(cs, complexity):
    for ci1, c1 in enumerate(cs):
        if(not isinstance(c1, Conjunction) and not isinstance(c1, Implication)):
            for ci2, c2 in enumerate(cs):
                if(not isinstance(c2, Implication)
                        and len(get_parts_flat(c1)) +
                        len(get_parts_flat(c2)) <= complexity):

                    for formula in make_logical_formula(c2, c1):
                        if(formula):
                            cf = Conjunction(formula)
                            concept, new_complexity = argument_alignment(cf)
                            if(new_complexity <= complexity and count_negations(concept)<=1):
                                yield concept

def args_implies(c1, c2):
    c1_args = set(get_concept_variables(c1))
    c2_args = set(get_concept_variables(c2))

    return c2_args.issubset(c1_args)


def concept_implication(cs, complexity, nonfluents=[]):
    for ci1, c1 in enumerate(cs):
        if(not isinstance(c1, Implication)):
            for ci2, c2 in enumerate(cs):
                if(not isinstance(c2, Implication)
                        and len(get_parts_flat(c1)) +
                        len(get_parts_flat(c2)) <= complexity):
                        # and num_concept_nonfluents(c1, nonfluents) == 0):
                    for formula in make_logical_formula(c1, c2):
                        if(formula and args_implies(c1, c2)):
                            df = Implication(*formula)
                            concept, new_complexity = argument_alignment(df)
                            if(new_complexity <= complexity and count_negations(concept)<=1):
                                yield concept


def universal_conjunction(roles, concepts):
    for r in roles:
        for c in concepts:
            yield UniversalCondition(["?y"], r.parts + c.parts)


def existential_conjunction(roles, concepts):
    for r in roles:
        for c in concepts:
            yield ExistentialCondition(["?y"], r.parts + c.parts)


def iterate_transitions(problem_transitions):
    for problem in problem_transitions:
        for ti in range(len(problem)):
            yield problem[ti]


def iterate_transition_pairs(problem_transitions):
    for problem in problem_transitions:
        for ti in range(len(problem)):
            for tj in range(ti + 1, len(problem)):
                yield problem[ti], problem[tj]


def iterate_successor_pairs(problem_transitions):
    for problem in problem_transitions:
        for ti in range(len(problem) - 1):
            yield problem[ti], problem[ti + 1]


def iterate_transition_quats(problem_transitions):
    for problem in problem_transitions:
        for ti in range(len(problem) - 1):
            for tj in range(ti + 1, len(problem) - 1):
                yield problem[ti], problem[tj], problem[ti + 1], \
                      problem[tj + 1]


def check_args_match_atom(atom, arg_dict):
    indep = atom.args[0] == arg_dict["?x"]
    deps = [atom.args[1] == arg_dict["?y"+str(i)] for i in range(len(arg_dict.items())-1)]
    return indep and all(deps)

def flatten_dd_matrix(dd):
    flattened_matrix = {}
    for ddk1, ddv1 in dd.items():
        for ddk2, ddv2 in ddv1.items():
            flattened_matrix[(ddk1, ddk2)] = ddv2
    return flattened_matrix


def is_constant(c):
    if (isinstance(c, Conjunction) or isinstance(c, Implication) or isinstance(c, ExistentialCondition) or isinstance(c, UniversalCondition)):
        for part in c.parts:
            if (not is_constant(part)):
                return False
    elif (isinstance(c, Atom) or isinstance(c, NegatedAtom)):
        if (len(c.args) == 0):
            return True
        else:
            return False

    elif (isinstance(c, Dist)):
        return False
    else:
        raise NotImplementedError
    return True


def prune_concepts(concepts, plans, unpruned, kb):
    signatures = []
    sig_map = {}
    isig_map = {}
    newC = []
    for ci, c in enumerate(concepts):
        sig = []
        fs, qmap = mixed_quantification([c], plans, kb)
        sig = tuple([tuple(results for results in v.values()) for v in qmap.values()])
        if(tuple(sig) not in signatures):
            sig_map[c] = tuple(sig)
            isig_map[tuple(sig)] = c
            signatures.append(tuple(sig))
            newC.append(c)
            print("{}/{}: {} Saved".format(ci, len(concepts), print_concept(c)))
        else:
            print("{}/{}: {} Pruned {}".format(ci, len(concepts), print_concept(c), print_concept(isig_map[tuple(sig)])))
            print([[len(si) for si in s] for s in sig])
            print()


    return newC


def get_all_concept_atoms(concept):
    if(isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return [atom_to_symbol_pyeda(concept)]
    elif(isinstance(concept, Conjunction) or isinstance(concept, Implication)):
        total = []
        for cpart in concept.parts:
            total += get_all_concept_atoms(cpart)
        return total


def get_all_concept_args(concept):
    if(isinstance(concept, Atom) or isinstance(concept, NegatedAtom)):
        return concept.args
    elif(isinstance(concept, Conjunction) or isinstance(concept, Implication)):
        total = []
        for cpart in concept.parts:
            total += get_all_concept_args(cpart)
        return set(total)


def findall_query(free_args, transition, base_concept, kb):
    results, _ = findall_query_raw(free_args, transition, base_concept, kb)
    results = list(set([str(result) for result in results]))
    return results

def mixed_quantification(C, plans, kb, verbose=False):
    QuantMap = defaultdict(lambda: {})
    features = []
    for ci, c in enumerate(C):
        timedout=False
        if(verbose):
            print()
            print("Quantifying {}/{}".format(ci, len(C)))
            print(print_concept(c))
        arg_set = get_all_concept_args(c)
        for free_arg in list(arg_set):
            bound_arg_set = set(arg_set)-set([free_arg])
            for ext in powerset(bound_arg_set):
            # ext = {}
                all_set = set(bound_arg_set) - set(ext)
                f = Feature(c, free_arg, frozenset(all_set), frozenset(ext))
                newc = copy.deepcopy(c)
                for ext_arg in list(ext):
                    newc = ExistentialCondition([ext_arg], [newc])
                for all_arg in list(all_set):
                    newc = UniversalCondition([all_arg], [newc])

                for transitions in plans:
                    for transition in transitions:
                        results = findall_query([free_arg], transition, newc, kb)
                        QuantMap[transition.index][f] = frozenset(results)
              
                # print([len(QuantMap[transition.index][f]) for transition in transitions])
                if(f not in features):
                    features.append(f)

        if(timedout):
            continue

    return features, QuantMap

def generate_concepts(parsed_domain, max_complexity, plans, kb, prune=True, nonfluents=[]):

    primitive_concepts = [Atom(c.name, [TypedObject("?y"+str(i), a.type_name) for i, a in enumerate(c.arguments)]) \
                            for c in parsed_domain.predicates if len(c.arguments)<=max_complexity]
                            
    C = copy.deepcopy(primitive_concepts) + []

    logical_eq_dict = defaultdict(lambda: None)
    complexity_map = defaultdict(lambda: 1)

    # Initialize the logical equiv dictionary
    for c in C:
        expression = simplify_logic(convert_to_sympy(c))
        if(logical_eq_dict[expression] is None):
            logical_eq_dict[expression] = c
            complexity_map[c] = num_concept_variables(c)

    print("k:0, |C|:" + str(len(C)))
    for complexity in range(2, max_complexity):
        print("Generating...")
        concept_negation_rule = primitive_negation(copy.deepcopy(C), complexity, nonfluents=nonfluents)
        concept_conjunction_rule = concept_conjunction(copy.deepcopy(C), complexity)
        concept_implication_rule = concept_implication(copy.deepcopy(C), complexity, nonfluents=nonfluents)
        rules = [concept_negation_rule, concept_conjunction_rule, concept_implication_rule]
        for ri, rule in enumerate(rules):
            print("Rule {}/{}".format(ri, len(rules)))
            rules_list = list(rule)
            for ci, new_concept in enumerate(rules_list):
                print("Concept {}/{}: {}".format(ci, len(rules_list), print_concept(new_concept)))
                new_complexity = len(get_parts_flat(new_concept))
                if(new_complexity <= complexity):
                    expression = simplify_logic(convert_to_sympy(new_concept))
                    if(logical_eq_dict[expression] is None):
                        logical_eq_dict[expression] = new_concept
                        complexity_map[new_concept] = complexity
        newC = list(logical_eq_dict.values())

        print("Pruning...")
        C = newC
        # C = prune_concepts(newC, plans, C, kb)

        print("k:" + str(complexity) + ", |C|:" + str(len(C)))

    # TODO: Remove duplicate quantification
    print("Quantifying Concepts")
    features, QuantMap = mixed_quantification(C, plans, kb, verbose=True)

    return features, dict(QuantMap), dict(complexity_map)



def infer_state_types(inits, typed_objects):
    typed_object_map = {to.name: to for to in typed_objects}
    new_atoms = []
    for atom in inits:
        new_atoms.append(Atom(atom.predicate, [typed_object_map[arg] for arg in atom.args]))
    return new_atoms


def get_object_list(tstates, domain_predicates):
    objects_list = []
    for tsn, tstate in enumerate(tstates):
        for atom in tstate.atoms:
            if(atom.predicate in domain_predicates):
                objects_list += list(atom.args)
    return objects_list

def load_transitions(parsed_domain, database):
    plans = []
    domain_predicates = [p.name for p in parsed_domain.predicates]
    count = 0
    
    # Loop through examples in the database
    for plan_name, plan_data in database.plans.items():
        concrete_states = []
        value_maps = []
        for tstate in plan_data.tstates:
            concrete_state = [atom for atom in tstate.atoms if atom.predicate in domain_predicates]
            concrete_states.append(concrete_state)
            value_maps.append(tstate.value_map)

        objects_list = get_object_list(plan_data.tstates, domain_predicates)

        typed_object_set = [TypedObject(obj, infer_object_type(parsed_domain, obj, concrete_states)) for obj in list(set(objects_list))]
        
        plan = Plan(parsed_domain, plan_name)
        for i in range(len(concrete_states)):
            goal = (i == len(concrete_states) - 1)
            if (not goal):
                action = plan_data.tactions[i]
            else:
                action = None

            typed_state = infer_state_types(concrete_states[i], typed_object_set)
            plan.add_transition(action, typed_state, value_maps[i], typed_object_set,
                           goal=goal, index=count)
            count += 1

        plans.append(plan)

    return plans

def populate_dynamic_prolog(parsed_domain, plans):

    """
        This function introduces dynamic variables for every possible predicate
        to avoid prolog errors when a predicate doesn't exist in a state
    """
    kb_lines = []
    total_states = sum([len(plan) for plan in plans])
    for si in range(total_states):
        kb_lines+=populate_state_prolog(parsed_domain, si)
    return kb_lines
    


def custom_debug(plans, kb):
    # custom_debug_concept = UniversalCondition(["_y0", Conjunction([  Atom("contain", ['_y1', '_y2'] ) ])])
    custom_debug_concept = None
    free_arg = None
    mixed_quantification(custom_debug_feature, plans, kb)
    for transitions in plans:
        for ti, transition in enumerate(transitions):
            results = findall_query([free_arg], transition, custom_debug_concept, kb)
            print("{}: {}".format(ti, results))
            

def compile_stream_facts(plan, exp_dict, max_complexity = 0):
    typed_domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "typed_domain.pddl")
    domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "domain.pddl")
    train_task_problem_file = exp_dict['task_problem_folder'] + plan.problem_name + ".pddl"
    task_domain_file = exp_dict['domain_file']
    task_domain = parse_sequential_domain(read(task_domain_file))
    problem = (globals()[exp_dict['task_name']])(task_domain, 
                                                 train_task_problem_file,
                                                 domain_fn,
                                                 typed_domain_fn)
    np.set_printoptions(precision=15)
    stream_fn = os.path.join(exp_dict['gen_tamp_folder'], "stream.pddl")
    spec = problem.get_problem_spec()
    _, _, _, stream_map_dict, _, _ = problem.pddlstream_from_spec(spec)
    streams = parse_stream_pddl(read(stream_fn), stream_map_dict)
    static_externals = compile_fluents_as_attachments(plan.task_domain, streams)
    sample_init = plan[0].get_value_state()
    evaluations = evaluations_from_init(sample_init)
    old_evals = copy.deepcopy(evaluations)
    instantiator = Instantiator(static_externals, evaluations)
    print(evaluations)
    print("processing stream queue with complexity: {}".format(max_complexity))
    _ = process_stream_queue(instantiator, evaluations, complexity_limit=max_complexity)
    print("finished processing stream queue")

    sample_init_tuples = []
    for ev in sample_init:
        stringed_objs = []
        for obj in list(ev):
            stobj = get_arg_string(obj)
            stringed_objs.append(stobj)

        sample_init_tuples.append(tuple(stringed_objs))


    all_facts = list(map(value_from_evaluation, evaluations))
    value_map = {}
    for ev in all_facts:
        stringed_objs = []
        for obj in list(ev):
            stobj = get_arg_string(obj)
            value_map[stobj] = obj
            stringed_objs.append(stobj)

        if(tuple(stringed_objs) not in sample_init_tuples):
            plan.stream_facts.append(to_generic_typed_atom(tuple(stringed_objs)))

    plan.stream_value_map.update(value_map)
    return plan


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name', default='stack_0', help='')
    # Load in a previous feature pool or create a new one
    parser.add_argument('-v', '--load', action="store_true", help='')
    parser.add_argument('-d', '--debug', action="store_true", help='')
    # Use cnf to check if no issues / wcnf for a compressed feature rep
    parser.add_argument('-m', '--mode', default="wcnf", help='')
    parser.add_argument('-mc', '--max_complexity', type=int, default=5, help='')


    args = parser.parse_args()
    exp_dict = get_experiment_dict(args.exp_name)

    MODE = args.mode
    USE_DISTANCE = False

    load_previous_features = args.load
    save_new_features = not args.load

    database = TDatabase(exp_dict)
    database.load()
    nonfluents = list(database.plans.values())[0].spec.nonfluents
    domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "typed_domain.pddl")
    parsed_domain = parse_sequential_domain(read(domain_fn))

    # Generate Plans for a number of Recycle problems
    plans = load_transitions(parsed_domain, database)

    # Compile non-fluent stream facts
    for plan in plans:
        # new_plan = compile_stream_facts(plan, exp_dict, max_complexity = exp_dict['nf_complexity']) # TODO: adaptable complexity
        print("Pruning redundant types")
        parsed_domain = plan.prune_redundant_types(parsed_domain)

    # Encode the knowledgebases into the states
    kb_lines = []
    for transitions in plans:
        for transition in transitions:
            kb_lines+=encode_state_kb(transition.state, transition.index)

    kb_lines = populate_dynamic_prolog(parsed_domain, plans) + kb_lines
    kb = create_kb(kb_lines)

    if(args.debug):
        custom_debug(plans, kb)
        sys.exit()

    pool_filename = os.path.join(exp_dict['gen_tamp_results_folder'], "GenTAMPFeaturePool.pkl")
    if(load_previous_features):
        with open(pool_filename, 'rb') as handle:
            gtp = pickle.load(handle)
        features, quantified_concept_map, complexity_map = \
            gtp.features, gtp.quantified_concept_map, gtp.complexity_map

    else:
        # Steps of the generative grammar
        features, quantified_concept_map, complexity_map = \
            generate_concepts(parsed_domain, exp_dict['feature_complexity'], plans, kb, nonfluents=nonfluents)

        gtp = GenTAMPProblem(features, quantified_concept_map, complexity_map)
        if(save_new_features):
            with open(pool_filename, 'wb') as handle:
                pickle.dump(gtp, handle, protocol=pickle.HIGHEST_PROTOCOL)

    print("Generating D1 Matrix")
    # Sample 1 to sample 2 to feature list
    boolean_numeric_mapping = defaultdict(lambda: BOOLEAN)
    D1_matrix = defaultdict(lambda: defaultdict(list))

    # Generate Matrix of t, s transitions and their differentials
    for ti, ts in iterate_transition_pairs(plans):

        D1_matrix[ti.index][ts.index] = []
        # Check each of the transitions with each of the features to get counts
        for feature in features:

            object_set_i = quantified_concept_map[ti.index][feature]
            object_set_j = quantified_concept_map[ts.index][feature]

            if ((len(object_set_i) > 0) != (len(object_set_j) > 0)):
                D1_matrix[ti.index][ts.index].append(feature)
            if (len(object_set_i) > 1 or len(object_set_j) > 1):
                boolean_numeric_mapping[feature] = NUMERIC
            if (is_constant(feature.concept)):
                boolean_numeric_mapping[feature] = BOOLEAN

    print("Generating D2 Matrix")
    D2_matrix = defaultdict(lambda: defaultdict(list))
    for ti, si, tii, sii in iterate_transition_quats(plans):
        # Check each of the transitions with each of the features to get counts
        for feature in features:
            object_set_i = quantified_concept_map[ti.index][feature]
            object_set_j = quantified_concept_map[si.index][feature]

            object_set_ipo = quantified_concept_map[tii.index][feature]
            object_set_jpo = quantified_concept_map[sii.index][feature]
            if (sign(len(object_set_i) - len(object_set_ipo)) !=
                    sign(len(object_set_j) - len(object_set_jpo))):
                D2_matrix[ti.index][si.index].append(feature)

    flatd1 = flatten_dd_matrix(D1_matrix)
    flatd2 = flatten_dd_matrix(D2_matrix)

    NUM_FEATURE_VARS = len(features)
    NUM_D1_VARS = len(flatd1.keys())
    NUM_D2_VARS = len(flatd2.keys())
    FEATURE_START = 1
    D1_START = FEATURE_START + NUM_FEATURE_VARS
    D2_START = FEATURE_START + NUM_FEATURE_VARS + NUM_D1_VARS
    NUM_VAR = FEATURE_START + NUM_FEATURE_VARS + NUM_D1_VARS + NUM_D2_VARS

    inverse_concept_map = {}
    for fi, feature in enumerate(features):
        inverse_concept_map[feature] = fi + FEATURE_START

    inverse_d1_map = {}
    inverse_d2_map = {}

    # Write the D1 clauses
    clause_lines = []
    print("Num D1 Clauses:" + str(len(flatd1.items())))
    for i, (key, concepts) in enumerate(flatd1.items()):
        inverse_d1_map[key] = D1_START + i
        to_write = [-(D1_START + i)]
        for ci, concept in enumerate(concepts):
            to_write.append(inverse_concept_map[concept])

            # Also need to append
            neg_to_write = [-(inverse_concept_map[concept]), (D1_START + i), 0]
            clause_lines.append(neg_to_write)

        to_write.append("0")
        clause_lines.append(to_write)

    print("Num D2 Clauses:" + str(len(flatd2.items())))
    for i, (key, concepts) in enumerate(flatd2.items()):    
        inverse_d2_map[key] = D2_START + i
        to_write = [-(D2_START + i)]
        for ci, concept in enumerate(concepts):
            to_write.append(inverse_concept_map[concept])

            # Also need to append
            neg_to_write = [-(inverse_concept_map[concept]), (D2_START + i), 0]
            clause_lines.append(neg_to_write)

        to_write.append(0)
        clause_lines.append(to_write)

    # Consistency clauses
    for ki, (k, d2v) in enumerate(inverse_d2_map.items()):
        if (k in inverse_d1_map.keys()):
            d1v = inverse_d1_map[k]
            clause_lines.append([d1v, -d2v, 0])

    # Goal clauses
    for ti, ts in iterate_transition_pairs(plans):
        if ((ti.goal and not ts.goal) or (not ti.goal and ts.goal)):
            clause_lines.append([inverse_d1_map[(ti.index, ts.index)], 0])

    for feature in features:
        # print(print_feature(feature))
        q = []
        for transitions in plans:
            nq = []
            for ti in transitions:
                nq.append(len(quantified_concept_map[ti.index][feature]))
            # print(nq)
            q+=nq


    # Write the D2 Clauses
    # Since this is constrained optimization, we want the hard constraint
    # weight to be >= num features * MAX_COMPLEXITY
    MAX_WEIGHT = len(features) * exp_dict['feature_complexity']
    result_string = solve_sat(clause_lines, features=features,
                              complexity_map=complexity_map,
                              mode=MODE,
                              max_weight=MAX_WEIGHT,
                              num_vars=NUM_VAR)

    selected_feature_ids = list(filter(lambda x: x > 0, result_string[:NUM_FEATURE_VARS]))
    print("Num selected features: " + str(len(selected_feature_ids)))


    selected_features = list(map(lambda y: features[y - FEATURE_START], selected_feature_ids))

    for feature in selected_features:
        print(print_feature(feature))
        for transitions in plans:
            nq = []
            for ti in transitions:
                nq.append(len(quantified_concept_map[ti.index][feature]))
            print(nq)

    features, QuantMap = mixed_quantification([f.concept for f in selected_features], plans, kb, verbose=True)


    for sfid, selected_feature in zip(selected_feature_ids, selected_features):
        print((sfid - FEATURE_START), print_feature(selected_feature))

    # Extract actions and transitions from the solution
    to_zero = defaultdict(lambda: defaultdict(lambda: False))

    # Loop through each of the transitions and
    # translate the effects and preconditions
    for ti, tii in iterate_successor_pairs(plans):
        for selected_feature in selected_features:
            object_set_i = quantified_concept_map[ti.index][selected_feature]
            ti.set_precondition(selected_feature, boolean_numeric_mapping[selected_feature], int(len(object_set_i) > 0))

        # Look at the change in selected features between transitions
        local_to_zero = defaultdict(lambda: False)
        for selected_feature in selected_features:
            object_set_i = quantified_concept_map[ti.index][selected_feature]
            object_set_j = quantified_concept_map[tii.index][selected_feature]
            diff_ij = len(object_set_j) - len(object_set_i)
            ti.set_effect(selected_feature, boolean_numeric_mapping[selected_feature], diff_ij)
            if (diff_ij < 0 and len(object_set_j) == 0):
                local_to_zero[selected_feature] = True

        for f in selected_features:
            to_zero[ti.effects_tuple][f] = to_zero[ti.effects_tuple][f] or local_to_zero[f]

    # Create abstract actions from transitions with unique effects
    abstract_actions = defaultdict(lambda: [])
    concrete_actions = defaultdict(lambda: [])
    for transition in iterate_transitions(plans):
        if (not transition.goal):
            abstract_actions[transition.effects_tuple].append(transition.preconditions)
            concrete_actions[transition.effects_tuple].append(transition.action)

    for effect, preconditions in abstract_actions.items():
        # Look for duplicate preconditions
        new_precondition = {}
        delcs = []
        for c in preconditions[0].keys():

            for precondition in preconditions:
                if (c not in new_precondition.keys()):
                    new_precondition[c] = precondition[c]
                else:
                    if (new_precondition[c] != precondition[c] and c not in delcs):
                        # Contradictory, remove
                        delcs.append(c)

        for delc in delcs:
            del new_precondition[delc]
        abstract_actions[effect] = new_precondition

    # Step 1: Translate the initial and goal states in terms of the selected features
    qnp_initial_state = get_subsumption(plans, 0, selected_features, quantified_concept_map)
    print("qnp_initial_state")
    print(qnp_initial_state)
    qnp_goal_state = get_subsumption(plans, -1, selected_features, quantified_concept_map)
    qnp_actions = []
    i = 0


    # Step 2: Translate the actions into QNP actions
    for effects, preconditions in abstract_actions.items():

        # Look for duplicate preconditions
        print("==Action#" + str(i) + "==")
        qnp_action = defaultdict(lambda: [])
        print("preconditions:")
        for c, (b, v) in preconditions.items():
            true_wording_map = {BOOLEAN: "true", NUMERIC: " > 0"}
            false_wording_map = {BOOLEAN: "false", NUMERIC: " == 0"}
            if (v > 0):
                print(print_feature(c) + " --> " + str(true_wording_map[b]))
            else:
                print(print_feature(c) + " --> " + str(false_wording_map[b]))
            qnp_action['precondition'].append((c, v > 0))

        print("effects:" + str(len(effects)))
        for c, (b, v) in effects:
            true_wording_map = {BOOLEAN: "true", NUMERIC: " inc"}
            false_wording_map = {BOOLEAN: "false", NUMERIC: " dec"}

            if (v > 0):
                print(print_feature(c) + " --> " + str(true_wording_map[b]))
            else:
                print(print_feature(c) + " --> " + str(false_wording_map[b]))

            if(v != 0):
                qnp_action['effect'].append((c, v > 0))
        i += 1

        
        qnp_actions.append(qnp_action)

    save_qnp_file(qnp_initial_state, qnp_goal_state, qnp_actions, selected_features, boolean_numeric_mapping,
                  filename=os.path.join(exp_dict['gen_tamp_results_folder'], "abstraction.qnp"))
    save_features(selected_features, directory=exp_dict['gen_tamp_results_folder'])
