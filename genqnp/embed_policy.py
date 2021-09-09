import argparse
import argparse
from utils import get_experiment_dict, Feature, remove_implication,\
    quantify_feature, create_new_task_domain, Domain, untype_concept,\
    print_concept, add_type_constraints, parse_qnp_file, QualitativeAction
from language.planner import parse_sequential_domain
from GeneralizedTAMP.pddlstream.pddlstream.utils import read
import os
import pickle
from pddl import Axiom, UniversalCondition, Action, Predicate, \
                Atom, NegatedAtom, Effect, TypedObject, ExistentialCondition, \
                Conjunction
from domain_utils import save_domain, save_typed_domain
import copy
from policy_utils.fondparser import grounder
from policy_utils.normalizer import flatten
from policy_utils.validator import VALAction, \
    _convert_conjunction, _convert_cond_effect, State
import policy_utils.validators.prp as val
from collections import defaultdict
from utils import NumericAtom


def progress(s, o, m):
    assert o.ppres <= s.fluents and 0 == len(o.npres & s.fluents), \
        "Failed to progress %s:\nPrecondition: %s\nState:\n%s" % \
        (o.name, str(o.pres), _state_string(m, s))

    #print "\nProgressing the following operator:"
    #print (o)

    adds = set()
    dels = set()
    for eff in o.effs:
        negeff = set(filter(lambda x: x < 0, eff[0]))
        poseff = eff[0] - negeff
        negeff = set(map(lambda x: x * -1, negeff))
        if (poseff <= s.fluents) and 0 == len(negeff & s.fluents):
            for reff in eff[1]:
                if reff < 0:
                    dels.add(reff * -1)
                else:
                    adds.add(reff)

    if 0 != len(adds & dels):
        print("Warning: Conflicting adds and deletes on action %s" % str(o))

    return State(((s.fluents - dels) | adds))

def get_new_action_set(abstract_actions, 
                       concrete_actions, 
                       dfile, 
                       pfile, 
                       features, 
                       aa_preconditions, 
                       sol, 
                       save_dir,
                       boolean_numeric_mapping):
    print("\nParsing the problem...")
    problem = grounder.GroundProblem(dfile, pfile)
    fluents = {}
    unfluents = {}
    index = 1
    for f in problem.fluents:
        fluents[str(f).lower()] = index
        fluents["not(%s)" % str(f).lower()] = -1 * index
        unfluents[index] = str(f).lower()
        unfluents[-1 * index] = "not(%s)" % str(f).lower()
        index += 1


    actions = {}
    for op in problem.operators:
        if '_' == op.name[-1]:
            op_name = op.name[:-1].lower()
        else:
            op_name = op.name.lower()
       
        actions[op_name] = [VALAction(_convert_conjunction(fluents, op.precondition),
                                      _convert_cond_effect(fluents, eff), op_name, unfluents)
                            for eff in flatten(op)]

    print("problem.init: "+str(problem.init))
    init_state = State(_convert_conjunction(fluents, problem.init))
    goal_state = State([-1])
    goal_fluents = set(_convert_conjunction(fluents, problem.goal))

    print("init state: "+str(init_state))
    open_list = [init_state]

    nodes = {init_state: 1, goal_state: 2}
    node_index = 3
    val.load(sol, fluents)
    unhandled = []
    embedded_actions = []

    state_predicate_map = {init_state: Predicate("state{}".format(0), [])}

    # There is absolutely a better way of doing this...
    resulting_states = defaultdict(list)
    new_embedded_actions = []
    new_embedded_actions_map = {}

    while open_list:
        u = open_list.pop(0)
        a = val.next_action(u)

        if not a:
            unhandled.append(u)
        else:
            for action in concrete_actions:
                new_action = copy.deepcopy(QualitativeAction(source=copy.deepcopy(action)))
                # Add in feature preconditions
                aaction_index = int(a.replace("a", ""))
                aaction_spec = abstract_actions[aaction_index]
                aaction_pre_spec, aaction_eff_spec = aaction_spec
                
                new_action.precondition.parts = tuple(list( new_action.precondition.parts )\
                                                +[Atom(aa_preconditions[aaction_index].name, [])])

                for feature_name, value in aaction_eff_spec.items():
                    feature_index = int(feature_name.replace("f", ""))
                    # Find the first parameter in the action preconditions which matches
                    new_param = features[feature_index].free_arg
                    new_param = TypedObject("?{}{}".format(feature_name,new_param.name[1:]), new_param.type_name)
                    
                    if(boolean_numeric_mapping[feature_name] == 0):
                        # Boolean Feature
                        if(value>0):
                            new_action.incs.append(Atom(feature_name, [new_param]))
                        else:
                            new_action.decs.append(Atom(feature_name, [new_param]))
                    elif(boolean_numeric_mapping[feature_name] > 0):
                        # Numeric Feature
                        if(value>0):
                            new_action.incs.append(NumericAtom(feature_name, [new_param]))
                        else:
                            new_action.decs.append(NumericAtom(feature_name, [new_param]))
                    else:
                        raise NotImplementedError


                # Add abstract action ordering constraints
                new_action.name = "{}_{}".format(new_action.name, state_predicate_map[u].name)
                new_action.precondition.parts = tuple(list( new_action.precondition.parts )+[Atom(state_predicate_map[u].name, [])])
                new_embedded_actions.append(new_action)
                new_embedded_actions_map[new_action] = u

            i = 0
            for outcome in actions[a]:
                v = progress(u, outcome, unfluents)

                resulting_states[u].append((outcome, v))
                if(v not in state_predicate_map):
                    new_state_index = len(list(state_predicate_map.values()))
                    state_predicate_map[v] = Predicate("state{}".format(new_state_index), [])
                
                i += 1

                if v.is_goal(goal_fluents):
                    v = goal_state
                elif v not in nodes:
                    nodes[v] = node_index
                    node_index += 1
                    open_list.append(v)


    # Num unique abstract_states 
    num_unique_astates = len(list(state_predicate_map.values()))

    # For each action in new_actions
    for new_embedded_aciton in new_embedded_actions:
        cslist = resulting_states[new_embedded_actions_map[new_embedded_aciton]]
        print(new_embedded_aciton.name)
        for (c, s) in cslist:
            print()
            condition = []
            for eff in c.effs:
                print(eff)
                orig = list(eff[1])[0]
                assert orig!=0, "Cannot negate 0"
                abs_eff = c.mapping[abs(orig)]
                
                print(abs_eff)
                if(abs_eff.endswith("()")):
                    # Boolean
                    axiom_name = abs_eff.replace("()", "")
                    if(orig>0):
                        condition.append(Atom("p_"+axiom_name, []))
                    else:
                        condition.append(NegatedAtom("p_"+axiom_name, []))
                elif(abs_eff.startswith("zero")):
                    # Numeric
                    axiom_name = abs_eff.replace("zero(","").replace(")","")
                    if(orig<0):
                        condition.append(Atom("p_"+axiom_name, []))
                    else:
                        condition.append(NegatedAtom("p_"+axiom_name, []))
                else:
                    assert False, abs_eff

            astate_predicate = state_predicate_map[s]
            new_embedded_aciton.order.append(Effect([], Conjunction(condition), Atom(astate_predicate.name, [])))
        astate_predicate = state_predicate_map[new_embedded_actions_map[new_embedded_aciton]]
        new_embedded_aciton.effects.append(Effect([], True, NegatedAtom(astate_predicate.name, [])))

    embedded_actions += new_embedded_actions

    return embedded_actions, list(state_predicate_map.values())


def get_aa_preconditions(abstract_actions, features):


    aa_pre_axioms = []
    aa_pre_predicates = []

    for aa_i, aa in enumerate(abstract_actions):
        aaction_spec = abstract_actions[aa_i]
        aa_pre_name = "prec{}".format(aa_i)
        aaction_pre_spec, aaction_eff_spec = aaction_spec
        conds = []
        for feature_name, value in aaction_pre_spec.items():
            feature_index = int(feature_name.replace("f", ""))
            # Find the first parameter in the action preconditions which matches
            if(value>0):
                conds.append(Atom("p_"+feature_name, []))
            else:
                conds.append(NegatedAtom("p_"+feature_name, []))

        a_cond = Conjunction(conds)

        aa_pre_axioms.append(Axiom(aa_pre_name, [], 0, a_cond))
        aa_pre_predicates.append(Predicate(aa_pre_name, []))

    return aa_pre_predicates, aa_pre_axioms

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name', default='stack_0', help='')


    args = parser.parse_args()
    exp_dict = get_experiment_dict(args.exp_name)

    # Load in the domain
    embedded_domain_file = os.path.join(exp_dict['gen_tamp_results_folder'], "embedded_domain.pddl")
    embedded_typed_domain_file = os.path.join(exp_dict['gen_tamp_results_folder'], "embedded_typed_domain.pddl")

    typed_domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "typed_domain.pddl")
    domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "typed_domain.pddl")

    abstraction_fn = os.path.join(exp_dict['gen_tamp_results_folder'], "abstraction.qnp")

    task_domain_args = parse_sequential_domain(read(typed_domain_fn))
    task_domain = Domain(*list(task_domain_args))

    # Load in the abstract actions
    features_filename = os.path.join(exp_dict['gen_tamp_results_folder'], "features.pkl")
    with open(features_filename, 'rb') as handle:
        features = pickle.load(handle)
    

    # Encode each of the features as an axiom
    abstract_axioms = copy.copy(task_domain.axioms)
    axiom_predicates = []
    for feature_index, feature in enumerate(features):
        quant_concept = quantify_feature(feature)
        # type_constrained_concept = add_type_constraints(quant_concept)
        # untyped_concept = untype_concept(quant_concept)
        feature_name = "f{}".format(str(feature_index))
        feature_params = [feature.free_arg]
        feature_cond = remove_implication(quant_concept)
        abstract_axioms.append(Axiom(feature_name, 
                                     feature_params, 
                                     len(feature_params), 
                                     feature_cond))
        abstract_axioms.append(Axiom("p_"+feature_name, [], 0, ExistentialCondition([feature.free_arg], [Atom(feature_name, [feature.free_arg])])))
        
        axiom_predicates.append(Predicate(feature_name, feature_params))
        axiom_predicates.append(Predicate("p_"+feature_name, []))

    # Load in the abstraction qnp and parse into actions
    boolean_numeric_mapping, init, goal, abstract_actions = parse_qnp_file(abstraction_fn)

    # Load in the abstract policy
    PRPFOND_INPUT_FILES = \
        [exp_dict['gen_tamp_results_folder']+"/tmp_fond" + str(f) for f in ["_d.pddl", "_p.pddl"]]

    PRPFOND_TRANSLATE_OUTPUT = "translated_policy.out"

    # For each action, create an abstract version of that action
    dfile, pfile, sol, save_dir = (PRPFOND_INPUT_FILES[0],
                                   PRPFOND_INPUT_FILES[1],
                                   os.path.join(exp_dict['gen_tamp_results_folder'], PRPFOND_TRANSLATE_OUTPUT),
                                   exp_dict['gen_tamp_results_folder'])

    aa_precondition_predicates, aa_precondition_axioms = get_aa_preconditions(abstract_actions, features)
    embedded_actions, new_predicates = get_new_action_set(abstract_actions,
                                                          task_domain_args.actions,
                                                          dfile, 
                                                          pfile,
                                                          features,
                                                          aa_precondition_predicates,
                                                          sol, 
                                                          save_dir,
                                                          boolean_numeric_mapping)


    # TODO: Enforce an action ordering using the preconditions and effects
    axiom_predicates += aa_precondition_predicates
    embedded_predicates = copy.copy(task_domain.predicates)+new_predicates+axiom_predicates

    embedded_task_domain = create_new_task_domain(copy.copy(task_domain), 
                                                  axioms=abstract_axioms+aa_precondition_axioms, 
                                                  actions=embedded_actions,
                                                  predicates=embedded_predicates)

    save_domain(embedded_task_domain, embedded_domain_file)
    save_typed_domain(embedded_task_domain, embedded_typed_domain_file)
    print("Embeddings saved to: ")
    print(embedded_domain_file)
    print(embedded_typed_domain_file)

