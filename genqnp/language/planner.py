import os
import sys
from collections import namedtuple

from GeneralizedTAMP.genqnp.language.pddl_structures import Domain, Problem
from GeneralizedTAMP.utils import *


# Fast downward translation requires command line arguments

def find_build(fd_path):
    for release in ['release']:  # TODO: list the directory
        path = os.path.join(fd_path, 'builds/{}/'.format(release))
        if os.path.exists(path):
            return path
    # TODO: could also just automatically compile
    raise RuntimeError('Please compile FastDownward first')


FD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../downward/')
print(FD_PATH)
TRANSLATE_PATH = os.path.join(find_build(FD_PATH), 'bin/translate')
DOMAIN_INPUT = 'domain.pddl'
PROBLEM_INPUT = 'problem.pddl'
TRANSLATE_FLAGS = []
original_argv = sys.argv[:]
sys.argv = sys.argv[:1] + TRANSLATE_FLAGS + [DOMAIN_INPUT, PROBLEM_INPUT]
sys.path.append(TRANSLATE_PATH)

from downward.src.translate.pddl_parser.parsing_functions import parse_task_pddl, \
    parse_condition
import pddl_parser.lisp_parser
import pddl_parser
import subprocess
import os
import re
from collections import namedtuple
import instantiate
import normalize
import pddl
import build_model
import pddl_to_prolog
from pddl_parser.parsing_functions import parse_task_pddl, \
    parse_condition, parse_typed_list, parse_predicate, parse_axiom, \
    parse_function, set_supertypes, _get_predicate_id_and_arity, add_effect,\
    parse_expression, parse_assignment
from pddlstream.language.object import Object, OptimisticObject
from pddlstream.language.external import parse_lisp_list
from pddl import Atom, Action
from utils import NumericAtom, QualitativeAction
sys.argv = original_argv



SEEN_WARNING_TYPE_PREDICATE_NAME_CLASH = False
def _get_predicate_id_and_arity(text, type_dict, predicate_dict):
    global SEEN_WARNING_TYPE_PREDICATE_NAME_CLASH
    num = "num_" in text
    the_type = type_dict.get(text.replace("num_", ""))
    the_predicate = predicate_dict.get(text.replace("num_", ""))

    if the_type is None and the_predicate is None:
        raise SystemExit("Undeclared predicate: %s" % text)
    elif the_predicate is not None:
        if the_type is not None and not SEEN_WARNING_TYPE_PREDICATE_NAME_CLASH:
            msg = ("Warning: name clash between type and predicate %r.\n"
                   "Interpreting as predicate in conditions.") % text
            print(msg, file=sys.stderr)
            SEEN_WARNING_TYPE_PREDICATE_NAME_CLASH = True
        if(num):
            return "num_"+the_predicate.name, the_predicate.get_arity()
        else:
            return the_predicate.name, the_predicate.get_arity()
    else:
        assert the_type is not None
        return the_type.get_predicate_name(), 1

def parse_literal(alist, type_dict, predicate_dict, negated=False):
    if alist[0] == "not":
        assert len(alist) == 2
        alist = alist[1]
        negated = not negated

    pred_id, arity = _get_predicate_id_and_arity(
        alist[0], type_dict, predicate_dict)

    if arity != len(alist) - 1:
        raise SystemExit("predicate used with wrong arity: (%s)"
                         % " ".join(alist))

    if negated:
        return pddl.NegatedAtom(pred_id, alist[1:])
    else:
        if(pred_id.startswith("num_")):
            return NumericAtom(pred_id[4:], alist[1:])
        else:
            return Atom(pred_id, alist[1:])


def parse_effect(alist, type_dict, predicate_dict):
    tag = alist[0]
    if tag == "and":
        return pddl.ConjunctiveEffect(
            [parse_effect(eff, type_dict, predicate_dict) for eff in alist[1:]])
    elif tag == "forall":
        assert len(alist) == 3
        parameters = parse_typed_list(alist[1])
        effect = parse_effect(alist[2], type_dict, predicate_dict)
        return pddl.UniversalEffect(parameters, effect)
    elif tag == "when":
        assert len(alist) == 3
        condition = parse_condition(
            alist[1], type_dict, predicate_dict)
        effect = parse_effect(alist[2], type_dict, predicate_dict)
        return pddl.ConditionalEffect(condition, effect)
    elif tag == "increase":
        assert len(alist) == 3
        assert alist[1] == ['total-cost']
        assignment = parse_assignment(alist)
        return pddl.CostEffect(assignment)
    else:
        # We pass in {} instead of type_dict here because types must
        # be static predicates, so cannot be the target of an effect.
        return pddl.SimpleEffect(parse_literal(alist, {}, predicate_dict))

def parse_effects(alist, result, type_dict, predicate_dict):
    """Parse a PDDL effect (any combination of simple, conjunctive, conditional, and universal)."""
    tmp_effect = parse_effect(alist, type_dict, predicate_dict)
    normalized = tmp_effect.normalize()
    cost_eff, rest_effect = normalized.extract_cost()
    add_effect(rest_effect, result)
    if cost_eff:
        return cost_eff.effect
    else:
        return None


def parse_action(alist, type_dict, predicate_dict):
    iterator = iter(alist)
    action_tag = next(iterator)
    assert action_tag == ":action"
    name = next(iterator)
    parameters_tag_opt = next(iterator)
    if parameters_tag_opt == ":parameters":
        parameters = parse_typed_list(next(iterator),
                                      only_variables=True)
        precondition_tag_opt = next(iterator)
    else:
        parameters = []
        precondition_tag_opt = parameters_tag_opt
    if precondition_tag_opt == ":precondition":
        precondition_list = next(iterator)
        if not precondition_list:
            # Note that :precondition () is allowed in PDDL.
            precondition = pddl.Conjunction([])
        else:
            precondition = parse_condition(
                precondition_list, type_dict, predicate_dict)
        effect_tag = next(iterator)
    else:
        precondition = pddl.Conjunction([])
        effect_tag = precondition_tag_opt
    assert effect_tag == ":effect"
    effect_list = next(iterator)
    eff = []
    if effect_list:
        try:
            cost = parse_effects(effect_list, eff, type_dict, predicate_dict)
        except ValueError as e:
            raise SystemExit("Error in Action %s\nReason: %s." % (name, e))
    
    incs = []
    decs = []
    order = []
    for _ in range(3):
        try:
            incdectag = next(iterator)
        except:
            break

        if(incdectag is not None):
            incdec_list = next(iterator)
            if(incdectag == ":inc"):
                _ = parse_effects(incdec_list, incs, type_dict, predicate_dict)
            if(incdectag == ":dec"):
                _ = parse_effects(incdec_list, decs, type_dict, predicate_dict)
            if(incdectag == ":order"):
                _ = parse_effects(incdec_list, order, type_dict, predicate_dict)
    
    if eff:
        act = Action(name, parameters, len(parameters), precondition, eff, cost)
        return QualitativeAction(source=act, incs = incs, decs = decs, order=order)
    else:
        return None

def parse_domain_pddl(domain_pddl):
    iterator = iter(domain_pddl)

    define_tag = next(iterator)
    assert define_tag == "define"
    domain_line = next(iterator)
    assert domain_line[0] == "domain" and len(domain_line) == 2
    yield domain_line[1]

    ## We allow an arbitrary order of the requirement, types, constants,
    ## predicates and functions specification. The PDDL BNF is more strict on
    ## this, so we print a warning if it is violated.
    requirements = pddl.Requirements([":strips"])
    the_types = [pddl.Type("object")]
    constants, the_predicates, the_functions = [], [], []
    correct_order = [":requirements", ":types", ":constants", ":predicates",
                     ":functions"]
    seen_fields = []
    first_action = None
    for opt in iterator:
        field = opt[0]
        if field not in correct_order:
            first_action = opt
            break
        if field in seen_fields:
            raise SystemExit("Error in domain specification\n" +
                             "Reason: two '%s' specifications." % field)
        if (seen_fields and
            correct_order.index(seen_fields[-1]) > correct_order.index(field)):
            msg = "\nWarning: %s specification not allowed here (cf. PDDL BNF)" % field
            print(msg, file=sys.stderr)
        seen_fields.append(field)
        if field == ":requirements":
            requirements = pddl.Requirements(opt[1:])
        elif field == ":types":
            the_types.extend(parse_typed_list(
                    opt[1:], constructor=pddl.Type))
        elif field == ":constants":
            constants = parse_typed_list(opt[1:])
        elif field == ":predicates":
            the_predicates = [parse_predicate(entry)
                              for entry in opt[1:]]
            the_predicates += [pddl.Predicate("=", [
                pddl.TypedObject("?x", "object"),
                pddl.TypedObject("?y", "object")])]
        elif field == ":functions":
            the_functions = parse_typed_list(
                opt[1:],
                constructor=parse_function,
                default_type="number")
    set_supertypes(the_types)
    yield requirements
    yield the_types
    type_dict = {type.name: type for type in the_types}
    yield type_dict
    yield constants
    yield the_predicates
    predicate_dict = {pred.name: pred for pred in the_predicates}
    yield predicate_dict
    yield the_functions

    entries = []
    if first_action is not None:
        entries.append(first_action)
    entries.extend(iterator)

    the_axioms = []
    the_actions = []
    for entry in entries:
        if entry[0] == ":derived":
            axiom = parse_axiom(entry, type_dict, predicate_dict)
            the_axioms.append(axiom)
        else:
            action = parse_action(entry, type_dict, predicate_dict)
            if action is not None:
                the_actions.append(action)
    yield the_actions
    yield the_axioms

def parse_lisp(lisp):
    return pddl_parser.lisp_parser.parse_nested_list(lisp.splitlines())


def parse_problem(domain, problem_pddl):
    return Problem(*parse_task_pddl(parse_lisp(problem_pddl), domain.type_dict, domain.predicate_dict))


def parse_sequential_domain(domain_pddl):
    domain = Domain(*parse_domain_pddl(parse_lisp(domain_pddl)))
    return domain


def get_stream_certified(lisp_list):
    value_from_attribute = parse_lisp_list(lisp_list)
    certified = value_from_attribute.get(':certified', None)
    return certified


def get_streams_certified(stream_pddl):
    stream_iter = iter(parse_lisp(stream_pddl))
    assert('define' == next(stream_iter))
    pddl_type, pddl_name = next(stream_iter)
    assert('stream' == pddl_type)
    externals = []
    for lisp_list in stream_iter:
        name = lisp_list[0]
        if name == ':stream':
            externals.append(get_stream_certified(lisp_list))
        else:
            continue
    print(externals)
    return externals


NO_INFO = None
RELATIONAL_INFO = 'relational_info' # structural_info
STATISTICS_INFO = 'statistics_info'


def get_streams_certified_pddl(stream_pddl):
    if isinstance(stream_pddl, str):
        stream_pddl = [stream_pddl]

    total_certified = []
    for pddls in stream_pddl:
        # TODO: check which functions are actually used and prune the rest
        total_certified.append(get_streams_certified(pddls))
    return total_certified


if __name__ == '__main__':
    print("Testing pddl parsing ...")
    test_problem_file = "./task_planning_problems/tasks/gridYx/Grid10x.pddl"
    test_domain_file = "./task_planning_problems/tasks/gridYx/GridYx_domain.pddl"
    domain = parse_sequential_domain(read(test_domain_file))
    problem = parse_problem(domain, read(test_problem_file))
    print(domain, problem)

DEFAULT_MAX_TIME = 30  # INF
DEFAULT_PLANNER = 'ff-astar'
FD_PATH = "./downward"
OBJECT = 'object'
Problem = namedtuple('Problem', ['task_name', 'task_domain_name', 'task_requirements',
                                 'objects', 'init', 'goal', 'use_metric'])


class MockSet(object):
    def __init__(self, test=lambda item: True):
        self.test = test

    def __contains__(self, item):
        return self.test(item)


def objects_from_evaluations(evaluations):
    # TODO: assumes object predicates
    objects = set()
    for evaluation in evaluations:
        objects.update(set(list(evaluation[1:])))

    return objects


def fd_from_evaluation(evaluation):
    name = evaluation[0]
    args = evaluation[1:]
    fluent = pddl.f_expression.PrimitiveNumericExpression(symbol=name, args=args)
    expression = pddl.f_expression.NumericConstant(True)
    return pddl.f_expression.Assign(fluent, expression)


def pddl_list_from_expression(tree):
    if isinstance(tree, Object) or isinstance(tree, OptimisticObject):
        return pddl_from_object(tree)
    if isinstance(tree, str):
        return tree
    return tuple(map(pddl_list_from_expression, tree))


def parse_goal(goal_expression, domain):
    return parse_condition(pddl_list_from_expression(goal_expression),
                           domain.type_dict, domain.predicate_dict).simplified()


def get_problem(init_evaluations, goal_expression, domain, unit_costs=False):
    objects = objects_from_evaluations(init_evaluations.all_facts)
    typed_objects = list({pddl.TypedObject(pddl_from_object(obj), OBJECT) for obj in objects} - set(domain.constants))
    # TODO: this doesn't include =
    init = [fd_from_evaluation(e) for e in init_evaluations.all_facts]
    goal = parse_goal(goal_expression, domain)
    return Problem(task_name=domain.name, task_domain_name=domain.name,
                   objects=sorted(typed_objects, key=lambda o: o.name),
                   task_requirements=pddl.tasks.Requirements([]), init=init, goal=goal, use_metric=not unit_costs)


def get_plan_filename(pf_index):
    return "./plan." + str(pf_index)


def plan(domain_file, problem_file):
    FD_BIN = "./downward/fast-downward.py"
    commandline_args = "--plan-file plan --alias seq-sat-lama-2011"
    # commandline_args = ""
    # other_args = "--evaluator \"hff=ff()\" --evaluator \"hcea=cea()\" --search \"lazy_greedy([hff, hcea], preferred=[hff, hcea])\""
    # other_args = "--search \"astar(lmcut())\""
    other_args = ""
    command = "%s %s %s %s %s" % (FD_BIN, commandline_args, domain_file, problem_file, other_args)
    print(command)
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, cwd=None, close_fds=True)
    try:
        output, error = proc.communicate(timeout=10)
        print("Proc Error: " + str(error))
    except:
        print("Timeout")
    pf_index = 1
    # Need to get the latest plan file
    while (os.path.isfile(get_plan_filename(pf_index))):
        pf_index += 1

    plan_file = get_plan_filename(pf_index - 1)
    return plan_file


def find_plan(domain_file, problem_file):
    plan_file = plan(domain_file, problem_file)
    return parse_solution(plan_file)


##################################################
PlanAction = namedtuple('Action', ['name', 'args'])


def parse_plan_action(line):
    entries = line.strip('( )').split(' ')
    name = entries[0]
    args = tuple(entries[1:])
    return PlanAction(name, args)


def parse_solution(solution_file):
    with open(solution_file, 'r') as f:
        solution = f.read()

    # action_regex = r'\((\w+(\s+\w+)\)' # TODO: regex
    if solution is None:
        return None, cost
    cost_regex = r'cost\s*=\s*(\d+)'
    matches = re.findall(cost_regex, solution)
    # TODO: recover the actual cost of the plan from the evaluations
    lines = solution.split('\n')[:-2]  # Last line is newline, second to last is cost
    plan = list(map(parse_plan_action, lines))
    return plan


def literal_holds(state, literal):
    # return (literal in state) != literal.negated
    return (literal.positive() in state) != literal.negated


def conditions_hold(state, conditions):
    return all(literal_holds(state, cond) for cond in conditions)


def apply_action(state, action):
    # print(action.name)
    state = set(state)
    assert (isinstance(action, pddl.PropositionalAction))
    # TODO: signed literals
    del_state = []
    for conditions, effect in action.del_effects:
        if conditions_hold(state, conditions):
            state.discard(effect)

    for conditions, effect in action.add_effects:
        if conditions_hold(state, conditions):
            state.add(effect)

    return state


def task_from_domain_problem(domain, problem):
    task_name, task_domain_name, task_requirements, objects, init, goal, use_metric = problem

    assert domain.name == task_domain_name
    requirements = pddl.Requirements(sorted(set(domain.requirements.requirements +
                                                task_requirements.requirements)))
    objects = domain.constants + objects
    init.extend(pddl.Atom("=", (obj.name, obj.name)) for obj in objects)

    task = pddl.Task(domain.name, task_name, requirements, domain.types, objects,
                     domain.predicates, domain.functions, init, goal,
                     domain.actions, domain.axioms, use_metric)
    normalize.normalize(task)
    # task.add_axiom
    return task


def find_unique(test, sequence):
    found, value = False, None
    for item in sequence:
        if test(item):
            if found:
                raise RuntimeError('Both elements {} and {} satisfy the test'.format(value, item))
            found, value = True, item
    if not found:
        raise RuntimeError('Unable to find an element satisfying the test')
    return value


def pddl_from_object(obj):
    # if isinstance(obj, str):
    # 	return obj
    # return obj.pddl
    return str(obj)


def get_function_assignments(task):
    return {f.fluent: f.expression for f in task.init
            if isinstance(f, pddl.f_expression.FunctionAssignment)}


def t_get_action_instance(task, action_name, atoms):
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = get_function_assignments(task)
    fluent_facts = MockSet()
    init_facts = set()
    action_instances = []
    predicate_to_atoms = {}

    # TODO: what if more than one action of the same name due to normalization?
    # Normalized actions have same effects, so I just have to pick one
    action = find_unique(lambda a: a.name == action_name, task.actions)
    args = list(map(pddl_from_object, atoms))
    assert (len(action.parameters) == len(args))
    variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
    instance = action.instantiate(variable_mapping, init_facts,
                                  fluent_facts, type_to_objects,
                                  task.use_min_cost_metric, function_assignments, predicate_to_atoms)
    assert (instance is not None)
    return instance

def t_get_action_instances(task, action_plan):
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = get_function_assignments(task)
    fluent_facts = MockSet()
    init_facts = set()
    action_instances = []
    predicate_to_atoms = {}

    for name, objects in action_plan:
        # TODO: what if more than one action of the same name due to normalization?
        # Normalized actions have same effects, so I just have to pick one
        action = find_unique(lambda a: a.name == name, task.actions)
        args = list(map(pddl_from_object, objects))
        assert (len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts,
                                      fluent_facts, type_to_objects,
                                      task.use_min_cost_metric, function_assignments, predicate_to_atoms)
        assert (instance is not None)
        action_instances.append(instance)
    return action_instances


def get_action_instances(task, action_plan):
    type_to_objects = instantiate.get_objects_by_type(task.objects, task.types)
    function_assignments = get_function_assignments(task)
    model = build_model.compute_model(pddl_to_prolog.translate(task))
    fluent_facts = instantiate.get_fluent_facts(task, model)
    init_facts = task.init
    action_instances = []
    for name, objects in action_plan:
        # TODO: what if more than one action of the same name due to normalization?
        # Normalized actions have same effects, so I just have to pick one
        action = find_unique(lambda a: a.name == name, task.actions)

        args = list(map(pddl_from_object, objects))
        assert (len(action.parameters) == len(args))
        variable_mapping = {p.name: a for p, a in zip(action.parameters, args)}
        instance = action.instantiate(variable_mapping, init_facts, function_assignments,
                                      fluent_facts, type_to_objects,
                                      task.use_min_cost_metric)
        assert (instance is not None)
        action_instances.append(instance)
    return action_instances
