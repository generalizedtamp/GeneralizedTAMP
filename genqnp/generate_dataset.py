from collections import namedtuple
import numpy as np
import argparse
import os
from database import TDatabase, TState, TAction, TPlan
from language.planner import apply_action,\
    task_from_domain_problem, \
    t_get_action_instances, get_problem, parse_sequential_domain
from tasks.carry.problem import Carry
from tasks.stack3d.problem import Stack3d
from tasks.sort.problem import Sort
from tasks.clutter.problem import Clutter
from pddlstream.language.conversion import obj_from_value_expression
from pddlstream.language.temporal import parse_domain
from pddlstream.utils import read
import pddl
from utils import get_arg_string, get_experiment_dict
from domain_utils import save_domain
from search import MCTSNode


Certificate = namedtuple('Certificate', ['all_facts', 'preimage_facts'])


def stringify_evals(evals):
    stringified_all_facts = []
    value_map = {}
    for ev in evals.all_facts:
        stringed_objs = []
        for obj in list(ev):
            stobj = get_arg_string(obj)
            value_map[stobj] = obj
            stringed_objs.append(stobj)
        stringified_all_facts.append(tuple(stringed_objs))
    return Certificate(stringified_all_facts, []), value_map

def stringify_atoms(atoms):
    stringified_atoms = []
    value_map = {}
    for atom in atoms:
        strobjs = []
        for arg in atom.args:
            ostr = get_arg_string(arg)
            strobjs.append(ostr)
            value_map[ostr] = arg.name.value
        stringified_atoms.append(tuple([atom.predicate]+strobjs))
    return stringified_atoms, value_map


def stringify_goal(goal):
    new_atoms = []
    for atom in goal[1:]:
        if (isinstance(atom, tuple)):
            new_atom = tuple([str(a) for a in list(atom)])
            new_atoms.append(new_atom)
        else:
            new_atoms.append(str(atom))
    new_goal = [goal[0]] + new_atoms
    return new_goal


def map_to_atoms(tuples):
    return [pddl.Atom(tup[0], tup[1:]) for tup in tuples]


def stringify_plan(plan):
    new_action_plan = []
    for name, objects in plan:
        new_action_plan.append((name, [str(o) for o in objects]))
    return new_action_plan


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name',  default='carry_0', help='')
    parser.add_argument('-v', '--visualize', action="store_true", help='')
    args = parser.parse_args()
    experiments_dict = get_experiment_dict(args.exp_name)
    visaualize_plan = args.visualize

    # Step 2: Create a database of task and motion plans
    task_domain_file = experiments_dict['domain_file']
    task_domain = parse_sequential_domain(read(task_domain_file))
    typed_domain_fn = os.path.join(experiments_dict['gen_tamp_folder'], "typed_domain.pddl")
    domain_fn = os.path.join(experiments_dict['gen_tamp_folder'], "domain.pddl")

    # Overwrite the existing tamp domain file with the untyped version of the typed tamp domain file
    parsed_tamp_domain_typed = parse_sequential_domain(read(typed_domain_fn))
    save_domain(parsed_tamp_domain_typed, domain_fn)
    database = TDatabase(experiments_dict)

    # Step 3: Load or recreate the database folder from the train problems
    for train_problem in experiments_dict['training_problems']:
        train_task_problem_file = experiments_dict['task_problem_folder'] \
                                    + train_problem + ".pddl"
        # Instantiate the problem class from the name
        problem = (globals()[experiments_dict['task_name']])(task_domain, 
                                                             train_task_problem_file,
                                                             domain_fn,
                                                             typed_domain_fn,
                                                             max_complexity=experiments_dict['nf_complexity'])

        plan, cost, evaluations, spec, total_time = problem.plan_problem()
       
        if (visaualize_plan):
            problem.visualize_plan(plan)
        
        if(isinstance(plan[0], tuple)):
            plan = stringify_plan(plan)
            evaluations, eval_value_map = stringify_evals(evaluations)
            tdomain = parse_domain(read(problem.get_domain()))
            goal_expression = obj_from_value_expression(problem.get_goal())
            goal_expression = stringify_goal(goal_expression)
            tamp_problem = get_problem(evaluations, goal_expression, tdomain)
            task = task_from_domain_problem(tdomain, tamp_problem)
            action_instances = t_get_action_instances(task, plan)

            tstates = [TState(map_to_atoms(evaluations.all_facts), eval_value_map)]

            tactions = []
            for action in action_instances:
                new_state = apply_action(tstates[-1].atoms, action)
                evals, value_map = stringify_atoms(new_state)
                tstates.append(TState(map_to_atoms(evals), value_map))
                tactions.append(TAction(action))

        elif(isinstance(plan[0], MCTSNode)):
            tstates = []
            tactions = []
            for node in plan[:-1]:
                evals, value_map = stringify_atoms(list(set(list(node.atoms)+list(node.axiom_atoms))))
                tstates.append(TState(map_to_atoms(evals), value_map))
                tactions.append(node.action_name)

            evals, value_map = stringify_atoms(list(set(list(plan[-1].atoms)+list(plan[-1].axiom_atoms))))
            tstates.append(TState(map_to_atoms(evals), value_map))

        # Take each action
        tplan = TPlan(tstates, tactions, {}, spec)
        database.add_plan(train_problem, tplan)

    # Save the plans in a plan database
    database.save()

    print("Success :)")
