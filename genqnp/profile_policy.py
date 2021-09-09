from collections import namedtuple
import numpy as np
import argparse
from utils import get_arg_string, get_experiment_dict
import pickle
import os
import time
from language.planner import apply_action,\
    task_from_domain_problem, \
    t_get_action_instances, get_problem, parse_sequential_domain
from pddlstream.utils import read
from tasks.carry.problem import Carry
from tasks.stack3d.problem import Stack3d
from tasks.sort.problem import Sort
from tasks.clutter.problem import Clutter
from pddl import Axiom, UniversalCondition, Action, Predicate, \
                Atom, NegatedAtom, Effect, TypedObject
from domain_utils import save_domain
from utils import write_to_minio, get_additional_init

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name',  default='stack_0', help='')
    parser.add_argument('-v', '--visualize', action="store_false", help='')
    parser.add_argument('-m', '--mode', default='', help='')
    parser.add_argument('-r', '--run-name', default='run0', help='')
    parser.add_argument('-we', '--write-external', action="store_true", help='')
    parser.add_argument('-t', '--test-problem', default='', help='')
    parser.add_argument('-s', '--seed', default=0, help='')

    args = parser.parse_args()
    print("Exp args")
    print(args)
    exp_dict = get_experiment_dict(args.exp_name)
    
    visaualize_plan = args.visualize
    task_domain_file = exp_dict['domain_file']
    parsed_task_domain = parse_sequential_domain(read(task_domain_file))

    # Two profiled domains
    typed_domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "typed_domain.pddl")
    untyped_domain_fn = os.path.join(exp_dict['gen_tamp_folder'], "domain.pddl")

    parsed_untyped_domain = parse_sequential_domain(read(typed_domain_fn))
    save_domain(parsed_untyped_domain, untyped_domain_fn)

    typed_embedded_domain_fn = os.path.join(exp_dict['gen_tamp_results_folder'], "embedded_typed_domain.pddl")
    untyped_embedded_domain_fn = os.path.join(exp_dict['gen_tamp_results_folder'], "embedded_domain.pddl")

    parsed_untyped_embedded_domain = parse_sequential_domain(read(typed_embedded_domain_fn))
    save_domain(parsed_untyped_embedded_domain, untyped_embedded_domain_fn)

    use_skeleton = False
    policy_heuristic = False
    profiling_results = {}

    if(args.mode == "gentamp"):
        domains = [untyped_embedded_domain_fn, typed_embedded_domain_fn]
        policy_heuristic = True
    elif(args.mode == "standard"):
        domains = [untyped_domain_fn, typed_domain_fn]
    elif(args.mode == "skeleton"):
        domains = [untyped_domain_fn, typed_domain_fn]
        use_skeleton = True
    else:
        raise NotImplementedError

    plans = []
    additional_init = get_additional_init(max_step=50)
    task_problem_file = exp_dict['task_problem_folder'] + args.test_problem + ".pddl"
    profiling_results[args.test_problem] = []
    (untyped_domain_file, typed_domain_file) = domains
    # Instantiate the problem class from the name
    problem = (globals()[exp_dict['task_name']])(parsed_task_domain, 
                                                exp_dict['task_problem_folder']+ args.test_problem + ".pddl",
                                                 untyped_domain_file,
                                                 typed_domain_file,
                                                 additional_init,
                                                 max_complexity=exp_dict['nf_complexity'],
                                                 policy_heuristic=policy_heuristic,
                                                 use_skeleton=use_skeleton,
                                                 seed = int(args.seed))

    plan, cost, evaluations, spec, total_time = problem.plan_problem()
    plans.append(plan)
    
    profiling_results[args.test_problem].append(total_time)

    # Save the results to the task folder
    print("===== Results =====")
    print(profiling_results)

    if(args.write_external):
        save_name = "{}_{}_{}_{}".format(args.run_name, args.mode, args.test_problem, args.seed)
        write_to_minio(profiling_results, save_name)
    else:
        results_fn = os.path.join(exp_dict['gen_tamp_results_folder'], "profresults_{}_{}.pkl".format(args.mode, args.test_problem))
        with open(results_fn, 'wb') as handle:
            pickle.dump(profiling_results, handle)


