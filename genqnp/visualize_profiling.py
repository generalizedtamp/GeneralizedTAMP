from collections import namedtuple, defaultdict
import numpy as np
import argparse
from utils import get_arg_string, get_experiment_dict, read_from_minio
import pickle
import os
import time
import math
import matplotlib.pyplot as plt

from language.planner import apply_action,\
    task_from_domain_problem, \
    t_get_action_instances, get_problem, parse_sequential_domain
from pddlstream.utils import read
from tasks.carry.problem import Carry
from tasks.sort.problem import Sort
from tasks.clutter.problem import Clutter
import networkx as nx


# run_name = "realrun4" # Stack3d
# run_name = "realrun5" # Sort
# run_name = "realrun10" # Clutter
# run_name = "realrun102" # Carry 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name',  default='stack3d_0', help='')
    parser.add_argument('-re', '--read-external',  action="store_true", help='')
    parser.add_argument('-v', '--visualize', action="store_false", help='')
    parser.add_argument('-vg', '--visualize-graphs', action="store_true", help='')
    parser.add_argument('-ns', '--num-seeds', default=6, help='')
    parser.add_argument('-r', '--run-name', default="run0", help='')
    exp_name_title  = {
        "stack3d_0": "Unstack",
        "carry_0":"Transport",
        "sort_0":"Sort",
        "clutter_0": "Clutter"
    }

    exp_name_x_idx  = {
        "stack3d_0": 1,
        "carry_0":2,
        "sort_0":1,
        "clutter_0":1
    }

    args = parser.parse_args()
    exp_dict = get_experiment_dict(args.exp_name)

    result_datas = defaultdict(lambda: defaultdict(list))
    if(args.read_external):
        for mode in ["gentamp", "standard", "skeleton"]:
            for testing_name in exp_dict['testing_problems']:
                for seed in range(args.num_seeds):
                    search_name = "{}_{}_{}_{}.pkl".format(args.run_name, mode, testing_name, seed)
                    print(search_name)
                    try:
                        if(read_from_minio(search_name)[testing_name][0]<60*2):
                            result_datas[mode][testing_name].append(read_from_minio(search_name)[testing_name])
                        else:
                            result_datas[mode][testing_name].append([60*2])
                    except:
                        print("File not found")
    else:
        # TODO: Fix this for local visualization
        results_fn = os.path.join(exp_dict['gen_tamp_results_folder'], "profiling_results.pkl")
        with open(results_fn, 'rb') as h:
            result_datas = pickle.load(h)

    # print(result_datas)
    # import sys
    # sys.exit(1)

    for mode in list(result_datas.keys()):
        print(mode)
        trace_data = result_datas[mode]
        print(trace_data)
        problems = list(trace_data.keys()) 
        list_num = [float(t.split("_")[exp_name_x_idx[args.exp_name]]) for t in problems]
        means = [np.mean([td[0] for td in trace_data[p]]) for p in problems]
        stds = [2*np.std([td[0] for td in trace_data[p]])/math.sqrt(args.num_seeds) for p in problems]
        plt.plot(list_num, means, marker='o',)
        plt.fill_between(list_num, np.array(means)-np.array(stds), np.array(means)+np.array(stds), alpha=0.2)

    plt.ylabel("Time (s)")
    # plt.ylim([0, 125])
    # plt.legend(["GenTAMP", "Unguided TAMP", "Oracle Skeleton"])
    plt.xlabel("Num Blocks")
    plt.title(exp_name_title[args.exp_name])
    plt.show()