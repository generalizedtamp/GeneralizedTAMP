"""
    Script that takes a QNP specification
    a generalized planning problem and, converts it to
    a FOND problem, solves the FOND problem, parses the results,
    traces the policy using BFS, and visualizes the resulting plan
"""

import os
import shutil
import subprocess
import argparse
import utils
from policy_utils.validator import validate


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--exp-name', default='stack_0', help='')
    args = parser.parse_args()
    experiments_dict = utils.get_experiment_dict(args.exp_name)

    # Task specification files
    TMPDIR = "./temp"

    VOLUME_PATH = "/GeneralizedTAMP/"
    QNP_FILE = os.path.join(VOLUME_PATH, experiments_dict['gen_tamp_results_folder'],
                            "abstraction.qnp")
    if (os.path.exists(TMPDIR) and os.path.isdir(TMPDIR)):
        shutil.rmtree(TMPDIR)
    os.mkdir(TMPDIR)

    QNP2FOND_PREFIX = os.path.join(VOLUME_PATH, experiments_dict['gen_tamp_results_folder']+"/tmp_fond")
    QNP2FOND_BINARY = "qnp2fond"
    QNP2FOND_FLAGS = ['--force-direct']

    PRPFOND_FOLDER = "./FOND_PRP"
    PRPFOND_BINARY = "prp"
    PRPFOND_VAL = os.path.join(PRPFOND_FOLDER, 'prp_scripts/validator.py')
    PRPFOND_TRANSLATE = \
        os.path.join(PRPFOND_FOLDER, 'prp_scripts/translate_policy.py')
    PRPFOND_INPUT_FILES = \
        [experiments_dict['gen_tamp_results_folder']+"/tmp_fond" + str(f) for f in ["_d.pddl", "_p.pddl"]]
    PRPFOND_INIT_POLICY = "policy.out"
    PRPFOND_INIT_OUTPUT = "output"

    PRPFOND_TRANSLATE_OUTPUT = "translated_policy.out"
    PRPFOND_FLAGS = ["--dump-policy", "2",
                     "--detect-deadends", "0",
                     "--generalize-deadends", "0",
                     "--online-deadends", "0"]

    # Call QNP2FOND with temp prefix

    print("============ QNP TO FOND ============")
    qnp2fond_command = " ".join([QNP2FOND_BINARY] + QNP2FOND_FLAGS +
                                [QNP_FILE, QNP2FOND_PREFIX])

    subprocess.call(['/bin/zsh', '-i', '-c', qnp2fond_command])

    print("============ FOND PLANNING ============")
    # Call FOND with the results
    fond_command = [PRPFOND_BINARY] + \
                    [VOLUME_PATH+f for f in PRPFOND_INPUT_FILES] + \
                    PRPFOND_FLAGS
    print(fond_command)
    prp_command = " ".join(fond_command)


    subprocess.call(['/bin/zsh', '-i', '-c', prp_command])

    print("============ POLICY TRANSLATE ============")
    # Translate the policy
    f = open(os.path.join(experiments_dict['gen_tamp_results_folder'],
                          PRPFOND_TRANSLATE_OUTPUT), "w")

    subprocess.run(["python", PRPFOND_TRANSLATE], stdout=f)


    print("============ POLICY VISUALIZATION ============")
    # Visualize graph
    vis_command = validate(PRPFOND_INPUT_FILES[0],
                           PRPFOND_INPUT_FILES[1],
                           os.path.join(experiments_dict['gen_tamp_results_folder'], PRPFOND_TRANSLATE_OUTPUT),
                           experiments_dict['gen_tamp_results_folder'])

