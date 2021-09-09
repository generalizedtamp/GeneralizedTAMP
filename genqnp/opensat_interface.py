import os
import time
import subprocess
from utils import print_concept

def solve_sat(clause_lines,
              features=None,
              complexity_map=None,
              mode=None,
              num_vars=0,
              max_weight=0):


    refined_complexity_map = {print_concept(ck):cv for ck, cv in complexity_map.items()}

    TMPDIR = "./temp"
    OPENWBO_TMPDIR = "/GeneralizedTAMP/"
    OPENWBO_FOLDER = ""
    cnf_filename = os.path.join(TMPDIR, str(time.time()) + ".cnf")
    with open(cnf_filename, 'a') as cnf_file:
        cnf_file.write('%s %s %s %s\n' % ("p", mode, str(num_vars), str(len(clause_lines))))
        for clause_line in clause_lines:
            if (mode == "wcnf"):
                clause_line = [max_weight] + clause_line
            cnf_file.write(" ".join([str(p) for p in clause_line]) + "\n")

        # Now write all of the soft constraints
        if (mode == "wcnf"):
            for fi, f in enumerate(features):
                cnf_file.write(" ".join([str(refined_complexity_map[print_concept(f.concept)]), str(-(fi+1)), str(0), "\n"]))

    # Pass to the SAT solver
    OPENWBO_PATH = os.path.join(OPENWBO_FOLDER, "open-wbo")
    RESULTS_PATH = os.path.join(TMPDIR, "results.txt")
    f = open(RESULTS_PATH, "w")
    sat_command = OPENWBO_PATH+" "+OPENWBO_TMPDIR+cnf_filename+" > "+RESULTS_PATH
    print(['/bin/zsh', '-i', '-c', sat_command])
    subprocess.call(['/bin/zsh', '-i', '-c', sat_command])
    f.close()

    # Extract the solution
    results_file = open(RESULTS_PATH, "rb")
    lines = results_file.readlines()
    # Strips the newline character
    for line in lines:
        line = line.decode()
        if (line[0] == 'v'):
            result_string = [int(i) for i in line.split(" ")[1:-1]]

    return result_string
