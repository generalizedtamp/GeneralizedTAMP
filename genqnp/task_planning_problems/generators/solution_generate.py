import os

from GeneralizedTAMP.genqnp.language.planner import find_plan, plan, parse_sequential_domain, parse_problem

TASK_BASE = "./genqnp/task_planning_problems/tasks/"
SOLUTION_BASE = "./genqnp/task_planning_problems/solutions/"
import shutil


def get_problem_filename(task, problem):
    return TASK_BASE + str(task) + "/" + problem + ".pddl"


def get_domain_filename(task):
    return TASK_BASE + str(task) + "/" + str(task) + "_domain.pddl"


def write_plan(task, problem, plan_file):
    dest_filename = os.path.join(SOLUTION_BASE, str(task) + "/" + str(problem) + ".sol")
    # TODO: Copy plan file to new filename
    shutil.copy(plan_file, dest_filename)


if __name__ == '__main__':
    # task = "carryY_cZ"
    # problem = "Carry8_c3"

    # task = "sortYb_Zr"
    # problem = "Sort3b_2r"

    # task = "navigationX_nY"
    # problem = "navigation20_n0"

    task = "stack"
    problem = "stack_b6"

    plan_filename = plan(get_domain_filename(task), get_problem_filename(task, problem))
    print(plan_filename)
    write_plan(task, problem, plan_filename)
