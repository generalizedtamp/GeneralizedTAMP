import os
import random
import sys

import imageio
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import scipy.misc
from GeneralizedTAMP.genqnp.language.planner import find_plan, apply_action, task_from_domain_problem, \
    get_action_instances, parse_sequential_domain, parse_problem
from GeneralizedTAMP.genqnp.language.planner import find_plan, plan, parse_solution, task_from_domain_problem
from GeneralizedTAMP.genqnp.task_planning_problems.generators.problem_generator import Generator
from PIL import Image
from matplotlib import colors

matplotlib.use('TkAgg')


class Navigation(Generator):

    def __init__(self, grid_size=4, seed=0, wall_res=0, walled=False):
        self.grid_size = grid_size
        self.seed = seed
        random.seed(self.seed)
        self.wall_res = wall_res
        self.num_rewards = grid_size ** 2 // 15
        self.num_boundaries = grid_size ** 2 // 15
        self.walled = walled
        self.room_size = grid_size // (wall_res + 1)
        assert self.num_rewards + self.num_boundaries < self.grid_size ** 2
        if (not walled):
            self.problem_name = "Navigation_" + str(self.seed) + "_" + str(self.grid_size)  
            self.domain_name = "Navigation"
        else:
            self.problem_name = "Walled_Navigation_" + str(self.wall_res) + "_" + str(self.seed) + "_" + str(self.grid_size) 
            self.domain_name = "WalledNavigation"

        self.problem_file_path = "./task_planning_problems/tasks/" + str(self.domain_name) + "/" + str(
            self.problem_name) + ".pddl"

    def get_objects(self):
        objects = []
        for pi in range(self.grid_size):
            for pj in range(self.grid_size):
                objects.append("pos-%s-%s" % (pi, pj))
        return objects

    def is_walled(self, pos1, pos2):
        if (self.wall_res == 0):
            return False
        elif (pos1[0] == pos2[0]):
            if (pos1[1] > pos2[1] and ((pos1[1]) % self.room_size == 0)):
                return True
            if (pos1[1] < pos2[1] and ((pos2[1]) % self.room_size == 0)):
                return True

        elif (pos1[1] == pos2[1]):
            if (pos1[0] > pos2[0] and ((pos1[0]) % self.room_size == 0)):
                return True
            if (pos1[0] < pos2[0] and ((pos2[0]) % self.room_size == 0)):
                return True

    def is_door(self, pos1, pos2):
        if (self.wall_res == 0):
            return False
        elif (pos1[0] == pos2[0] and (pos1[0] % self.room_size) == self.room_size // 2):
            if (pos1[1] > pos2[1] and ((pos1[1]) % self.room_size == 0)):
                return True
            if (pos1[1] < pos2[1] and ((pos2[1]) % self.room_size == 0)):
                if (pos1[0] % self.room_size == self.room_size // 2):
                    return True

        elif (pos1[1] == pos2[1] and (pos1[1] % self.room_size) == self.room_size // 2):
            if (pos1[0] > pos2[0] and ((pos1[0]) % self.room_size == 0)):
                return True
            if (pos1[0] < pos2[0] and ((pos2[0]) % self.room_size == 0)):
                return True

    def get_init(self):
        init = []
        total_set = []
        for pi in range(self.grid_size):
            for pj in range(self.grid_size):

                if (pi - 1 >= 0):
                    if (not self.is_walled((pi, pj), (pi - 1, pj)) or self.is_door((pi, pj), (pi - 1, pj))):
                        init.append(['adjacent', "pos-%s-%s" % (pi, pj), "pos-%s-%s" % (pi - 1, pj)])
                if (pj - 1 >= 0):
                    if (not self.is_walled((pi, pj), (pi, pj - 1)) or self.is_door((pi, pj), (pi, pj - 1))):
                        init.append(['adjacent', "pos-%s-%s" % (pi, pj), "pos-%s-%s" % (pi, pj - 1)])
                if (pi + 1 < self.grid_size):
                    if (not self.is_walled((pi, pj), (pi + 1, pj)) or self.is_door((pi, pj), (pi + 1, pj))):
                        init.append(['adjacent', "pos-%s-%s" % (pi, pj), "pos-%s-%s" % (pi + 1, pj)])
                if (pj + 1 < self.grid_size):
                    if (not self.is_walled((pi, pj), (pi, pj + 1)) or self.is_door((pi, pj), (pi, pj + 1))):
                        init.append(['adjacent', "pos-%s-%s" % (pi, pj), "pos-%s-%s" % (pi, pj + 1)])
                total_set.append("pos-%s-%s" % (pi, pj))

        # Use the seed to create a distribution of boundaries and rewards
        random.shuffle(total_set)
        boundaries = total_set[:self.num_rewards]
        rewards = total_set[self.num_rewards:self.num_rewards + self.num_boundaries]

        for boundary in boundaries:
            init.append(['boundary', boundary])

        for reward in rewards:
            init.append(['reward', reward])

        init.append(["at", total_set[self.num_rewards + self.num_boundaries]]);

        return init

    @staticmethod
    def get_grid_state(grid_size, state):
        grid = np.zeros((grid_size + 2, grid_size + 2))
        print(grid.shape)
        for atom in state:
            if (atom.predicate == "reward"):
                i, j = int(str(atom.args[0]).split("-")[1]), int(str(atom.args[0]).split("-")[2])
                grid[i + 1][j + 1] = 1.0
            if (atom.predicate == "boundary"):
                i, j = int(str(atom.args[0]).split("-")[1]), int(str(atom.args[0]).split("-")[2])
                grid[i + 1][j + 1] = 3.0
            if (atom.predicate == "at"):
                i, j = int(str(atom.args[0]).split("-")[1]), int(str(atom.args[0]).split("-")[2])
                grid[i + 1][j + 1] = 2.0

        return grid

    def get_goal(self):
        goal = []
        for pi in range(self.grid_size):
            for pj in range(self.grid_size):
                goal.append(['not', 'reward', "pos-%s-%s" % (pi, pj)])
        return goal


def vis_grid(grid, gridsize):
    # create discrete colormap
    cmap = colors.ListedColormap(['white', 'green', 'blue', 'red', 'black'])
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5]
    norm = colors.BoundaryNorm(bounds, cmap.N)
    fig, ax = plt.subplots()
    ax.imshow(grid, cmap=cmap, norm=norm)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(np.arange(-0.5, gridsize + 2, 1));
    ax.set_yticks(np.arange(-0.5, gridsize + 2, 1));

    fig.canvas.draw()
    image_from_plot = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    image_from_plot = image_from_plot.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    print(image_from_plot)
    im = Image.fromarray(image_from_plot)

    return im


if __name__ == "__main__":

    # Locate the plan
    task_root = "./task_planning_problems/tasks/"
    sol_root = "./task_planning_problems/solutions/"
    task_name = "navigationX_nY"
    grid_size = 20
    problem_file = os.path.join(task_root, task_name, "navigation" + str(grid_size) + "_n0.pddl")
    plan_file = os.path.join(sol_root, task_name, "navigation" + str(grid_size) + "_n0.sol")
    domain_file = os.path.join(task_root, task_name, task_name + "_domain.pddl")
    plan = parse_solution(plan_file)
    print(plan)

    # Get the states
    transitions = []
    task_domain = parse_sequential_domain(read(domain_file))
    task_problem = parse_problem(task_domain, read(problem_file))

    # Get the initial state of the problem and step through
    task = task_from_domain_problem(task_domain, task_problem)
    action_instances = get_action_instances(task, plan)

    print(task.init)
    concrete_states = [task.init]
    grid = NavigationX_nY.get_grid_state(grid_size, task.init)
    im = vis_grid(grid, grid_size)
    filename_gen = lambda a, ext: "./task_planning_problems/solutions/navigationX_nY/navigation" + str(
        grid_size) + "_n0/a" + str(a) + "." + ext
    filenames = [filename_gen(0, "png")]
    im.save(filename_gen(0, "png"))

    for action_idx, action in enumerate(action_instances):
        state = apply_action(concrete_states[-1], action)
        concrete_states.append(state)
        grid = NavigationX_nY.get_grid_state(grid_size, state)
        im = vis_grid(grid, grid_size)
        im.save(filename_gen(action_idx, "png"))
        filenames.append(filename_gen(action_idx, "png"))

    print(filenames)

    # Turn the images into a gif
    images = []
    for filename in filenames:
        images.append(imageio.imread(filename))
    imageio.mimsave(filename_gen(-1, "gif"), images)
