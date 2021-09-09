import random

from GeneralizedTAMP.genqnp.task_planning_problems.generators.problem_generator import Generator


class Sort(Generator):

    def __init__(self, num_red=5, num_blue=5):
        self.num_red = num_red
        self.num_blue = num_blue

        self.problem_name = "sort_{}_{}_{}".format(str(self.num_red+self.num_blue) , str(self.num_red), str(self.num_blue))
        self.domain_name = "sort"
        self.problem_file_path = "./genqnp/task_planning_problems/tasks/" + str(self.domain_name) + "/" + str(
            self.problem_name) + ".pddl"

    def get_objects(self):
        objects = []
        for r_block in range(self.num_red):
            objects.append("or%s" % r_block)
        for b_block in range(self.num_blue):
            objects.append("ob%s" % b_block)
        objects.append("rred")
        objects.append("rblue")
        objects.append("rtable")
        return objects

    def get_init(self):
        init = []

        for r_block in range(self.num_red):
            init.append(["on", "or%s" % r_block, "rtable"])
            init.append(["block", "or%s" % r_block])
            init.append(["red", "or%s" % r_block])

        for b_block in range(self.num_blue):
            init.append(["on", "ob%s" % b_block, "rtable"])
            init.append(["block", "ob%s" % b_block])
            init.append(["blue", "ob%s" % b_block])

        init.append(["table", "rtable"])
        init.append(["redregion", "rred"])
        init.append(["blueregion", "rblue"])
        init.append(["region", "rtable"])
        init.append(["region", "rred"])
        init.append(["region", "rblue"])
        init.append(["hfree"])

        return init

    def get_goal(self):
        goal = []

        for b_block in range(self.num_red):
            goal.append(["on", "or%s" % b_block, "rred"])

        for r_block in range(self.num_blue):
            goal.append(["on", "ob%s" % r_block, "rblue"])
            
        return goal
