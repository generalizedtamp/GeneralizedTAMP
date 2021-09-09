import random

from GeneralizedTAMP.genqnp.task_planning_problems.generators.problem_generator import Generator


class Carry(Generator):

    def __init__(self, num_obj=5, num_carry=5):
        self.num_obj = num_obj
        self.num_carry = num_carry
        self.problem_name = "carry_" + str(self.num_carry) + "_" + str(self.num_obj)
        self.domain_name = "carry"
        self.problem_file_path = "./genqnp/task_planning_problems/tasks/" + str(self.domain_name) + "/" + str(
            self.problem_name) + ".pddl"

    def get_objects(self):
        objects = []
        for obji in range(self.num_obj):
            objects.append("obj%s" % obji)
        for sloti in range(self.num_carry):
            objects.append("slot%s" % sloti)
        objects.append("src")
        objects.append("dest")
        objects.append("c")
        return objects

    def get_init(self):
        init = []
        for obji in range(self.num_obj):
            init.append(["on", "obj%s" % obji, "src"])
            init.append(["block", "obj%s" % obji])

        for sloti in range(self.num_carry):
            init.append(["slot", "slot%s" % sloti])

        init.append(["src_region", "src"])
        init.append(["dest_region", "dest"])
        init.append(["container", "c"])
        init.append(["region", "c"])
        init.append(["region", "dest"])
        init.append(["region", "src"])
        init.append(["canmove", "dest", "src"])
        init.append(["canmove", "src", "dest"])
        init.append(["at", "src"])
        init.append(["at", "c"])
        init.append(["hfree"])
        init.append(["on", "c", "src"])
        return init

    def get_goal(self):
        goal = []
        for obji in range(self.num_obj):
            goal.append(["on", "obj%s" % obji, "dest"])
        return goal
