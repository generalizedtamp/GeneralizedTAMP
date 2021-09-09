import random

from GeneralizedTAMP.genqnp.task_planning_problems.generators.problem_generator import Generator


class Stack(Generator):

    def __init__(self, num_blocks=5):
        self.num_blocks = num_blocks
        self.problem_name = "Stack_" + str(self.num_blocks)
        self.domain_name = "Stack"
        self.problem_file_path = "./genqnp/task_planning_problems/tasks/" + str(self.domain_name) + "/" + str(
            self.problem_name) + ".pddl"

    def get_objects(self):
        objects = []
        objects.append("table")
        for obji in range(self.num_blocks):
            objects.append("obj%s" % obji)
        return objects

    def get_init(self):
        init = []
        total_regions = ["table"]
        block_poses = {}
        on_tuples = []
        for obji in range(self.num_blocks):
            random.shuffle(total_regions)
            obj_sym = "obj%s" % obji
            place_region = total_regions.pop(0)
            if (place_region == "table"):
                total_regions.append("table")
                init.append(["on-table", obj_sym])
            else:
                init.append(["on", obj_sym, place_region])
            total_regions.append(obj_sym)

        for place_region in total_regions:
            if (place_region != "table"):
                init.append(["clear", place_region])

        return init

    def get_goal(self):
        goal = []
        for obji in range(self.num_blocks):
            goal.append(["on-table", "obj%s" % obji])
        return goal
