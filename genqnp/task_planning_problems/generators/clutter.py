from GeneralizedTAMP.genqnp.task_planning_problems.generators.problem_generator import Generator

class Clutter(Generator):

    def __init__(self, num_blocks = 5):
        self.num_blue = num_blocks
        self.problem_name = "Clutter" + "_" + str(self.num_blue)
        self.domain_name = "clutter"
        self.problem_file_path = "./genqnp/task_planning_problems/tasks/" \
            + str(self.domain_name) + "/" + str(self.problem_name) + ".pddl"

    def get_objects(self):
        objects = []
        objects.append("or1")
        for b_block in range(self.num_blue):
            objects.append("ob%s" % b_block)

        objects.append("rred")
        objects.append("rblue")
        objects.append("rtable")
        return objects

    def get_init(self):
        init = []

        init.append(["on", "or1", "rtable"])
        init.append(["block", "or1"])
        init.append(["red", "or1"])

        for b_block in range(self.num_blue):
            init.append(["on", "ob%s" % b_block, "rtable"])
            init.append(["block", "ob%s" % b_block])
            init.append(["blue", "ob%s" % b_block])

        init.append(["table", "rtable"])
        init.append(["region", "rtable"])
        init.append(["hfree"])

        return init

    def get_goal(self):
        goal = [["holding", "or1"]]
        return goal
