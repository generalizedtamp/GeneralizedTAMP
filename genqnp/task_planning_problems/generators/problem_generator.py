class Generator:
    """
        Superclass for all task generators
    """

    def __init__(self):
        pass

    def generate_header(self, problem_name, domain_name):
        return "(define (problem %s)\n" % problem_name \
               + "(:domain %s)" % domain_name \
 
    def generate_footer(self):
        return ")"

    def generate_objects(self, objects):
        return "(:objects " \
               + " ".join(objects) \
               + ")"

    def generate_literal(self, literal):
        if (literal[0] == "not"):
            return "(not(" + " ".join(literal[1:]) + "))"
        else:
            return "(" + " ".join(literal) + ")"

    def generate_init(self, init):
        return "(:init " \
               + "\n".join([self.generate_literal(p) for p in init]) \
               + ")"

    def generate_goal(self, goal):
        return "(:goal (and " \
               + " ".join([self.generate_literal(p) for p in goal]) \
               + "))"

    def generate_int(self, problem_name, domain_name, objects, init, goal):
        return self.generate_header(problem_name, domain_name) + "\n" \
               + self.generate_objects(objects) + "\n" \
               + self.generate_init(init) + "\n" \
               + self.generate_goal(goal) + "\n" \
               + self.generate_footer() \
 
    def generate(self):
        problem_string = self.generate_int(self.problem_name, self.domain_name, self.get_objects(), self.get_init(),
                                           self.get_goal())
        with open(self.problem_file_path, "w") as problem_file:
            problem_file.write(problem_string)
