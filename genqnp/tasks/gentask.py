# Abstract class for generalized planning tasks
from GeneralizedTAMP.genqnp.language.planner import \
    parse_sequential_domain, parse_problem
from pddlstream.utils import read
import pybullet as p
import cProfile
from pddlstream.utils import read, INF, get_file_path
from search import solve_mcts
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import LockRenderer, wait_if_gui
from GeneralizedTAMP.pybullet_planning.pybullet_tools.pr2_primitives import \
    Pose, Conf, \
    get_stable_gen, get_grasp_gen, Attach, Detach, \
    get_gripper_joints, GripperCommand, apply_commands, State


class ProblemSpec():
    def __init__(self):
        self.costs = {}
        self.skeleton = None
        self.cost = INF
        self.nonfluents = []

class GenTask():
    def __init__(self, 
                 task_domain, 
                 task_problem_file, 
                 domain, 
                 typed_domain,
                 additional_init=[], 
                 viewer=False, 
                 teleport=False, 
                 partial=False,
                 max_complexity=1,
                 policy_heuristic=False,
                 use_skeleton=True,
                 seed=0):

        #### These are mostly useless at this point ####
        self.task_domain = task_domain
        self.task_problem_file = task_problem_file
        self.task_problem = parse_problem(self.task_domain, read(task_problem_file))

        # Domains
        self.domain = domain
        self.typed_domain = typed_domain
        self.parsed_domain = parse_sequential_domain(read(self.domain))
        self.parsed_typed_domain = parse_sequential_domain(read(self.typed_domain))

        # Additional params
        self.additional_init = additional_init
        self.viewer = viewer
        self.teleport = teleport
        self.partial = partial
        self.seed = sum(list(task_problem_file.encode('utf8')))+seed
        self.policy_heuristic = policy_heuristic

        self.action_costs = {}
        self.max_complexity = max_complexity
        self.use_skeleton = use_skeleton


    def add_init_types(self, init):
        # Each object is tied to at least one predicate and each predicate has typed arguments
        typed_object_tuples = []
        objects_stored = []
        for atom_tuple in init:
            for predicate in self.parsed_typed_domain.predicates:
                if(predicate.name == atom_tuple[0]):
                    assert (len(atom_tuple[1:]) == len(predicate.arguments)), "A predicate has incorrect length"
                    for object_name, predicate_arg in zip(atom_tuple[1:], predicate.arguments):
                        if(str(object_name) not in objects_stored):
                            typed_object_tuples.append((predicate_arg.type_name, object_name))
                            objects_stored.append(str(object_name))

        return init+list(typed_object_tuples)

    def terminate(self):
        pass

    def plan_problem(self, verbose=True):


        # Get the problem specification from the task file
        spec = self.get_problem_spec()

        # Get the pddlstream formatted problem
        pddlstream_problem = self.pddlstream_from_spec(spec)

        pddlstream_domain, constant_map, external_pddl, stream_map, init, goal = pddlstream_problem

        for ap in self.additional_init:
            init.append(tuple([ap.predicate]+list(ap.args)))

        pddlstream_problem = pddlstream_domain, constant_map, external_pddl, stream_map, init, goal

        print('Init:', init)
        print('Goal:', goal)
        print('Streams:', stream_map.keys())

        pr = cProfile.Profile()
        pr.enable()

        print("Success cost: "+str(spec.cost))
        heuristic = "policy" if self.policy_heuristic else "tamp"
        solution, stats = solve_mcts(pddlstream_problem,
                                     success_cost = spec.cost,
                                     parsed_domain = self.parsed_typed_domain,
                                     max_complexity = self.max_complexity,
                                     action_costs = self.action_costs,
                                     skeleton = spec.skeleton if self.use_skeleton else None,
                                     nonfluents = spec.nonfluents,
                                     default_action_cost=0,
                                     heuristic = heuristic)

        plan, cost, evaluations, total_time = solution
        print("Cost: "+str(cost))
        self.terminate()
        return plan, cost, evaluations, spec, total_time

    def get_problem_spec(self):
        raise NotImplementedError

    def get_goal(self):
        spec = self.get_problem_spec()
        return self.pddlstream_from_spec(spec).goal

    def get_domain(self):
        return self.domain

    def visualize_plan(self, plan):
        print("Get problem spec")
        spec = self.get_problem_spec(vis=True)
        wait_if_gui()

        pddlstream_problem = self.pddlstream_from_spec(spec)
        self.commands, evaluation_map = \
            self.post_process(spec, pddlstream_problem.init, plan)
       
        apply_commands(State(), self.commands, time_step=0.05)
        self.terminate()
