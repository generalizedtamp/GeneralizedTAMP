from task_planning_problems.generators.carry import Carry
from task_planning_problems.generators.clutter import Clutter
from task_planning_problems.generators.navigation import Navigation
from task_planning_problems.generators.sort import Sort
from task_planning_problems.generators.stack import Stack

if __name__ == '__main__':
    task = "sort"
    if (task == "sort"):
        for num_red in range(0, 10):
            for num_blue in range(0, 10):
                sort_generator = Sort(num_red=num_red, num_blue=num_blue)
                sort_generator.generate()

    elif (task == "carry"):
        for obj in range(1, 20):
            for slots in range(1, obj + 1):
                carry = Carry(num_obj=obj, num_carry=slots)
                carry.generate()
    elif (task == "navigation"):
        for x in range(4, 100):
            for seed in range(10):
                nav = Navigation(grid_size=x, seed=seed)
                nav.generate()
    elif (task == "walled_navigation"):
        for x in range(5, 50, 5):
            for g in range(0, 10):
                if (g == 0 or (x % (g + 1) == 0 and x // (g + 1) >= 5)):
                    for seed in range(10):
                        nav = Navigation(grid_size=x, seed=seed, wall_res=g, walled=True)
                        nav.generate()
    elif (task == "stack"):
        for num_blocks in range(3, 30, 1):
            nav = Stack(num_blocks=num_blocks)
            nav.generate()
    elif (task == "clutter"):
        for num_blocks in range(1, 30, 1):
            nav = Clutter(num_blocks=num_blocks)
            nav.generate()
