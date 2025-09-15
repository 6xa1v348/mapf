import os
import sys
sys.path.append(os.path.join(os.getcwd(), "build"))

import numpy as np
import mapf
import time


def main():
    params = mapf.Parameters()
    params.map = "assets/warehouse"
    params.solver = "PIBT"
    params.verbose = True
    params.with_weights = True
    print("==== Parameters ====")
    for name in dir(params):
        if not name.startswith("__"):
            value = getattr(params, name)
            print(f"{name:<20} {value}")
    print("--------------------")

    t_start = time.time()
    ## make graph
    graph = mapf.make_graph(params)
    weights = graph.weights()
    print(f"Shape of weights : {weights.shape}")

    ## make instance
    instance = mapf.make_instance(graph, params)
    print(f"Number of agents (initial) : {instance.num_agents}")
    # instance.make(200)
    start = [(9, 17, 3), (25, 17, 1), (17, 9, 0)]
    goal = [(17, 18, 0), (17, 18, 0), (17, 18, 0)]
    instance.make(start, goal, 3)
    print(instance.num_agents)
    print(f"Start states : {instance.get_starts()}")
    print(f"Goal states : {instance.get_goals()}")
    print(f"Number of agents : {instance.num_agents}")

    ## make solver
    solver = mapf.make_solver(instance, params)
    print(f"Name of solver : {solver.name}")

    ## run solver
    solver.solve()
    print(f"Succed? : {solver.succeed()}")
    plan = solver.get_solution()
    print(f"Number of paths : {plan.size()}")
    print(f"Makespan : {plan.get_makespan()}")
    print(f"No Conflicts? : {plan.validate(instance)}")
    plan.save("app.plan")
    elapsed = (time.time() - t_start) * 1000
    print(f"Elapsed Time (ms) : {elapsed:.4f}")


if __name__ == "__main__":
    main()