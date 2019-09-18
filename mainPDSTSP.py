from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import pdb
from itertools import combinations
import ga
import time
import logging

logging.basicConfig(level=logging.INFO)

from utils import *


# Creating the route
def create_data_model(route_time_truck):
    data = dict()
    data['time_matrix'] = route_time_truck
    data['num_vehicles'] = 1
    return data


def print_solution(solution, solution_time):
    """Prints assignment on console."""
    plan_output = 'Route for vehicle 0:\n'
    print(' -> '.join(list(map(lambda x: str(x), solution))))
    print("Total time: {}".format(solution_time))


def getSolution(manager, routing, assignment):
    """Prints assignment on console."""
    index = routing.Start(0)
    route_distance = 0
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    route.append(manager.IndexToNode(index))
    return route, route_distance


def adjust_index(ctruck, solution):
    translate = dict()
    for x in ctruck:
        translate[ctruck.index(x)] = x
    result = list()
    for x in solution:
        result.append(translate[x])
    print(translate)
    return result


def get_new_index(original, points, index):
    points.sort()
    new_index = list(filter(lambda x: x[0] == index, [(x, i) for i, x in enumerate(points)]))[-1][1]
    cop = original.copy()

    complementary = [x for x in range(original.shape[0]) if x not in points]
    cop = np.delete(cop, complementary, axis=0)
    assert all(original[index] == cop[new_index])
    cop = np.delete(cop, complementary, axis=1)
    return new_index, cop


def solveTSP(ctruck, tmatrix, dist_center):
    ctruck = [int(x) for x in ctruck]
    new_index, truck_matrix = get_new_index(tmatrix, ctruck, dist_center)
    solution = None
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(truck_matrix), 1, new_index)

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return truck_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic PCA
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        solution, solution_time = getSolution(manager, routing, assignment)
        print_solution(solution, solution_time)
    return solution, solution_time



def get_tmatrix(filename, num_custom):
    return get_matrix(filename, num_custom)


def get_truckMatrix(dist_center, cdrone, tmatrix):
    print(tmatrix[dist_center])
    truck_matrix = tmatrix.copy()
    truck_matrix = np.delete(truck_matrix, cdrone, axis=0)
    truck_matrix = np.delete(truck_matrix, cdrone, axis=1)
    #### Actualizar depot con el nuevo valor del distribution center
    ### Se debe actualizar truck_matrix y generar nuevo data cada vez que se crea nueva truck_matrix
    data = create_data_model(truck_matrix)
    truck_matrix = data['time_matrix']
    return truck_matrix, data


def solvePMSGA(customers, n_drones, time_matrix, dist_center, names):
    # Number of jobs
    n = len(customers)
    # Number of machines
    m = n_drones
    # Genetic Algorithm : Population size
    GA_POPSIZE = 10
    # Genetic Algorithm : Elite rate
    GA_ELITRATE = 0.1
    # Genetic Algorithm : Mutation rate
    GA_MUTATIONRATE = 0.25
    # Genetic Algorithm : Iterations number
    GA_ITERATIONS = 10

    # TODO log this line
    try:
        customers = [int(x) for x in customers]
    except Exception as e:
        print("Exception {}".format(e))
        pass

    jobsProcessingTime = [(x[1], time_matrix[dist_center][x[1]] * 2) for x in enumerate(customers)]
    jobsProcessingTime.sort(key=lambda x: x[1])
    final_names = list(map(lambda x: x[0], jobsProcessingTime))
    jobsProcessingTime = list(map(lambda x: x[1], jobsProcessingTime))

    start = time.time()
    if customers:
        # Generate an initial random population
        population = [[] for i in range(GA_POPSIZE)]
        for i in range(GA_POPSIZE):
            population[i] = ga.random_chromosome(m, n)

        # Sort the population based on the fitness of chromosomes
        population = sorted(population, key=lambda c: ga.fitness(c, m, n, jobsProcessingTime))
        # Print initial best makespan
        print("Starting makespan = %03d" % (ga.fitness(population[0], m, n, jobsProcessingTime)))
        # Iterate
        for i in range(GA_ITERATIONS):
            # Sort the population : order by chromosone's scores.
            population = sorted(population, key=lambda c: ga.fitness(c, m, n, jobsProcessingTime))
            # Generate the following generation (new population)
            population = ga.evolve(population, GA_POPSIZE, GA_ELITRATE, GA_MUTATIONRATE, m, n)

        # Print the best fitness and the execution time after iterations
        dronemkspan = ga.fitness(population[0], m, n, jobsProcessingTime)
    else:
        dronemkspan = 0
        population = [list()]
    print("Ending makespan   = %03d" % (dronemkspan))
    print("Execution time = %02d s" % (time.time() - start))
    print("GA Solution")
    print(jobsProcessingTime)
    print(population[0])
    jobs_done = parse_drone_solution(population[0], jobsProcessingTime, final_names)
    return jobs_done, dronemkspan


def solvePMS(customers, n_drones, time_matrix, dist_center, names, random_search=False):
    drone_solution = list()
    drone_list = ["D#" + str(x) for x in range(n_drones)]
    drone_status = dict()

    try:
        customers = [int(x) for x in customers]
    except Exception as e:
        print("Exception {}".format(e))
        pass

    for x in drone_list:
        drone_status[x] = 'idle'

    jobs = [(x[0], time_matrix[dist_center][x[1]] * 2, x[1]) for x in enumerate(customers)]

    if random_search:
        import random;
        random.shuffle(jobs)
    else:
        jobs.sort(key=lambda x: x[1])

    jobs = list(map(lambda x: (x[1], x[2]), jobs))

    jobs_done = list()
    time_step = 0
    remain = True
    working_drones = []
    if not jobs:
        remain = False

    while remain:
        time_step += 1

        for x in drone_status:
            if drone_status[x] != 'busy':
                if jobs:
                    drone_status[x] = 'busy'
                    working_drones.append([(x, jobs[0][0]), 0.0])
                    jobs_done.append((x, jobs[0][1]))
                    jobs.pop(0)
        to_pop = list()
        for i, item in enumerate(working_drones):
            # Update job evolution
            item[1] += 1.0
            # if progress is equal to job duration
            if (item[1] >= item[0][1]):
                drone_status[item[0][0]] = 'idle'
                drone_solution.append((item[0], time_step))
                to_pop.append(i)
                if not jobs and all([x == 'idle' for x in drone_status.values()]):
                    remain = False
        working_drones = [x for i, x in enumerate(working_drones) if i not in to_pop]

    print(drone_solution)

    try:
        assert set(customers) == set([x for x in map(lambda x: x[1], jobs_done)])
    except Exception as e:
        print("Exception {}".format(e))
        pdb.set_trace()
    return jobs_done, time_step


def droneSolver(customers, n_drones, time_matrix, dist_center, names, solver_type='SJF'):
    jobs = list()
    total_time = -1
    if solver_type == 'SJF':
        jobs, total_time = solvePMS(customers, n_drones, time_matrix, dist_center, names, random_search=False)
    elif solver_type == 'random':
        jobs, total_time = solvePMS(customers, n_drones, time_matrix, dist_center, names, random_search=True)
    elif solver_type == 'genetic':
        jobs, total_time = solvePMSGA(customers, n_drones, time_matrix, dist_center, names)
    return jobs, int(round(total_time, 0))


def swap(cdrone, ctruck, elements):
    copy_drone = cdrone.copy()
    copy_truck = ctruck.copy()
    copy_drone = [x for x in copy_drone if x not in elements]
    copy_truck += elements
    copy_drone.sort(key=lambda x: int(x))
    copy_truck.sort(key=lambda x: int(x))
    return copy_drone, copy_truck


def parse_drone_solution(chromosome, jobs, names):

    result = list()
    for i, item in enumerate(jobs):
        for j, d in enumerate(chromosome[i]):
            if d == 1:
                result.append(("D#{}[{}]".format(j, names[i]), round(item, 2)))

    return result


def main():
    # Get inputs - uncomment when ready for many tests
    # num_custom, dist_center, max_drone = get_inputs()

    dist_center = 0
    num_drones = 1
    max_drone_time = 30
    num_clients = 10
    # Drone solver: 'genetic', 'SJF', 'random'
    drone_solver = 'genetic'

    # Calculate drone and truck matrices
    ratio = max_drone_time / 2
    print("Max distance for drones: {}".format(ratio))

    BASE_PATH = "20140810T123437v1/"
    TAU_FILE = BASE_PATH + "tau.csv"
    TAUPRIME_FILE = BASE_PATH + "tauprime.csv"
    CPRIME_FILE = BASE_PATH + "Cprime.csv"

    tmatrix, tdrone, cdrone, ctruck = get_matrices_test_psp(TAU_FILE, TAUPRIME_FILE, CPRIME_FILE, dist_center, ratio, num_clients)

    cdrone_translated = [int(x) for x in cdrone]

    truck_solution, truck_time = solveTSP(ctruck, tmatrix, dist_center)
    drone_solution, drone_time = droneSolver(cdrone, num_drones, tdrone, dist_center, cdrone_translated,
                                             solver_type=drone_solver)

    base_result = max(drone_time, truck_time)

    max_savings = 0
    candidate_drone = cdrone.copy()
    candidate_truck = ctruck.copy()
    candidate_drone_solution = drone_solution.copy()
    candidate_truck_solution = truck_solution.copy()
    candidate_drone_time = drone_time
    candidate_truck_time = truck_time

    groups = [x for x in range(1, len(cdrone) + 1)]
    for i in groups:
        comby = [list(x) for x in combinations(cdrone, i)]
        for el in comby:
            new_drone, new_truck = swap(cdrone, ctruck, el)
            logging.info("Comination: From drones {} --> to truck".format(el))
            try:
                cdrone_translated = [int(x) for x in cdrone]
            except Exception as e:
                print("Exception. Point names are no numbers {}".format(e))
                pass

            print("Combinations: {}\nDrone customers: {}\nTruck customers: {}\n".format(el, len(new_drone),
                                                                                        len(new_truck)))
            partial_truck_solution, partial_truck_time = solveTSP(new_truck, tmatrix, dist_center)
            partial_drone_solution, partial_drone_time = droneSolver(new_drone, num_drones, tdrone,
                                                                     dist_center,
                                                                     cdrone_translated,
                                                                     solver_type=drone_solver)
            new_result = max(partial_drone_time, partial_truck_time)
            savings = base_result - new_result
            if savings > max_savings:
                candidate_drone = new_drone
                candidate_truck = new_truck
                candidate_drone_solution = partial_drone_solution
                candidate_truck_solution = partial_truck_solution
                candidate_drone_time = partial_drone_time
                candidate_truck_time = partial_truck_time
                max_savings = savings

    # Format ORIGINAL result

    result = adjust_index(ctruck, truck_solution)
    route = result
    drone_routes = cdrone
    print("Original Truck route: {}".format(route))
    print("Original Drone solution: {}".format(drone_routes))

    # print("Drone solution: {}".format(drone_solution))
    print("Original Truck time: {}".format(truck_time))
    print("Original Drone time: {}".format(drone_time))
    print("Original Total time: {}".format(max(drone_time, truck_time)))

    # Format Optimun result
    result = adjust_index(candidate_truck, candidate_truck_solution)

    route = result
    drone_routes = candidate_drone

    assert all([x not in route for x in drone_routes]), "Duplicated customers"
    assert all([x not in drone_routes for x in route]), "Duplicated customers"

    print("Truck route(*): {}".format(route))
    print("Drone solution(*): {} --> {}".format(drone_routes, candidate_drone_solution))
    print("Truck time(*): {}".format(candidate_truck_time))
    print("Drone time(*): {}".format(candidate_drone_time))
    print("Total time(*): {}".format(max(candidate_drone_time, candidate_truck_time)))
    print("Performance: +{}%".format(100 - round(100 * max(candidate_drone_time, candidate_truck_time) / float(
        max(drone_time, truck_time))), 2))

    return 0

if __name__ == '__main__':
    main()
