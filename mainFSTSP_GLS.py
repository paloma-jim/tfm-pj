from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from utils import *
import time
import pdb

INFINITE = 99999999
giant_table = list()


# Creating the route
def create_data_model(route_time_truck):
    data ={}
    data['time_matrix'] = route_time_truck
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def print_solution(solution, solution_time):
    """Prints assignment on console."""
    plan_output = 'Route for vehicle 0:\n'
    print(' -> '.join(list(map(lambda x: str(x), solution))))
    print("Total time: {}".format(solution_time))


def getSolution(manager, routing, assignment):
    """Prints assignment on console."""
    index = routing.Start(0)
    route_time = 0
    route = []
    while not routing.IsEnd(index):
        route.append(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_time += routing.GetArcCostForVehicle(previous_index, index, 0)
    route.append(manager.IndexToNode(index))
    return route, route_time

def Tsubroute(start, end, route, ttruck, tdrone, drone_capacity):

    sindex = [i for i, x in enumerate(route) if x == start][0]
    eindex = [i for i, x in enumerate(route) if x == end][-1]

    if sindex == eindex:
        best = (0, [], [start, end], start, end)
    else:
        initial = route.index(start)
        final = [i for i, x in enumerate(route) if x == end][-1] + 1
        # final = route.index(end)
        subroute = route[initial:final]
        # print([x for x in subroute])
        partial_solution = []
        if len(subroute) > 2:
            for k in subroute[1:len(subroute)-1]:
                partial_solution.append(Ttriple(start, end, k, subroute, ttruck, tdrone, drone_capacity))
        else:
            partial_solution.append(Ttriple(start, end, -1, subroute, ttruck, tdrone, drone_capacity))

        print(partial_solution)
        partial_solution = sorted(partial_solution, key=lambda x: x[0])
        best = partial_solution[0]

    result = dict()
    result['total_time'] = best[0]
    result['drone_nodes'] = best[1]
    result['truck_nodes'] = best[2]
    result['time_drone'] = best[3]
    result['time_truck'] = best[4]
    result['initial_node'] = start
    result['end_node'] = end
    return result


def Ttriple(start, end, k, route, ttruck, tdrone, drone_capacity):
    time_truck = 0
    time_drone = -1
    drone_nodes = []
    truck_nodes = route.copy()

    if k != -1:
        time_drone = tdrone[start][k] + tdrone[k][end]
        if time_drone > drone_capacity:
            time_drone = INFINITE
        truck_nodes.remove(k)
        drone_nodes = [start, k, end]

    initial = start
    for element in truck_nodes:
        time_truck += ttruck[initial][element]
        initial = element
    return max(time_drone, time_truck), drone_nodes, truck_nodes, time_drone, time_truck


def compound_solution(final_result, tab, initial_node, route):
    current_link = final_result[1]['truck_nodes'][0]
    result = list()
    result.append(final_result)
    while current_link != initial_node:
        candidates = list((filter(lambda x: x[1]['end_node'] == current_link, tab)))
        nr = list()
        for x in candidates:
            if x not in nr:
                nr.append(x)
        assert len(nr) == 1
        result.append(nr[-1])
        current_link = nr[0][1]['initial_node']
    result.reverse()
    return result


def Vfn(i, route, tdrone, ttruck, drone_capacity):
    result = list()
    if i == 0:
        result = ([0],)
    else:
        for k in range(i):
            result.append((Vfn(k, route, tdrone, ttruck, drone_capacity)[0] +
                           Tsubroute(route[k], route[i], route, ttruck, tdrone, drone_capacity)['total_time'],
                           Tsubroute(route[k], route[i], route, ttruck, tdrone, drone_capacity)))

    result = min(result, key=lambda x: x[0])
    giant_table.append(result)
    return result


def main():


    drone_battery_lifetime = 40
    num_clients = 10

    BASE_PATH = "20140810T123437v1/"
    TAU_FILE = BASE_PATH + "tau.csv"
    TAUPRIME_FILE = BASE_PATH + "tauprime.csv"
    CPRIME_FILE = BASE_PATH + "Cprime.csv"



    route_time_truck, ttruck, tdrone = get_matrices_test(TAU_FILE, TAUPRIME_FILE, CPRIME_FILE, num_clients)

    # n = 100
    # route_time_truck = generate_random_matrix(n, crop=num_custom)
    # ttruck, tdrone = generate_dicts_from_random(route_time_truck, alpha)
    ###

    data = create_data_model(route_time_truck)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

     # Setting Guided Local Search
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 2
    search_parameters.log_search = True

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        solution, solution_time = getSolution(manager, routing, assignment)
        print_solution(solution, solution_time)

    route = translate(ttruck.keys(), solution)
    print(route)
    print(data['time_matrix'])
    start = time.time()
    final_result = Vfn(len(route) - 1, route, tdrone, ttruck, drone_battery_lifetime)
    print("Execution time = %02d s" % (time.time() - start))
    print("Final result: {}".format(final_result))
    tab = list(filter(lambda x: type(x) == tuple, giant_table))

    final_sol = compound_solution(final_result, tab, route[0], route)
    print("Final solution with drone:\n")
    for x in final_sol:
        print("{}\n".format(x))
    print("Solution with no drone: {}\n{}".format(solution_time, route))


if __name__ == '__main__':
    main()
