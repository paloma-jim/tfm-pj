
import pandas as pd
import numpy as np

INFINITE = 999999999

def get_matrix(filename, crop):
    res = pd.read_csv(filename)
    res = res.drop(res.columns[0], axis=1).values[0:crop, 0:crop]
    return res


def get_matrix_as_dict(filename, crop):
    res = pd.read_csv(filename)
    res = res.drop(res.columns[0], axis=1)
    columns = res.columns[:crop]
    res = pd.DataFrame(res.values[0:crop, 0:crop])
    res.columns = columns
    res.index = res.columns
    return res.to_dict()


def get_truck_drone_dicts(filename, num_custom, factor=0.5):
    result = get_matrix_as_dict(filename, num_custom)
    result_drone = dict()
    for k in result.keys():
        result_drone[k] = dict()
        for j in result.keys():
            if k == j:
                result_drone[k][j] = 0
            else:
                drone_val = int(round(result[k][j] * factor, 0))
                result_drone[k][j] = drone_val if drone_val > 0 else 1
    return result, result_drone

def get_matrices_test(taufile, tauprimefile, cprimefile, num_clients):
    # customers = [x for x in 'ABCDEFGHIJKL']
    customers = [str(x) for x in range(0, num_clients+1)]
    tau = pd.read_csv(taufile, names=customers).astype(int)
    tauprime = pd.read_csv(tauprimefile, names=customers).astype(int)
    cprime = set(pd.read_csv(cprimefile).columns)
    full = set([str(x) for x in range(1, num_clients+1)])
    no_cdrone = [str(x) for x in full.difference(cprime)]
    tau.index = customers
    tauprime.index = customers
    tauprime[no_cdrone] = INFINITE
    tauprime.iloc[[int(x) for x in no_cdrone]] = INFINITE

    return tau.values, tau.to_dict(), tauprime.to_dict()

def get_matrices_test_psp(taufile, tauprimefile, cprimefile, dist_center,ratio, num_clients):
    # customers = [x for x in 'ABCDEFGHIJKL']
    customers = [str(x) for x in range(0, num_clients+1)]
    tau = pd.read_csv(taufile, names=customers).astype(int)
    tauprime = pd.read_csv(tauprimefile, names=customers).astype(int)
    cprime2 = pd.read_csv(cprimefile).columns
    cprime2 = set([int(x) for x in cprime2])
    aux = set([x for x in np.nonzero(tauprime.values[dist_center] >= ratio)[0] if x != dist_center])
    cprime = [str(x) for x in cprime2.difference(aux)]
    full = set([str(x) for x in range(1, num_clients+1)])
    no_cdrone = [str(x) for x in full.difference(cprime)]
    tau.index = customers
    tauprime.index = customers
    tauprime[no_cdrone] = INFINITE
    tauprime.iloc[[int(x) for x in no_cdrone]] = INFINITE
    no_cdrone += [str(dist_center)]
    no_cdrone.sort()
    cprime.sort(key=lambda x: int(x))
    no_cdrone.sort(key=lambda x: int(x))
    return tau.values, tauprime.values, cprime, no_cdrone


# Technical Debt
def translate(list_keys, route):
    result = []
    t = dict()
    lk = [x for x in list_keys]
    try:
        lk.sort(key=lambda x: int(x))
    except Exception as e:
        print("Exception {}".format(e))
        lk.sort()
    for i, x in enumerate(lk):
        t[str(i)] = x
    for item in route:
        result.append(t[str(item)])
    return result


def generate_random_matrix(n=100, max_dist=100, crop=100):
    b = np.random.random_integers(1, max_dist, size=(n, n))
    b_symm = (b + b.T) / 2
    np.fill_diagonal(b_symm, 0)
    return b_symm[0:crop, 0:crop].astype(int)


def generate_dicts_from_random(route_time_truck, factor=0.5):
    drone_time = dict()
    truck_time = dict()
    customer = 'C#{}'
    for i in range(route_time_truck.shape[0]):
        drone_time[customer.format(i)] = dict()
        truck_time[customer.format(i)] = dict()
        for j in range(route_time_truck.shape[0]):
            truck_time[customer.format(i)][customer.format(j)] = route_time_truck[i][j]
            if i == j:
                drone_time[customer.format(i)][customer.format(j)] = 0
            else:
                drone_val = int(round(route_time_truck[i][j] * factor, 0))
                drone_time[customer.format(i)][customer.format(j)] = drone_val if drone_val > 0 else 1
    return truck_time, drone_time
