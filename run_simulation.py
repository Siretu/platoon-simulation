# -*- coding: utf-8 -*-

import sys
import gc
import clusteralg as cl
import convex_optimization as cv
import pairwise_planning as pp
import statistics
from platooning.assignments import Truck
from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from route_calculation import get_path_data_sets, get_routes
import numpy as np
import constants
from constants import NONE, LEADER
from verifier import verify_truck
import email_settings
import time

HORIZON = 0


def average_fuel_savings(method, folders, horizon=HORIZON, interval=None, cutoff=constants.START_INTERVAL / 2):
    total = 0
    for folder in folders:
        start = time.time()
        result = dynamic_simulation(method, folder=folder, horizon=horizon, interval=interval)
        sliced_result = [x for x in result if x.start_time >= cutoff]
        if len(sliced_result) > 0:
            fuel_saving = 1 - sum([x.current_fuel_consumption() for x in sliced_result]) / sum([x.default_plan.fuel for x in sliced_result])
        else:
            fuel_saving = 0
        total += fuel_saving
        print fuel_saving
        t = time.time()-start
        print "%s: %.3f" % ("Time", t)
            
        email_settings.mail("erikihr@gmail.com", "Simulation results", "Fuel savings (%s): %f%%" % (method, fuel_saving * 100))
        del result
        del sliced_result
        gc.collect()
        print "Ran garbage collect"

    return total / len(folders)


def calculate_platoons(leaders):
    platoons = {truck: 1 for truck in leaders}
    for key in leaders:
        if leaders[key] >= 0:
            platoons[leaders[key]] += 1

    sizes = {}
    for key in platoons:
        size = platoons[key]
        if size > 1:
            if size not in sizes:
                sizes[size] = 0
            sizes[size] += 1
    return sizes


def calculate_similarities(G_p, similarities, prev_leaders):
    methods = [GreedyPlatooning(), RandomPlatooning(), SubModularityPlatooning(False), SubModularityPlatooning(True)]
    leaders = {str(method): method.clustering(G_p, leaders=prev_leaders[i]) for i,method in enumerate(methods)}
    for i,method in enumerate(methods):
        for method2 in methods[i:]:
            m1 = str(method)
            m2 = str(method2)
            old = similarities[m1][m2]
            if len(leaders[m1][1] | leaders[m2][1]) == 0:
                continue
            avg = len((leaders[m1][1] & leaders[m2][1])) / float(len((leaders[m1][1] | leaders[m2][1])))
            new_count = old[1]+1
            new_total = ((old[0] * old[1] + avg) / float(new_count), new_count)
            similarities[m1][m2] = new_total
            similarities[m2][m1] = new_total
    return [x[2] for x in leaders.values()]


def dynamic_simulation(method, folder=None, horizon=HORIZON, interval=None):
    pp.INTERSECTION_CACHE = {}
    print 'retrieving the routes'
    path_data_sets = get_path_data_sets(folder)

    assignments = [Truck(i, path_data_sets[i]) for i in path_data_sets]
    path_data_sets = []
    assignments.sort(key=lambda x: x.start_time)

    update_times = [x.start_time for x in assignments]

    if interval:
        update_times = range(int(update_times[0]), int(update_times[-1] + interval), interval)

    expected = []
    current_trucks = {}
    print "computing the coordination graph"
    G_p = cl.ClusterGraph([])
    leaders = None
    previous_time = 0
    platoons = []
    methods = [GreedyPlatooning(), RandomPlatooning(), SubModularityPlatooning(False), SubModularityPlatooning(True)]
    similarities = {str(x): {str(x): (0,0) for x in methods} for x in methods}
    prev_leaders = [None, None, None, None]
    for time in update_times:
        # print time - previous_time

        current_trucks = {x.id: x for x in assignments if x.start_time <= time + horizon}
        map(lambda x: x.update(time), current_trucks.values())
        current_trucks = {k: v for k, v in current_trucks.items() if not v.done}
        new_trucks = [current_trucks[x] for x in current_trucks if x not in G_p.nodes]

        # G_p = pp.build_graph(current_trucks, G_p)
        G_p.update(new_trucks, current_trucks, time)

        # Clustering
        print "clustering: %d: %d" % (len([x for x in assignments if x.start_time <= time + horizon]), len(current_trucks))
        N_f, N_l, leaders, counter = method.clustering(G_p, leaders=leaders)
        prev_leaders = calculate_similarities(G_p, similarities, prev_leaders)

        for follower in current_trucks:
            if leaders[follower] >= 0:
                current_trucks[follower].change_plan(G_p[follower][leaders[follower]], time)
            else:
                current_trucks[follower].change_plan(current_trucks[follower].calculate_default(leaders[follower] == LEADER), time)

        # if len(current_trucks) > 0:
        #     platoon_ratio = len([x for x in current_trucks.values() if x.is_platoon_follower(time)]) / float(len(current_trucks))
        #     print platoon_ratio
        #     platoons.append(platoon_ratio)
        expected.append(sum([x.current_fuel_consumption() for x in assignments]))

    print expected
    print platoons
    print similarities
    print expected[-1]/expected[0]
    print statistics.average_platoon_time(assignments)
    print statistics.average_plan_length(assignments)
    print statistics.average_plan_length(assignments, False)
    print "Verifying result"
    [verify_truck(x, {y.id: y for y in assignments}) for x in assignments]
    return assignments


def simulation(folder, method, optimize=True):
    results = {}
    print 'retrieving the routes'
    path_data_sets = get_path_data_sets(folder)
    assignments = [Truck(i, path_data_sets[i]) for i in path_data_sets]
    print "computing the coordination graph"
    G_p = pp.build_graph({x.id: x for x in assignments})

    # Clustering
    print "clustering"
    N_f, N_l, leaders, counter = method.clustering(G_p)

    # Joint optimization for all clusters
    plans = pp.retrieve_adapted_plans(assignments, leaders, G_p)

    # Calculate fuel consumption
    f_total_default = pp.total_fuel_consumption({i: x.default_plan for i,x in enumerate(assignments)})
    f_total_before_convex = pp.total_fuel_consumption(plans)
    f_relat_before_convex = (f_total_default - f_total_before_convex) / f_total_default

    if optimize:
        # Joint optimization for all clusters
        print "convex optimization"
        T_stars, f_opt_total, f_init_total = cv.optimize_all_clusters(leaders, N_l, plans, assignments)
        f_total_after_convex = float(f_total_before_convex - (f_init_total - f_opt_total))
        f_relat_after_convex = (f_total_default - f_total_after_convex) / f_total_default
        results['f_total_after_convex'] = f_total_after_convex
        results['f_relat_after_convex'] = f_relat_after_convex

    f_total_spont_plat = pp.total_fuel_consumption_spontaneous_platooning(assignments)
    f_relat_spont_plat = (f_total_default - f_total_spont_plat) / f_total_default
    f_total_no_time = pp.total_fuel_consumption_no_time_constraints(assignments)
    f_relat_no_time = (f_total_default - f_total_no_time) / f_total_default

    results['f_relat_spont_plat'] = f_relat_spont_plat
    results['f_relat_no_time'] = f_relat_no_time
    results['f_total_default'] = f_total_default
    results['f_total_before_convex'] = f_total_before_convex
    results['f_relat_before_convex'] = f_relat_before_convex
    results['size_stats'] = cv.get_platoon_size_stats(leaders, N_l, plans, assignments)
    results['upper_bound'] = cl.get_upper_bound(G_p)
    results['leaders'] = leaders
    return results


def print_simulation_result(result):
    print '------- node selection --------'
    print 'relative fuel savings before convex optimizaton: {}'.format(result["f_relat_before_convex"])
    print 'relative fuel savings after convex optimizaton: {}'.format(result["f_relat_after_convex"])
    print 'total fuel before convex optimizaton: {}'.format(result["f_total_before_convex"])
    print 'total fuel after convex optimizaton: {}'.format(result["f_total_after_convex"])

    print '------- spontanous platooning --------'
    print 'relative fuel savings: {}'.format(result["f_relat_spont_plat"])


def main():
    # Ks = [100, 200, 300, 500, 1000, 1500, 2000, 3000, 5000]
    Ks = [100]

    print sys.argv
    start = int(sys.argv[1])
    stop = int(sys.argv[2])

    for i in xrange(start, stop + 1):
        results = {}
        for K in Ks:
            results[K] = simulation("./testing/testroutes/test100-3/", GreedyPlatooning())
            print_simulation_result(results[K])


if __name__ == "__main__":
    main()
