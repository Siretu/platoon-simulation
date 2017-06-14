# -*- coding: utf-8 -*-

import sys

import clusteralg as cl
import constants
import convex_optimization as cv
import pairwise_planning as pp
from platooning.platooning_methods import GreedyPlatooning
from route_calculation import get_path_data_sets


def simulation(folder, method):
    results = {}
    print 'retrieving the routes'
    path_data_sets = get_path_data_sets(folder)
    default_plans = pp.get_default_plans(path_data_sets)
    print "computing the coordination graph"
    G_p = pp.build_G_p(path_data_sets, default_plans)

    # Clustering
    print "clustering"
    N_f, N_l, leaders, counter = method.clustering(G_p)

    # Joint optimization for all clusters
    print "convex optimization"
    plans = pp.retrieve_adapted_plans(path_data_sets, G_p, leaders, default_plans)
    T_stars, f_opt_total, f_init_total = cv.optimize_all_clusters(leaders, N_l, plans, path_data_sets)

    # Calculate fuel consumption
    f_total_default = pp.total_fuel_consumption(default_plans)
    f_total_before_convex = pp.total_fuel_consumption(plans)
    f_relat_before_convex = (f_total_default - f_total_before_convex) / f_total_default
    f_total_after_convex = float(f_total_before_convex - (f_init_total - f_opt_total))
    f_relat_after_convex = (f_total_default - f_total_after_convex) / f_total_default
    f_total_spont_plat = pp.total_fuel_comsumption_spontaneous_platooning(path_data_sets, default_plans, constants.time_gap)
    f_relat_spont_plat = (f_total_default - f_total_spont_plat) / f_total_default
    f_total_no_time = pp.total_fuel_comsumption_no_time_constraints(path_data_sets, default_plans, constants.time_gap)
    f_relat_no_time = (f_total_default - f_total_no_time) / f_total_default

    results['f_relat_spont_plat'] = f_relat_spont_plat
    results['f_relat_no_time'] = f_relat_no_time
    results['f_total_default'] = f_total_default
    results['f_total_before_convex'] = f_total_before_convex
    results['f_total_after_convex'] = f_total_after_convex
    results['f_relat_before_convex'] = f_relat_before_convex
    results['f_relat_after_convex'] = f_relat_after_convex
    results['size_stats'] = cv.get_platoon_size_stats(leaders, N_l, plans, path_data_sets)
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
