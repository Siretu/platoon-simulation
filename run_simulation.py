# -*- coding: utf-8 -*-

import cPickle as pkl
import copy
import os
import random
import sys
import time

import clusteralg as cl
import constants
import convex_optimization as cv
import pairwise_planning as pp
import route_calculation as rc
# import visualizer.visualizer as vis


# units a in km and h

# cd /mnt/data/Uni/PHD/truck_platoon_planning/osrm/osrm_backend_edge_ids/build/
# ./osrm-routed sweden-latest.osrm
from platooning.platooning_methods import GreedyPlatooning


def build_path_data_sets(route_links, route_weights, start_times, arrival_dlines, active_trucks,
                         start_poses=None):
    path_data_sets = {}
    for k in active_trucks:
        path_data = {}
        path_data['path'] = route_links[k]
        path_data['path_set'] = set(path_data['path'])
        path_data['path_weights'] = route_weights[k]
        if start_poses != None:
            path_data['start_pos'] = start_poses[k]
        else:
            path_data['start_pos'] = {'i': 0, 'x': 0.}
        path_data['t_s'] = start_times[k]
        path_data['arrival_dline'] = arrival_dlines[k]
        path_data_sets[k] = path_data

    return path_data_sets


def save_routes_to_pkl(folder, routes):
    for i in range(len(routes)):
        f = open('{}{}.pkl'.format(folder, i), 'w')
        pkl.dump(routes[i], f, protocol=pkl.HIGHEST_PROTOCOL)


def get_path_data_sets(folder):
    f = open('{}paths.pkl'.format(folder), 'r')
    return pkl.load(f)

def save_path_data_sets(path_data_set, folder):
    f = open('{}paths.pkl'.format(folder), 'w')
    pkl.dump(path_data_set, f, protocol=pkl.HIGHEST_PROTOCOL)

def generate_routes(K, folder):
    if not os.path.exists(folder):
        os.makedirs(folder)
    route_links, route_weights, K_set, routes = rc.get_routes_from_osrm(K, True)
    active_trucks = copy.copy(K_set)

    start_times = {k: random.random() * constants.start_interval for k in K_set}
    # start_times = {0:0.14,1:0.14,3:0.01}
    arrival_dlines = {k: start_times[k] + sum(route_weights[k]) / constants.v_nom for k in K_set}

    # %%%%%% build coordination graph
    # Calculate who can adapt to who
    print 'building coordination graph'
    path_data_sets = build_path_data_sets(route_links, route_weights, start_times, arrival_dlines, active_trucks)
    save_routes_to_pkl(folder, routes)
    save_path_data_sets(path_data_sets, folder)

def simulation(folder, method):
    results = {}
    print 'retrieving the routes'
    path_data_sets = get_path_data_sets(folder)
    default_plans = pp.get_default_plans(path_data_sets)
    print "computing the coordination graph"
    G_p = pp.build_G_p(path_data_sets, default_plans)

    ## Clustering
    print "clustering"
    N_f, N_l, leaders, counter = method.clustering(G_p)

    ## Joint optimization for all clusters
    print "convex optimization"
    plans = pp.retrieve_adapted_plans(path_data_sets, G_p, leaders, default_plans)
    T_stars, f_opt_total, f_init_total = cv.optimize_all_clusters(leaders, N_l, plans, path_data_sets)

    ## Calculate fuel consumption
    f_total_default = pp.total_fuel_consumption(default_plans)
    f_total_before_convex = pp.total_fuel_consumption(plans)
    f_relat_before_convex = (f_total_default - f_total_before_convex) / f_total_default
    f_total_after_convex = float(f_total_before_convex - (f_init_total - f_opt_total))
    f_relat_after_convex = (f_total_default - f_total_after_convex) / f_total_default
    f_total_spont_plat = pp.total_fuel_comsumption_spontanous_platooning(path_data_sets, default_plans, constants.time_gap)
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
    results['upper_bound'] = cl.upper_bound(G_p)
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

        f = open('./simres/{}__{}.pkl'.format(i, time.time()), 'w')
        pkl.dump(results, f, protocol=pkl.HIGHEST_PROTOCOL)
        f.close()

    # bmp.scatter(0, 0, latlon=True)
    # plot_results(results)

if __name__ == "__main__":
    main()
