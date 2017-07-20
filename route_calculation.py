# -*- coding: utf-8 -*-

import cPickle as pkl
import os
import random
import re

from constants import START_INTERVAL, V_NOM
import sample_routes as sr


def get_routes_from_pkl(folder):
    route_links = {}
    route_weights = {}
    K_set = []
    routes = []

    p = re.compile('(\d+)\.pkl')

    for f_name in os.listdir(folder):

        m = p.match(f_name)
        if m:
            k = int(m.group(1))
            K_set.append(k)

            f = open('{}{}'.format(folder, f_name), 'r')
            data = pkl.load(f)
            f.close()

            route_links[k] = data['node_ids']
            route_weights[k] = data['link_lengths']

            routes.append(data)

    return route_links, route_weights, K_set, routes


def get_routes_from_osrm(number, verbose=False):
    route_links = {}
    route_weights = {}
    K_set = []
    routes = []
    k = 0

    while len(K_set) < number:
        route = sr.calc_route_retry(random.randint(0, 1e10))
        #    gc.collect()
        if route:
            K_set.append(k)

            route_links[k] = route['node_ids']
            route_weights[k] = route['link_lengths']

            routes.append(route)

            k += 1
            if k % 10 == 0:
                print '{} of {} routes calculated'.format(k, number)

    return route_links, route_weights, K_set, routes


def build_path_data_sets(route_links, route_weights, start_times, arrival_dlines, active_trucks, routes):
    # path_data_sets = {}
    for k in active_trucks:
        routes[k]['path'] = route_links[k]
        routes[k]['path_set'] = set(route_links[k])
        routes[k]['path_weights'] = route_weights[k]
        routes[k]['t_s'] = start_times[k]
        routes[k]['arrival_dline'] = arrival_dlines[k]
        # path_data = {'path': route_links[k],
        #              'path_set': set(route_links[k]),
        #              'path_weights': route_weights[k],
        #              't_s': start_times[k],
        #              'arrival_dline': arrival_dlines[k]
        #              }
        routes[k]['start_pos'] = {'i': 0, 'x': 0.}
        # path_data_sets[k] = path_data
    return {i:x for i,x in enumerate(routes)}


def get_path_data_sets(folder):
    f = open('{}paths.pkl'.format(folder), 'r')
    return pkl.load(f)


def get_routes(folder):
    f = open('{}routes.pkl'.format(folder), 'r')
    return pkl.load(f)


def save_path_data_sets(path_data_set, folder):
    f = open('{}paths.pkl'.format(folder), 'w')
    pkl.dump(path_data_set, f, protocol=pkl.HIGHEST_PROTOCOL)


def save_routes(routes, folder):
    f = open('{}routes.pkl'.format(folder), 'w')
    pkl.dump(routes, f, protocol=pkl.HIGHEST_PROTOCOL)


def collect_route_info(routes):
    result = {}
    for x in routes:
        for i,y in enumerate(x['node_ids']):
            if y not in result:
                lat = x['node_coords_lat'][i]
                lon = x['node_coords_lon'][i]
                if i < len(x['link_lengths']):
                    length = x['link_lengths'][i]
                else:
                    length = 0
                result[y] = {'lat': lat, 'lon': lon, 'length': length}
    return result


def generate_routes(K, folder=""):
    route_links, route_weights, K_set, routes = get_routes_from_osrm(K, True)

    start_times = {k: random.random() * START_INTERVAL for k in K_set}
    arrival_dlines = {k: start_times[k] + sum(route_weights[k]) / V_NOM for k in K_set}
    route_info = collect_route_info(routes)

    # %%%%%% build coordination graph
    print 'building coordination graph'
    path_data_sets = build_path_data_sets(route_links, route_weights, start_times, arrival_dlines, K_set, routes)
    if folder:
        if not os.path.exists(folder):
            os.makedirs(folder)
        save_routes(route_info, folder)
        # save_route_info(route_info, folder)
        save_path_data_sets(path_data_sets, folder)
    return path_data_sets
