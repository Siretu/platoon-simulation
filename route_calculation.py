# -*- coding: utf-8 -*-

import cPickle as pkl
import os
import random
import re

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
            if k % 100 == 0:
                print '{} of {} routes calculated'.format(k, number)

    return route_links, route_weights, K_set, routes
