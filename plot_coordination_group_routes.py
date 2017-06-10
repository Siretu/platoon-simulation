# -*- coding: utf-8 -*-

import copy
import httplib
import os
import random
import time
from cStringIO import StringIO

import matplotlib.pyplot as plt
import numpy as np

import clusteralg as cl
import convex_optimization as cv
import pairwise_planning as pp
import route_calculation as rc


# units a in km and h

# cd /mnt/data/Uni/PHD/truck_platoon_planning/osrm/osrm_backend_edge_ids/build/
# ./osrm-routed sweden-latest.osrm


def build_path_data_sets(problem_data, route_links, route_weights, route_lat, route_lon, start_times, arrival_dlines,
                         active_trucks, start_poses=None):
    path_data_sets = {}
    for k in active_trucks:
        path_data = {'path': route_links[k],
                    'path_set': set(path_data['path']),
                    'path_weights': route_weights[k],
                    'path_lat': route_lat[k],
                    'path_lon': route_lon[k],
                    't_s': start_times[k],
                    'arrival_dline': arrival_dlines[k]
                     }
        if start_poses is not None:
            path_data['start_pos'] = start_poses[k]
        else:
            path_data['start_pos'] = {'i': 0, 'x': 0.}
        path_data_sets[k] = path_data

    return path_data_sets


def transform_path(path):
    lat = np.array(path['path_lat'], dtype='float') / 1e6 / 180. * np.pi
    lon = np.array(path['path_lon'], dtype='float') / 1e6 / 180. * np.pi

    x = 128. / np.pi * (lon + np.pi)
    y = 128. / np.pi * (np.pi - np.log(np.tan(np.pi / 4. + lat / 2.)))

    return [x, y, lat, lon]


def get_platoon_segment_inds(path, plan):
    cum_w = np.cumsum(path['path_weights'])
    start = np.where(cum_w > plan['d_s'])[0][0] - 1
    stop = np.where(cum_w > cum_w[-1] - plan['d_sp'])[0][0]

    return start, stop


def one_simulation(K):
    # %%%%%%%%%%%% Generate Routes and start times / deadlines
    print 'retrieving the routes'

    route_links, route_weights, K_set, routes, route_lat, route_lon = rc.get_routes_from_osrm(K, True)

    # debug: give all trucks the same route
    #
    # route_links = {k:route_links[K_set[0]] for k in K_set}
    # route_weights = {k:route_weights[K_set[0]] for k in K_set}

    problem_data['K'] = copy.copy(K_set)
    active_trucks = copy.copy(K_set)

    # test data, will be generated from osrm
    # route_links = {0:[1,2,3,4,5], 1:[12,13,3,4,5], 3:[21,22,23,1,2,3,24]}
    # route_weights = {0:[5.5,5.,6.,4.,8.], 1:[5.,7.,6.,4.,8.], 3:[7.,2.,3.,5.5,5.,6.,100.]}
    # TODO: move into problem data ???

    # TODO: might need to change this to dicts because of truck dropping out

    start_times = {k: random.random() * start_interval for k in problem_data['K']}
    # start_times = {0:0.14,1:0.14,3:0.01}
    arrival_dlines = {k: start_times[k] + sum(route_weights[k]) / problem_data['v_nom'] for k in problem_data['K']}

    # %%%%%%%%%%%% loop until all trucks have arrived

    # %%%%%% build coordination graph
    print 'building coordination graph'
    path_data_sets = build_path_data_sets(problem_data, route_links, route_weights, route_lat, route_lon, start_times,
                                          arrival_dlines, active_trucks)

    default_plans = pp.get_default_plans(path_data_sets)

    G_p = pp.build_G_p(path_data_sets, default_plans)

    # %%%%%% clustering
    print 'clustering'
    N_f, N_l, leaders, counter = cl.clustering(G_p, 'greedy')

    # %%%%%% joint optimization for all clusters
    print 'starting convex optimization'
    plans = pp.retrieve_adapted_plans(path_data_sets, G_p, leaders, default_plans)
    followers_dict = cv.get_followers(N_l, leaders)

    def plot_one_group(n_l):
        #  n_l = 0
        n_f = followers_dict[n_l]
        n_all = [n_l] + n_f

        xref = 400
        yref = 300

        XYLL = {n: transform_path(path_data_sets[n]) for n in n_all}

        all_x = np.array([])
        all_y = np.array([])
        all_lat = np.array([])
        all_lon = np.array([])
        for n in n_all:
            all_x = np.concatenate((all_x, XYLL[n][0]))
            all_y = np.concatenate((all_y, XYLL[n][1]))
            all_lat = np.concatenate((all_lat, XYLL[n][2]))
            all_lon = np.concatenate((all_lon, XYLL[n][3]))

            #  print '{},{}   {},{}'.format(all_lat[0]*180/np.pi,all_lon[0]*180/np.pi,all_lat[-1]*180/np.pi,all_lon[-1]*180/np.pi)

        # compute zoom factor
        x_w = all_x.max() - all_x.min()
        zoom_x = np.log2(xref / x_w)
        y_w = all_y.max() - all_y.min()
        zoom_y = np.log2(yref / y_w)
        max_zoom = min([zoom_x, zoom_y])
        zoom = np.floor(max_zoom)

        # zoom XYLL

        all_x_zoom = all_x * 2 ** zoom
        all_y_zoom = all_y * 2 ** zoom

        for n in n_all:
            XYLL[n][0] = (XYLL[n][0] - all_x.min()) * 2 ** zoom
            XYLL[n][0] += (xref - (all_x_zoom.max() - all_x_zoom.min())) / 2  # center
            XYLL[n][1] = (XYLL[n][1] - all_y.min()) * 2 ** zoom
            XYLL[n][1] += (yref - (all_y_zoom.max() - all_y_zoom.min())) / 2 - 3  # center

        c_lat = (all_lat.max() + all_lat.min()) / 2. * 180 / np.pi
        c_lon = (all_lon.max() + all_lon.min()) / 2. * 180 / np.pi

        mapquest_url = 'open.mapquestapi.com'
        request = '/staticmap/v4/getmap?key=98P6M5uV2XIEeJO1KGAx0WXpOER1HA1X&size={},{}&zoom={}&center={:.6f},{:.6f}'.format(
            int(xref), int(yref), int(zoom), c_lat, c_lon)

        conn = httplib.HTTPConnection(mapquest_url)
        conn.request("GET", request)
        map_resp = conn.getresponse()
        data = map_resp.read()
        image = StringIO(data)
        from PIL import Image
        image.seek(0)
        map_image = Image.open(image)
        conn.close()
        map_im_np = np.array(map_image)

        # plotting
        plt.figure()
        plt.imshow(map_im_np)
        colors = ['b', 'g', 'r', 'c', 'm', 'b', 'g', 'r', 'c', 'm', 'b', 'g', 'r', 'c', 'm', 'b', 'g', 'r', 'c', 'm',
                  'b', 'g', 'r', 'c', 'm']

        plt.plot(XYLL[n_l][0], XYLL[n_l][1], '-k', linewidth=2.)
        plt.plot(XYLL[n_l][0][0], XYLL[n_l][1][0], '*k', markersize=10.)
        for i, n in enumerate(n_f):
            plt.plot(XYLL[n][0], XYLL[n][1], '--' + colors[i], linewidth=2.)
            plt.plot(XYLL[n][0][0], XYLL[n][1][0], '*' + colors[i], markersize=10.)

            start, stop = get_platoon_segment_inds(path_data_sets[n], plans[n])
            plt.plot(XYLL[n][0][start], XYLL[n][1][start], '^' + colors[i], markersize=10.)
            plt.plot(XYLL[n][0][stop], XYLL[n][1][stop], 'v' + colors[i], markersize=10.)

        plt.xlim((0, xref))
        plt.ylim((0, yref))
        ax = plt.gca()
        ax.invert_yaxis()
        plt.show(block=False)

        plt.savefig(new_dir + '{}.png'.format(n_l))

    max_plot_num = 10
    new_dir = './plots/routes/{}/'.format(time.time())
    os.mkdir(new_dir)
    for i, n_l in enumerate(list(N_l)):
        if i == max_plot_num:
            break
        try:
            plot_one_group(n_l)
        except:
            print 'Could not plot for leader {}'.format(n_l)
    return


def plot_results(results):
    f_relat_after_convex_gr = []
    Ks = list(results)
    Ks.sort()

    for K in Ks:
        f_relat_after_convex_gr.append(results[K]['f_relat_after_convex_gr'])

    fig, ax1 = plt.subplots()
    ax1.plot(Ks, f_relat_after_convex_gr, 'b-+')

    plt.show(block=False)

    return


# %%%%%%%%%%%% Simulation configuration

# time_step = 1./60*5

# %%%%%%%%%%%% Definition of Problem Data

problem_data = {'v_max': 90. / 3.6, 'v_min': 70. / 3.6, 'v_nom': 80. / 3.6, 'min_intersection_length': 5e3}

# problem_data['F'] = {'F0':1, 'F0p':0.9, 'F1':1./80, 'F1p':1./80*.9} # fuel model
# problem_data['F'] = {'F0':0., 'F0p':1.-eta, 'F1':2./80, 'F1p':2./80*eta} # fuel model

# everything in SI
c_0D = .6
c_PD = .4
m = 4e4
c_r = 7e-3
A = 10.

p_0 = 5.3628e-4
p_1 = 5.1526e-8

g = 9.81
rho = 1.225

# f = (Fm1/v + F0 + F2*v**2)*d
bar_Fm1 = p_0
bar_F0 = c_r * m * g * p_1
bar_F2 = .5 * rho * A * c_0D * p_1
bar_FP2 = .5 * rho * A * c_PD * p_1

eta = 0.6

# todo: fill in the calculations for F0,...

# v_nom = problem_data['v_nom']/3.6
v_nom = problem_data['v_nom']
F0 = bar_F0 - bar_F2 * v_nom ** 2
F1 = 2 * bar_F2 * v_nom
F0p = bar_F0 - eta * bar_F2 * v_nom ** 2
F1p = 2 * eta * bar_F2 * v_nom
# adapt to km and h
# F0 = F0*1000. # m --> km
# F0p = F0p*1000.
# F1 = F1*1000./3.6 # m/s --> km/h
# F1p = F1p*1000./3.6

problem_data['F'] = {'F0': F0, 'F0p': F0p, 'F1': F1, 'F1p': F1p}  # fuel model

# Savings with this fuel model at v_nom 0.15913792711482916 (F0+F1*v_nom - F0p - F1p*v_nom)/(F0 + F1*v_nom)


# K = 3
# problem_data['K'] = [0,1,3] #range(K) # trucks
# active_trucks = [0,1,3] #range(K)

problem_data['eps'] = 1e-6  # threshold to consider to positions coinciding

start_interval = 2. * 3600

# time gap for spontanous platooning
time_gap = 60.  # one minute

# folder = './testroutes/'
# route_links, route_weights, K_set, routes = rc.get_routes_from_pkl(folder)

K = 100
one_simulation(K)


# sp_im = scipy.misc.imread(image)
# conn.close()

# f = open('./testsimres/{}.pkl'.format(time.time()),'w')
# pkl.dump(results,f)
# f.close()

# plot_results(results)
