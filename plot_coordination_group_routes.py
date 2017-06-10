# -*- coding: utf-8 -*-

import httplib
import os
import time
from cStringIO import StringIO

import matplotlib.pyplot as plt
import numpy as np

import convex_optimization as cv
import pairwise_planning as pp
import route_calculation as rc
from platooning.platooning_methods import GreedyPlatooning


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
    path_data_sets = rc.generate_routes(K)

    default_plans = pp.get_default_plans(path_data_sets)

    G_p = pp.build_G_p(path_data_sets, default_plans)

    # %%%%%% clustering
    print 'clustering'
    N_f, N_l, leaders, counter = GreedyPlatooning().clustering(G_p)

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
    new_dir = '.{}/'.format(time.time())
    os.mkdir(new_dir)
    for i, n_l in enumerate(list(N_l)):
        if i == max_plot_num:
            break
        try:
            plot_one_group(n_l)
        except Exception as e:
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

K = 100
one_simulation(K)
