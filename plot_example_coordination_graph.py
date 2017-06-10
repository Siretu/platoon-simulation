# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import pairwise_planning as pp
import route_calculation as rc
from platooning.platooning_methods import GreedyPlatooning


def plot(K):
    # %%%%%%%%%%%% Generate Routes and start times / deadlines
    print 'retrieving the routes'
    path_data_sets = rc.generate_routes(K)

    # %%%%%% build coordination graph
    print 'building coordination graph'
    default_plans = pp.get_default_plans(path_data_sets)

    G_p = pp.build_G_p(path_data_sets, default_plans)

    N_f_gr, N_l_gr, leaders_gr, counter_gr = GreedyPlatooning().clustering(G_p)
    # create adjecancy matrix

    adj_matrix = np.ones((K, K, 3))
    Klist = range(K)
    Klist.sort()
    for i in G_p:
        for j in G_p[i]:
            ii = Klist.index(i)
            jj = Klist.index(j)
            if G_p[i][j] > 0:
                if leaders_gr[i] == j:
                    adj_matrix[ii, jj, :] = np.array([1., 0., 0.])
                else:
                    adj_matrix[ii, jj, :] = np.array([0., 0., 0.])

    plt.figure()
    plt.imshow(adj_matrix, interpolation="nearest")
    plt.show(block=False)

    return


K = 100

plot(K)