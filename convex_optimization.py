# -*- coding: utf-8 -*-
"""
Created on Wed Aug  6 13:50:30 2014

Convex optimization of the speed profiles given by the clustering algorithm

@author: sebastian
"""
import cvxopt
import cvxopt as co
import matplotlib.pyplot as plt
import numpy as np

import constants

LEADER = -1
NONE = -2


def get_W_n_I_msp(leader, followers, start_times, arrival_dlines, arrival_times, merge_dists, split_dists, merge_times,
                  split_times, default_speed):
    # SEGMENTATION
    # W_n: dict of the segment lengths, I_m, I_sp: dicts with first/last common segment in W_n[leader],
    # T_n_def: dict with lists of the default speed profiles as traversal times

    min_seg_dur = 1e-6  # minmum duration of a segment to compensate for numerical inaccuracy

    # create W for the platoon leader and remember which follower is resp for which point in time
    merge_order = [(time, truck, 'M') for truck, time in merge_times.iteritems()] + \
                  [(time, truck, 'Sp') for truck, time in split_times.iteritems()]
    merge_order.sort()
    W_n = {n: [] for n in [leader] + followers}
    T_n = {n: [] for n in [leader] + followers}
    I_m = {}  # key: followers, value: index in W_n[leader] where they platoon for the first time
    I_sp = {}  # key: followers, value: index in W_n[leader] where they platoon for the last time
    for truck in followers:
        if merge_dists[truck] != 0:
            W_n[truck].append(merge_dists[truck])
            T_n[truck].append(merge_times[truck] - start_times[truck])

    last_time = start_times[leader]
    active_trucks = set([leader])
    i_cur = 0
    for poi in merge_order:
        time, truck, maneuvre = poi
        if time > last_time + min_seg_dur:  # we've advanced in time --> add segment to all active trucks
            curr_W_n = (time - last_time) * default_speed
            for k in active_trucks:
                W_n[k].append(curr_W_n)
                T_n[k].append(time - last_time)
            i_cur += 1

        if maneuvre == 'M':
            active_trucks.add(truck)
            I_m[truck] = i_cur
        if maneuvre == 'Sp':
            active_trucks.remove(truck)
            I_sp[truck] = i_cur - 1

        last_time = time

    if arrival_times[leader] > last_time + min_seg_dur:
        W_n[leader].append((arrival_dlines[leader] - last_time) * default_speed)
        T_n[leader].append(arrival_times[leader] - last_time)
    for truck in followers:
        if split_dists[truck] != 0:
            W_n[truck].append(split_dists[truck])
            T_n[truck].append(arrival_times[truck] - sum(T_n[truck]) - start_times[truck])

    return W_n, I_m, I_sp, T_n


def get_W_n_I_msp_simple_call(leader, followers, path_data_sets, plans):
    # reformates data and calls get_W_n_I_msp

    start_times = {k: path_data_sets[k]['t_s'] for k in [leader] + followers}
    arrival_dlines = {k: path_data_sets[k]['arrival_dline'] for k in [leader] + followers}
    arrival_times = {k: plans[k]['t_a'] for k in [leader] + followers}
    merge_dists = {k: plans[k]['d_s'] for k in followers}
    split_dists = {k: plans[k]['d_sp'] for k in followers}
    merge_times = {k: plans[k]['t_m'] for k in followers}
    split_times = {k: plans[k]['t_sp'] for k in followers}
    default_speed = constants.v_nom
    # TODO: adapt to leader specific speed

    return get_W_n_I_msp(leader, followers, start_times, arrival_dlines, arrival_times, merge_dists, split_dists,
                         merge_times, split_times, default_speed)


def get_P_n(merge_dists, split_dists, W_n, leader, followers):
    P_n = {n: [] for n in followers}
    for f in followers:
        if merge_dists[f] == 0 and split_dists[f] == 0:
            P_n[f] = len(W_n[f]) * [True]
        if merge_dists[f] == 0 and split_dists[f] != 0:
            P_n[f] = (len(W_n[f]) - 1) * [True] + [False]
        if merge_dists[f] != 0 and split_dists[f] == 0:
            P_n[f] = [False] + (len(W_n[f]) - 1) * [True]
        if merge_dists[f] != 0 and split_dists[f] != 0:
            P_n[f] = [False] + (len(W_n[f]) - 2) * [True] + [False]
    P_n[leader] = len(W_n[leader]) * [False]

    return P_n


def get_P_n_simple_call(W_n, leader, followers, plans):
    # reformates data and calls get_P_n

    merge_dists = {k: plans[k]['d_s'] for k in followers}
    split_dists = {k: plans[k]['d_sp'] for k in followers}
    return get_P_n(merge_dists, split_dists, W_n, leader, followers)


def get_A(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp):
    # the order is leader, followers

    all_trucks = [leader] + followers
    segment_nums = [len(W_n[n]) for n in all_trucks]  # number of segments for each truck
    total_segments = sum(segment_nums)
    offsets = np.cumsum(segment_nums[:-1])
    A = np.zeros([1, total_segments])

    # leader positive, follower negative

    # constraint 8c
    for i, f in enumerate(followers):
        if not P_n[f][0]:  # they don't start already platooning
            line = np.zeros((1, total_segments))
            line[0, offsets[i]] = -1  # follower
            for j in range(I_m[f]):  # leader
                line[0, j] = 1
            A = np.vstack([A, line])

    # constraint 8d
    for i, f in enumerate(followers):
        for ii, j in enumerate(range(I_m[f], I_sp[f] + 1)):
            line = np.zeros((1, total_segments))
            line[0, j] = 1
            if P_n[f][0]:  # check if they platoon already on the first link
                line[0, offsets[i] + ii] = -1
            else:
                line[0, offsets[i] + ii + 1] = -1
            A = np.vstack([A, line])

    A = co.matrix(A[1:, :])  # get rid of the intial zero
    return A


def get_b(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp):
    # calculates the right-hand-side of the equality constraints

    b = np.zeros([1, 1])

    # constraint 8c
    for i, f in enumerate(followers):
        if not P_n[f][0]:  # they don't start already platooning
            line = start_times[f] - start_times[leader]
            b = np.vstack([b, line])

    # constraint 8d
    for i, f in enumerate(followers):
        for ii in range(I_m[f], I_sp[f] + 1):
            line = np.zeros((1, 1))
            b = np.vstack([b, line])

    b = co.matrix(b[1:, :])

    return b


# TODO: header aufräumen
def get_G(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp):
    all_trucks = [leader] + followers
    segment_nums = [len(W_n[n]) for n in all_trucks]  # number of segments for each truck
    total_segments = sum(segment_nums)
    offsets = np.concatenate([np.array([0]), np.cumsum(segment_nums[:-1])])

    # constraints 8a
    G = -1 * np.eye(total_segments)
    # constraints 8b
    G = np.vstack([G, np.eye(total_segments)])

    # constraints 8c
    for i, k in enumerate(all_trucks):
        line = np.zeros((1, total_segments))
        line[0, offsets[i]:offsets[i] + segment_nums[i]] = np.ones([1, segment_nums[i]])
        G = np.vstack([G, line])

    G = co.matrix(G)

    return G


def get_h(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp):
    all_trucks = [leader] + followers

    # constraints 8a
    h = []
    for k in all_trucks:
        for w in W_n[k]:
            h.append(np.array(-1 * w / v_max, ndmin=1))
    # constraints 8b
    for k in all_trucks:
        for w in W_n[k]:
            h.append(np.array(w / v_min, ndmin=1))

    # constraints 8c
    h.append(np.array([arrival_dlines[k] for k in all_trucks]) - np.array([start_times[k] for k in all_trucks]))
    return co.matrix(np.concatenate(h))


def dict_to_array(val_dict, leader, followers):
    all_trucks = [leader] + followers
    val_vec = []
    for truck in all_trucks:
        val_vec += val_dict[truck]

    return np.array(val_vec)


def array_to_dict(val_array, leader, followers, segment_nums):
    all_trucks = [leader] + followers
    val_dict = {}
    offsets = np.concatenate([np.array([0]), np.cumsum(segment_nums)])
    for i, truck in enumerate(all_trucks):
        val_dict[truck] = val_array[offsets[i]:offsets[i + 1]]

    return val_dict


def get_F_vec(P_n, leader, followers):
    F0_vec = []
    F1_vec = []
    for t in dict_to_array(P_n, leader, followers):
        if t:
            F0_vec.append(constants.F0p)
            F1_vec.append(constants.F1p)
        else:
            F0_vec.append(constants.F0)
            F1_vec.append(constants.F1)

    F0_vec = np.array(F0_vec)
    F1_vec = np.array(F1_vec)

    return F0_vec, F1_vec


def solve(W_n, P_n, T_n, leader, followers, v_min, v_max, start_times, arrival_dlines, arrival_times, I_m, I_sp):
    A = get_A(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp)
    b = get_b(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp)
    G = get_G(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp)
    h = get_h(W_n, P_n, leader, followers, v_min, v_max, start_times, arrival_dlines, I_m, I_sp)

    T_0 = co.matrix(dict_to_array(T_n, leader, followers))

    W_vec = dict_to_array(W_n, leader, followers)
    F0_vec, F1_vec = get_F_vec(P_n, leader, followers)

    #  F0 is not needed at this point

    def F(T=None, z=None):
        if T is None: return 0, T_0  # initial conditions
        if min(T) <= 0.0: return None  # not in the domain
        T = np.array(T)
        f = np.sum(np.divide(np.multiply(F1_vec, np.power(W_vec, 2)).reshape([1, -1]),
                             T.reshape([1, -1])))  # objective function
        f = co.matrix(f)
        Df = co.matrix(-1 * np.divide(np.multiply(F1_vec, np.power(W_vec, 2)).reshape([1, -1]),
                                      np.power(T, 2).reshape([1, -1])))  # gradient transpose
        if z is None: return f, Df
        H = co.spdiag(z[0] * co.matrix(2 * np.divide(np.multiply(F1_vec, np.power(W_vec, 2)).reshape([1, -1]),
                                                     np.power(T, 3).reshape(
                                                         [1, -1]))))  # compute hessian (spdiag: matlab diag)
        return f, Df, H

    res = co.solvers.cp(F, G=G, h=h, A=A, b=b)
    f_opt = np.array(F(T=res['x'])[0])
    f_initial = np.array(F(T=T_0)[0])
    T_star_array = np.array(res['x'])
    all_trucks = [leader] + followers
    segment_nums = [len(W_n[n]) for n in all_trucks]
    T_star = array_to_dict(T_star_array, leader, followers, segment_nums)
    return T_star, f_opt, f_initial


def optimize_cluster(leader, followers, path_data_sets, plans):
    W_n, I_m, I_sp, T_n = get_W_n_I_msp_simple_call(leader, followers, path_data_sets, plans)
    P_n = get_P_n_simple_call(W_n, leader, followers, plans)

    # debug
    cluster_route_length_ref = 0
    cluster_route_length = 0
    for k in [leader] + followers:
        cluster_route_length_ref += np.sum(path_data_sets[k]['path_weights'])
        cluster_route_length += np.sum(W_n[k])
    if abs(cluster_route_length_ref - cluster_route_length) > 1.:
        print 'total route lenghts do not match. difference: {}'.format(cluster_route_length_ref - cluster_route_length)

    # adapt data
    v_max = constants.v_max
    v_min = constants.v_min
    start_times = {k: path_data_sets[k]['t_s'] for k in [leader] + followers}
    arrival_dlines = {k: path_data_sets[k]['arrival_dline'] for k in [leader] + followers}
    arrival_times = {k: plans[k]['t_a'] for k in [leader] + followers}

    T_star, obj_opt, obj_init = solve(W_n, P_n, T_n, leader, followers, v_min, v_max, start_times, arrival_dlines,
                                      arrival_times, I_m, I_sp)

    # debug!!!
    # plot_result(W_n,P_n,T_star,leader,followers,I_m,start_times)

    return T_star, obj_opt, obj_init


def plot_result(W_n, P_n, T_star, leader, followers, I_m, start_times):
    # todos: pgf, remove default speed, show old grayed out


    all_trucks = [leader] + followers
    segment_nums = [len(W_n[n]) for n in all_trucks]
    plt.figure()
    #  T = array_to_dict(T_star,leader, followers, segment_nums)
    T = T_star
    space_offsets = {leader: 0}
    for f in followers:
        if P_n[f][0]:  # platoons from the start
            space_offsets[f] = sum(W_n[leader][:I_m[f]])
        else:
            space_offsets[f] = sum(W_n[leader][:I_m[f]]) - W_n[f][0]
    print space_offsets
    for truck in all_trucks:
        x = np.concatenate((np.array([0]), np.cumsum(W_n[truck]))) + space_offsets[truck]
        t = np.concatenate((np.array([0]), np.cumsum(T[truck]))) + start_times[truck]
        plt.plot(x, t, '--')
    plt.show(block=False)

    return


# there is something wrong with the calculation of t

def get_followers(N_l, leaders):
    # returns a dict with lists of followers
    followers = {kl: [] for kl in N_l}
    for k in leaders:
        if leaders[k] != LEADER and leaders[k] != NONE:
            followers[leaders[k]].append(k)

    return followers


def optimize_all_clusters(leaders, N_l, plans, path_data_sets):
    # optimizes the speed profiles of all the clusters, returns speed profiles,
    # total fuel consumption before and after the joint optimization

    cvxopt.solvers.options['show_progress'] = False

    followers_dict = get_followers(N_l, leaders)
    T_stars = {}
    f_opt_total = 0.
    f_init_total = 0.
    # TODO: take care of the nones


    for kl in N_l:
        followers = followers_dict[kl]
        T_star, f_opt, f_init = optimize_cluster(kl, followers, path_data_sets, plans)
        for k in [kl] + followers:
            T_stars[k] = T_star[k]
            f_opt_total += f_opt
            f_init_total += f_init

    return T_stars, f_opt_total, f_init_total


def get_platoon_size_stats(leaders, N_l, plans, path_data_sets):
    # calculates what distance was traveled in what kind of platoon sizes


    followers_dict = get_followers(N_l, leaders)
    stats = {1: 0.}

    for kl in N_l:
        followers = followers_dict[kl]
        stats_cluster = get_platoon_size_stats_one_cluster(kl, followers, path_data_sets, plans)
        for k in stats_cluster:
            if k in stats:
                stats[k] += stats_cluster[k]
            else:
                stats[k] = stats_cluster[k]

    NONE = -2
    for k in leaders:
        if leaders[k] == NONE:
            stats[1] += np.sum(path_data_sets[k]['path_weights'])

    return stats


def get_platoon_size_stats_one_cluster(leader, followers, path_data_sets, plans):
    # reformates data and calls get_W_n_I_msp

    start_times = {k: path_data_sets[k]['t_s'] for k in [leader] + followers}
    arrival_dlines = {k: path_data_sets[k]['arrival_dline'] for k in [leader] + followers}
    arrival_times = {k: plans[k]['t_a'] for k in [leader] + followers}
    merge_dists = {k: plans[k]['d_s'] for k in followers}
    split_dists = {k: plans[k]['d_sp'] for k in followers}
    merge_times = {k: plans[k]['t_m'] for k in followers}
    split_times = {k: plans[k]['t_sp'] for k in followers}
    default_speed = constants.v_nom

    min_seg_dur = 1e-6  # minmum duration of a segment to compensate for numerical inaccuracy

    # create W for the platoon leader and remember which follower is resp for which point in time
    merge_order = [(time, truck, 'M') for truck, time in merge_times.iteritems()] + \
                  [(time, truck, 'Sp') for truck, time in split_times.iteritems()]
    merge_order.sort()

    stats = {k + 1: 0. for k in xrange(len(followers) + 1)}

    for truck in followers:
        if merge_dists[truck] != 0:
            stats[1] += merge_dists[truck]

    last_time = start_times[leader]
    active_trucks = set([leader])
    i_cur = 0
    for poi in merge_order:
        time, truck, maneuvre = poi
        if time > last_time + min_seg_dur:  # we've advanced in time --> add segment to all active trucks
            curr_W_n = (time - last_time) * default_speed
            for k in active_trucks:
                stats[len(active_trucks)] += curr_W_n
            i_cur += 1

        if maneuvre == 'M':
            active_trucks.add(truck)
        if maneuvre == 'Sp':
            active_trucks.remove(truck)

        last_time = time

    if arrival_times[leader] > last_time + min_seg_dur:
        stats[1] += (arrival_dlines[leader] - last_time) * default_speed
    for truck in followers:
        if split_dists[truck] != 0:
            stats[1] += split_dists[truck]

    for k in xrange(len(followers) + 1):
        if stats[k + 1] == 0.:
            stats.pop(k + 1)

    return stats


######################## new simulation code ##################################
'''

## fuel model, linear affine, _p for platoon follower
#F_0
#F_1
#F_0p
#F_1p
F = {'F0':1, 'F0p':0.9, 'F1':1./80, 'F1p':1./80*.9}

leader = 0 # the coordination leader
followers = [1,2,3] # list of the coordination followers
#R # dicitionary with routes as lists of links
#W_r # dicitionary with route length sequences, lists of floats

default_speed = 80
v_max = 90

start_times = {0:0, 1:0, 2:0.1, 3:0.37} # dict with start times
arrival_dlines = {0:1.1, 1:.85, 2: .95, 3:1.4} # dict with arrival deadlines
arrival_times = {0:1, 1:.85, 2: .95, 3:1.4} # dict with arrival times according to speed profile
#start_pos# dict with start positions

# dicts with followers as keys
merge_times = {1:0, 2:0.2, 3:0.4} # nominal merge times
split_times = {1:0.8, 2:0.9, 3:1.0} # nominal split times
#merge_pos # merge positions (x,l)
#split_pos # positions (x,l)
merge_dists = {1:0,2:3.1,3:3.2}# dict with distance from start to merge point
split_dists = {1:3.1,2:6.2,3:0} # dict with distance form split point to destination



W_n, I_m, I_sp, T_n = get_W_n_I_msp(leader, followers, start_times, arrival_dlines, arrival_times, merge_dists, split_dists, merge_times, split_times, default_speed)
P_n = get_P_n(merge_dists, split_dists, W_n, leader, followers)

A = get_A(W_n,P_n,leader, followers, v_max, start_times, arrival_dlines, I_m, I_sp)
b = get_b(W_n,P_n,leader, followers, v_max, start_times, arrival_dlines, I_m, I_sp)
G = get_G(W_n,P_n,leader, followers, v_max, start_times, arrival_dlines, I_m, I_sp)
h = get_h(W_n,P_n,leader, followers, v_max, start_times, arrival_dlines, I_m, I_sp)

x_0 = dict_to_array(T_n,leader, followers)

F0_vec,F1_vec = get_F_vec(F,P_n,leader,followers)

A_np = np.array(A)
b_np = np.array(b)
G_np = np.array(G)
h_np = np.array(h)
x_0 = np.array(x_0)

res, obj_opt, obj_init = solve(W_n,P_n,leader, followers, v_max, start_times, arrival_dlines, arrival_times, I_m, I_sp,F)
T_star = np.array(res['x'])
obj_opt = np.array(obj_opt)
obj_init = np.array(obj_init)
plot_result(W_n,P_n,T_star,leader,followers,I_m,start_times)
plot_result(W_n,P_n,x_0,leader,followers,I_m,start_times)

#################### simple test data #########################################
'''
