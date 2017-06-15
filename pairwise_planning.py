import math

import matplotlib.pyplot as plt
import numpy as np

import constants
import cPickle as pkl


def find_route_intersection(route1, route2):
    #  Gives first and last intersecting element on both routes
    path1 = route1['path']
    path2 = route2['path']
    intersections = route1['path_set'] & route2['path_set']

    if not intersections:
        return False

    def find_first_index(known_intersection):
        ## Given a known intersection, find the indexes where route1 and route2 first intersect.
        start_ind1 = np.where(path1 == known_intersection)[0][0]
        start_ind2 = np.where(path2 == known_intersection)[0][0]

        # Look at the route with the shortest previous nodes, and search for previous intersections with the other route
        if start_ind1 >= start_ind2:
            ind2 = np.where(path2[:start_ind2 + 1] == path1[start_ind1 - start_ind2:start_ind1 + 1])[0][0]
            ind1 = ind2 + (start_ind1 - start_ind2)
        elif start_ind1 < start_ind2:
            ind1 = np.where(path1[:start_ind1 + 1] == path2[start_ind2 - start_ind1:start_ind2 + 1])[0][0]
            ind2 = ind1 + (start_ind2 - start_ind1)

        return ind1, ind2

    ind1_start, ind2_start = find_first_index(intersections.pop())

    ind1_split = ind1_start + len(intersections)
    ind2_split = ind2_start + len(intersections)

    # Don't think this is necessary anymore
    # if ind1_split >= path1.shape[0]:
    #     ind1_split -= 1
    #     if ind1_split >= path1.shape[0]:
    #         return False
    # if ind2_split >= path2.shape[0]:
    #     ind2_split -= 1
    #     if ind2_split >= path2.shape[0]:
    #         return False

    return (ind1_start, ind1_split), (ind2_start, ind2_split)


def calculate_default(path_data):
    # returns the arrival time, default speed, and fuel consumption of the default plan

    path_weights = path_data['path_weights']
    start_pos = path_data['start_pos']  # index of the current link (!)
    t_s = path_data['t_s']
    t_d = path_data['arrival_dline']

    end_pos = {'i': len(path_weights) - 1, 'x': path_weights[-1]}
    path_L = get_distance(path_weights, start_pos, end_pos)

    v_d = path_L / (t_d - t_s)  # speed to arrive exactly at the deadline

    if v_d > constants.v_max:
        print "Warning: a truck cannot make its deadline!"
    if v_d <= constants.v_nom:
        t_a = path_L / constants.v_nom + t_s
        v_default = constants.v_nom
    if v_d > constants.v_nom:
        t_a = path_L / v_d + t_s
        v_default = v_d

    f = (constants.F0 + constants.F1 * v_default) * path_L

    return t_a, v_default, f


def calculate_adaptation(ref_path_data, ada_path_data, intersection, ref_default_plan, ada_default_plan, verbose=False):
    #  returns -1 if platooning is not feasible or beneficial, and the fuel saving (positive) if platooning is beneficial
    v_min = constants.v_min
    v_max = constants.v_max
    v_nom = constants.v_nom

    ref_path = ref_path_data['path']
    ref_path_weights = ref_path_data['path_weights']
    ref_start_pos = ref_path_data['start_pos']  # index of the current link (!)
    ref_t_s = ref_path_data['t_s']
    ref_t_d = ref_path_data['arrival_dline']
    ref_merge_ind = intersection[0][0]
    ref_split_ind = intersection[0][1]
    ref_v_def = ref_default_plan['v_default']
    #  ref_f_def = ref_default_plan['f']
    #  ref_t_a_def = ref_default_plan['t_a']

    ada_path = ada_path_data['path']
    ada_path_weights = ada_path_data['path_weights']
    ada_start_pos = ada_path_data['start_pos']
    ada_t_s = ada_path_data['t_s']
    ada_t_d = ada_path_data['arrival_dline']
    ada_merge_ind = intersection[1][0]
    ada_split_ind = intersection[1][1]
    #  ada_v_def = ada_default_plan['v_default']
    ada_f_def = ada_default_plan['f']
    #  ada_t_a_def = ada_default_plan['t_a']

    # check if times overlap at all
    if ref_t_d < ada_t_s or ada_t_d < ref_t_s:
        if verbose:
            print 'Cannot platoon since the time intervals do not overlap!'
        return -1

    Delta_F0 = constants.F0 - constants.F0p
    v_star_slow = max(
        [v_nom * (1 - math.sqrt(1 - constants.F1p / constants.F1 + Delta_F0 / (constants.F1 * v_nom))), v_min])
    v_star_fast = min(
        [v_nom * (1 + math.sqrt(1 - constants.F1p / constants.F1 + Delta_F0 / (constants.F1 * v_nom))), v_max])

    #  try:
    ref_end_pos = {'i': len(ref_path_weights) - 1, 'x': ref_path_weights[-1]}
    ref_first_merge_pos = {'i': ref_merge_ind, 'x': 0.}
    ref_last_split_pos = {'i': ref_split_ind, 'x': ref_path_weights[ref_split_ind]}

    ada_end_pos = {'i': len(ada_path_weights) - 1, 'x': ada_path_weights[-1]}
    ada_first_merge_pos = {'i': ada_merge_ind, 'x': 0.}
    ada_last_split_pos = {'i': ada_split_ind, 'x': ada_path_weights[ada_split_ind]}
    #  except:
    #    print 'fail'

    if get_distance(ref_path_weights, ref_start_pos, ref_last_split_pos) == 0.:  # we have passed the common part
        if verbose:
            print 'Cannot platoon since the leader has passed the common part!'
        return -1
    if get_distance(ada_path_weights, ada_start_pos, ada_last_split_pos) == 0.:
        if verbose:
            print 'Cannot platoon since the follower has passed the common part!'
        return -1

    ref_path_L = get_distance(ref_path_weights, ref_start_pos, ref_end_pos)
    ada_path_L = get_distance(ada_path_weights, ada_start_pos, ada_end_pos)

    # length of the common segment
    d_p = get_distance(ref_path_weights, ref_first_merge_pos, ref_last_split_pos)

    # distance from the leaders start to the earliest merge point
    ref_d_s = get_distance(ref_path_weights, ref_start_pos, ref_first_merge_pos)
    ada_d_s = get_distance(ada_path_weights, ada_start_pos, ada_first_merge_pos)
    ref_d_sp = ref_path_L - ref_d_s - d_p
    ada_d_sp = ada_path_L - ada_d_s - d_p
    if abs(ada_d_sp) < constants.eps:
        ada_d_sp = 0.
    if abs(ref_d_sp) < constants.eps:
        ref_d_sp = 0.

    # they might have the same start position
    if ref_path[ref_start_pos['i']] == ada_path[ada_start_pos['i']] and abs(
                    ref_start_pos['x'] - ada_start_pos['x']) < constants.eps and ref_t_s == ada_t_s:
        ada_d_s_opt = 0.
        t_m_opt = ada_t_s  # merge time is now
        v_star_s = v_nom  # does not make the difference, just defined to not run into errors later
    else:  # they need to coordinate
        # //////// Merging //////////

        ref_t_m = ref_t_s + ref_d_s / ref_v_def  # arrival time at the earliest merge point of the leader
        ada_t_m_fast = ada_t_s + ada_d_s / v_star_fast
        ada_t_m_slow = ada_t_s + ada_d_s / v_star_slow

        if ada_t_m_fast > ref_t_m:  # cannot catch up at the earliest merge point with max speed
            v_star_s = v_star_fast
            dx = (ref_t_s - ada_t_s + ref_d_s / ref_v_def - ada_d_s / v_star_s) / (
                1. / v_star_s - 1. / ref_v_def)  # distance to travel past the merge point
        elif ada_t_m_slow < ref_t_m:  # cannot wait in at the earliest merge point with min speed
            v_star_s = v_star_slow
            dx = (ref_t_s - ada_t_s + ref_d_s / ref_v_def - ada_d_s / v_star_s) / (1. / v_star_s - 1. / ref_v_def)
        else:  # can merge at the earliest merge point
            v_star_s = ada_d_s / (ref_t_m - ada_t_s)
            dx = 0.

        ada_d_s_opt = ada_d_s + dx
        t_m_opt = ada_t_s + ada_d_s_opt / v_star_s  # merge time

        # if we merge after the common segment, there is no platooning
        if ada_d_s_opt > ada_d_s + d_p:
            if verbose:
                print 'Cannot platoon since the merge point would be past the common segment!'
            return -1

    # //////// Splitting //////////

    # check if before deadline arrival is possible with platooning
    # --> drive from merge point on with max speed

    total_dist_left = ada_path_L - ada_d_s_opt
    t_earl = total_dist_left / v_star_fast + t_m_opt
    if t_earl > ada_t_d:  # arrival before deadline not possible with platooning
        if verbose:
            print 'Cannot platoon since we cannot arrive before the deadline with platooning!'
        return -1

    platoon_dist_left = total_dist_left - ada_d_sp

    t_sp_last = t_m_opt + platoon_dist_left / ref_v_def  # arrival time at the split point of the routes of the leader
    # lastest time to pass the last split point
    t_sp_last_max = ada_t_d - ada_d_sp / v_star_fast
    if t_sp_last <= t_sp_last_max:  # can platoon until the end
        ada_d_sp_opt = ada_d_sp
        if ada_d_sp > constants.eps:
            v_d = ada_d_sp_opt / (ada_t_d - t_sp_last)  # speed to arrive at the deadline
            v_star_sp = max([v_min, v_d])  # go at least minimal speed
            t_a_opt = t_sp_last + ada_d_sp_opt / v_star_sp
        else:  # platoon until the end of the journey of ada
            t_a_opt = t_sp_last
            v_star_sp = v_nom  # does not make a difference
    elif t_sp_last >= t_sp_last_max:  # split early
        ada_d_sp_opt = (ada_t_d - t_m_opt - total_dist_left / ref_v_def) / (1. / v_star_fast - 1. / ref_v_def)
        v_star_sp = v_star_fast
        t_a_opt = ada_t_d
    t_sp_opt = t_a_opt - ada_d_sp_opt / v_star_sp  # optimal split time

    # calculate the fuel consumptions
    ada_d_p_opt = ada_path_L - ada_d_s_opt - ada_d_sp_opt
    f = (constants.F0 + constants.F1 * v_star_s) * ada_d_s_opt + (
                                                                 constants.F0 + constants.F1 * v_star_sp) * ada_d_sp_opt + (
                                                                                                                           constants.F0p + constants.F1p * ref_v_def) * ada_d_p_opt

    if verbose:
        t_a_test = ada_t_s + ada_d_s_opt / v_star_s + ada_d_sp_opt / v_star_sp + (
                                                                                 ada_path_L - ada_d_s_opt - ada_d_sp_opt) / ref_v_def
        print 'differenece between computed arrival time and test arrival time: {}\n (Should be zero!)'.format(
            t_a_test - t_a_opt)

    if f < ada_f_def:
        if verbose:
            print 'platooning is better'
        return f, ada_f_def - f, ada_d_s_opt, ada_d_sp_opt, t_m_opt, t_sp_opt, t_a_opt
    else:
        if verbose:
            print 'not platooning is better'
        return -1


def get_distance(path_weights, start_pos, end_pos):
    #  assumes a list with path weights, start_pos, end_pos as dict of link 'i' index
    #  and position on the link 'x'
    # returns zero the end lies before the start

    si = start_pos['i']
    sx = start_pos['x']
    ei = end_pos['i']
    ex = end_pos['x']

    if ei >= si:
        dist = np.sum(path_weights[si:ei]) - sx + ex
    elif ei < si:
        dist = 0.

    return dist


def build_G_p(path_data_sets, default_plans):
    K = len(path_data_sets)
    K_set = list(path_data_sets)  # truck indices set
    G_p = {key: {} for key in K_set}

    for i_f in xrange(K):
        for i_l in xrange(i_f + 1, K):
            kl = K_set[i_l]
            kf = K_set[i_f]
            intersection = find_route_intersection(path_data_sets[kl], path_data_sets[kf])
            #      old_intersection = find_route_intersection_old(path_data_sets[kl]['path'],path_data_sets[kf]['path'])
            if intersection:
                intersection_length = np.sum(
                    path_data_sets[kl]['path_weights'][intersection[0][0]:intersection[0][1] + 1])
                if intersection_length >= constants.min_intersection_length:
                    res = calculate_adaptation(path_data_sets[kl], path_data_sets[kf], intersection, default_plans[kl],
                                               default_plans[kf])
                    if res != -1:
                        f, df, ada_d_s_opt, ada_d_sp_opt, t_m_opt, t_sp_opt, t_a_opt = res
                        if df > 0.:
                            G_p[kf][kl] = df
                    # swap role
                    intersection = (intersection[1], intersection[0])
                    res = calculate_adaptation(path_data_sets[kf], path_data_sets[kl], intersection, default_plans[kf],
                                               default_plans[kl])
                    if res != -1:
                        f, df, ada_d_s_opt, ada_d_sp_opt, t_m_opt, t_sp_opt, t_a_opt = res
                        if df > 0.:
                            G_p[kl][kf] = df
    return G_p


def get_default_plans(path_data_sets):
    default_plans = {}
    for k in path_data_sets:
        t_a, v_default, f = calculate_default(path_data_sets[k])
        default_plans[k] = {'t_a': t_a, 'v_default': v_default, 'f': f, 'type': 'default'}

    return default_plans


def retrieve_adapted_plans(path_data_sets, leaders, default_plans):
    # calculates the selected adapted plans

    LEADER = -1
    NONE = -2

    plans = {}
    for k in leaders:
        if leaders[k] != LEADER and leaders[k] != NONE:
            kf = k
            kl = leaders[k]
            intersection = find_route_intersection(path_data_sets[kl], path_data_sets[kf])
            #      if intersection != False:
            f, df, ada_d_s_opt, ada_d_sp_opt, t_m_opt, t_sp_opt, t_a_opt = calculate_adaptation(path_data_sets[kl],
                                                                                                path_data_sets[kf],
                                                                                                intersection,
                                                                                                default_plans[kl],
                                                                                                default_plans[kf])
            plans[k] = {'f': f, 'df': df, 'd_s': ada_d_s_opt, 'd_sp': ada_d_sp_opt, 't_m': t_m_opt, 't_sp': t_sp_opt,
                        't_a': t_a_opt, 'type': 'adapted', 'leader': leaders[k]}
        else:
            plans[k] = default_plans[k]

    return plans


def total_fuel_consumption(plans):
    # returns the total fuel consumption of the plans submitted
    f_total = 0.
    for k, plan in plans.iteritems():
        f_total += plan['f']

    return f_total


def total_fuel_consumption_spontaneous_platooning(path_data_sets, default_plans, time_gap):
    # assumes for now that the trucks start at the beginning of the first link
    # TODO: There might be a problem with the nominal velocity.

    def calc_platoon_f(size, w):

        f = w * (constants.F0 + constants.F1 * v_nom)
        f += (size - 1) * w * (constants.F0p + constants.F1p * v_nom)

        return f

    edge_arrival_times = {}

    # collect arrival time at every edge in the paths
    for k in default_plans:
        v_nom = default_plans[k]['v_default']
        path = path_data_sets[k]['path']
        path_weights = path_data_sets[k]['path_weights']
        t_s = path_data_sets[k]['t_s']
        arrival_times = t_s + (np.cumsum(path_weights) - path_weights[0]) / v_nom
        for i in xrange(len(path)):
            edge = path[i]
            if edge in edge_arrival_times:
                edge_arrival_times[edge]['tt_link'].append(arrival_times[i])
            else:
                edge_arrival_times[edge] = {'weight': path_weights[i], 'tt_link': [arrival_times[i]]}

    total_f = 0.
    # go through G_e and calculate the fuel consumption per edge
    for edge, edge_data in edge_arrival_times.iteritems():
        tt_link = edge_data['tt_link']
        tt_link.sort()
        weight = edge_data['weight']

        cur_platoon = [tt_link[0]]
        for traversal_time in tt_link[1:]:
            if traversal_time - cur_platoon[0] < time_gap:
                cur_platoon.append(traversal_time)
            else:
                platoon_size = len(cur_platoon)
                total_f += calc_platoon_f(platoon_size, weight)
                cur_platoon = [traversal_time]
        platoon_size = len(cur_platoon)
        if platoon_size != 0:
            total_f += calc_platoon_f(platoon_size, weight)
        cur_platoon = []

    return total_f


def total_fuel_consumption_no_time_constraints(path_data_sets, default_plans, time_gap):
    # assumes for now that the trucks start at the beginning of the first link
    # calculates the fuel consumption if all trucks that share a link platoon

    def calc_platoon_f(size, w):

        f = w * (constants.F0 + constants.F1 * v_nom)
        f += (size - 1) * w * (constants.F0p + constants.F1p * v_nom)

        return f

    edge_platoon_sizes = {}

    # collect arrival time at every edge in the paths
    for k in default_plans:
        v_nom = default_plans[k]['v_default']
        path = path_data_sets[k]['path']
        path_weights = path_data_sets[k]['path_weights']
        for i in xrange(len(path)):
            edge = path[i]
            if edge in edge_platoon_sizes:
                edge_platoon_sizes[edge]['tt_link'] += 1.
            else:
                edge_platoon_sizes[edge] = {'weight': path_weights[i], 'tt_link': 1.}

    total_f = 0.
    # go through G_e and calculate the fuel consumption per edge
    for edge, edge_data in edge_platoon_sizes.iteritems():
        platoon_size = edge_data['tt_link']
        weight = edge_data['weight']
        total_f += calc_platoon_f(platoon_size, weight)

    return total_f
