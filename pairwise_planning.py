import math
import numpy as np

from clusteralg import ClusterGraph
from constants import NONE, LEADER, V_MAX, V_MIN, V_NOM, F0, F0p, F1, F1p, EPS, MIN_INTERSECTION_LENGTH, TIME_GAP
# from clusteralg import ClusterGraph


class PlatoonPlan(object):
    def __init__(self, fuel):
        self.fuel = fuel

    def calculate_history(self, previous_t, current_t):
        raise NotImplementedError("This method is not implemented")

    def recalculate_fuel(self, current_t):
        pass


class AdaptedPlan(PlatoonPlan):
    # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time, merge speed, platoon speed, split speed
    def __init__(self, fuel, default_fuel, fuel_difference, merge_distance, split_distance, merge_time, split_time, arrival_time, merge_speed, platoon_speed, split_speed, leader, current_t):
        self.fuel = fuel
        self.default_fuel = default_fuel
        self.fuel_diff = fuel_difference
        self.merge_distance = merge_distance
        self.split_distance = split_distance
        self.merge_time = merge_time
        self.split_time = split_time
        self.arrival_time = arrival_time
        self.merge_speed = merge_speed
        self.platoon_speed = platoon_speed
        self.split_speed = split_speed
        self.type = None
        self.leader = leader
        self.speed = None
        self.merge_fuel_multiplier = (F0 + F1 * merge_speed) * merge_speed
        self.platoon_fuel_multiplier = (F0p + F1p * platoon_speed) * platoon_speed
        self.split_fuel_multiplier = (F0 + F1 * split_speed) * split_speed
        self.current_t = current_t

    def recalculate_fuel(self, current_t):
        total = 0
        if current_t < self.merge_time:
            total += self.merge_fuel_multiplier * (self.merge_time - current_t)
        if current_t < self.split_time:
            total += self.platoon_fuel_multiplier * (self.split_time - max(current_t, self.merge_time))
        total += self.split_fuel_multiplier * (self.arrival_time - max(current_t, self.split_time))
        self.fuel = total
        self.default_fuel = (F0 + F1 * V_NOM) * V_NOM * (self.arrival_time - current_t)
        self.fuel_diff = self.default_fuel - self.fuel
        return self.fuel_diff

    def calculate_history(self, previous_t, current_t):
        from platooning.assignments import SpeedChange
        result = []
        if previous_t < self.merge_time:
            result.append(SpeedChange(previous_t, self.merge_speed))
            if current_t > self.merge_time:
                result.append(SpeedChange(self.merge_time, self.platoon_speed, platooning=self.leader))
        elif previous_t < self.split_time:
            result.append(SpeedChange(previous_t, self.platoon_speed, platooning=self.leader))
        else:
            result.append(SpeedChange(previous_t, self.split_speed))

        if previous_t < self.split_time < current_t:
            result.append(SpeedChange(self.split_time, self.split_speed))

        for i,change in enumerate(result[:-1]):
            change.end_time = result[i+1].start_time
        result[-1].end_time = current_t

        return result

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        return self.leader == other.leader

    def __ne__(self, other):
        return not self.__eq__(other)


class DefaultPlan(PlatoonPlan):
    def __init__(self, arrival_time, fuel, speed):
        super(self.__class__, self).__init__(fuel)
        self.arrival_time = arrival_time
        self.speed = speed

    def calculate_history(self, previous_t, current_t):
        from platooning.assignments import SpeedChange
        return [SpeedChange(previous_t, self.speed, end_time=current_t)]

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        return abs(self.arrival_time - other.arrival_time) < 0.001 and abs(self.speed - other.speed) < 0.001

    def __ne__(self, other):
        return not self.__eq__(other)


INTERSECTION_CACHE = {}
PLAN_CACHE = {}


def find_first_index(known_intersection, path1, path2):
    # Given a known intersection, find the indexes where route1 and route2 first intersect.
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

def find_route_intersection(route1, route2):
    if (route1.id, route2.id) in INTERSECTION_CACHE:
        return INTERSECTION_CACHE[(route1.id, route2.id)]
    #  Gives first and last intersecting element on both routes
    path1 = route1.edge_path
    path2 = route2.edge_path
    intersections = route1.edge_set & route2.edge_set

    if not intersections:
        return False

    ind1_start, ind2_start = find_first_index(intersections.pop(), path1, path2)

    ind1_split = ind1_start + len(intersections)
    ind2_split = ind2_start + len(intersections)

    result = convert_edge_intersection(ind1_split, ind1_start, ind2_split, ind2_start, route1, route2)
    if not result:
        return False
    split1, split2, start1, start2 = result
    intersection = (start1, split1), (start2, split2)
    if route1.path[start1] != route2.path[start2] or route1.path[split1] != route2.path[split2]:
        print "Error: wrong intersection result"
    INTERSECTION_CACHE[(route1.id, route2.id)] = intersection
    return intersection


def convert_edge_intersection(ind1_split, ind1_start, ind2_split, ind2_start, route1, route2):
    start1 = route1.edge_offsets[ind1_start]
    start2 = route2.edge_offsets[ind2_start]
    if ind1_split + 1 >= len(route1.edge_offsets):
        split1 = len(route1.path) - 1
    else:
        split1 = route1.edge_offsets[ind1_split + 1]
    if ind2_split + 1 >= len(route2.edge_offsets):
        split2 = len(route2.path) - 1
    else:
        split2 = route2.edge_offsets[ind2_split + 1]

    # In case one ends on an edge, adjust so other intersection does not continue for entire edge
    if split1 - start1 != split2 - start2:
        diff = min(split1 - start1, split2 - start2)
        split1 = start1 + diff
        split2 = start2 + diff

    intersections = set(route1.path[start1:split1]) & set(route2.path[start2:split2])
    if not intersections:
        return False
    int1_start, int2_start = find_first_index(intersections.pop(), route1.path[start1:split1], route2.path[start2:split2])
    s1 = start1 + int1_start
    s2 = start2 + int2_start
    e1 = s1 + len(intersections)
    e2 = s2 + len(intersections)
    return e1, e2, s1, s2


def calculate_default(path_data):
    # returns the arrival time, default speed, and fuel consumption of the default plan

    path_weights = path_data['path_weights']
    start_pos = path_data['start_pos']  # index of the current link (!)
    t_s = path_data['t_s']
    t_d = path_data['arrival_dline']

    end_pos = {'i': len(path_weights) - 1, 'x': path_weights[-1]}
    path_L = get_distance(path_weights, start_pos, end_pos)

    v_d = path_L / (t_d - t_s)  # speed to arrive exactly at the deadline

    if v_d > V_MAX:
        print "Warning: a truck cannot make its deadline!"
    if v_d <= V_NOM:
        t_a = path_L / V_NOM + t_s
        v_default = V_NOM
    if v_d > V_NOM:
        t_a = path_L / v_d + t_s
        v_default = v_d

    f = (F0 + F1 * v_default) * path_L

    return DefaultPlan(t_a, f, v_default)


def calculate_adaptation(ref_path_data, ada_path_data, intersection, verbose=False):
    #  returns None if platooning is not feasible or beneficial, and the fuel saving (positive) if platooning is beneficial

    ref_current_t = ref_path_data.current_time
    ref_path = ref_path_data.path
    ref_path_weights = ref_path_data.path_weights
    ref_start_pos = ref_path_data.current_pos  # index of the current link (!)
    ref_t_s = ref_path_data.current_time
    ref_t_d = ref_path_data.deadline
    ref_merge_ind = intersection[0][0]
    ref_split_ind = intersection[0][1]
    ref_v_def = ref_path_data.default_plan.speed
    # if isinstance(ref_path_data.plan_history[-1], DefaultPlan):
    #     ref_v_def = ref_path_data.plan_history[-1].speed
    #  ref_f_def = ref_default_plan['f']
    #  ref_t_a_def = ref_default_plan['t_a']

    ada_current_t = ada_path_data.current_time
    ada_path = ada_path_data.path
    ada_path_weights = ada_path_data.path_weights
    ada_start_pos = ada_path_data.current_pos
    ada_t_s = ada_path_data.current_time
    ada_t_d = ada_path_data.deadline
    ada_merge_ind = intersection[1][0]
    ada_split_ind = intersection[1][1]
    #  ada_v_def = ada_default_plan['v_default']
    ada_f_def = ada_path_data.default_plan.fuel
    #  ada_t_a_def = ada_default_plan['t_a']

    # check if times overlap at all
    if ref_t_d < ada_t_s or ada_t_d < ref_t_s:
        if verbose:
            print 'Cannot platoon since the time intervals do not overlap!'
        return

    # Check if already at the end
    if ref_start_pos['i'] >= len(ref_path) or ada_start_pos['i'] >= len(ada_path):
        return

    Delta_F0 = F0 - F0p
    v_star_slow = max(
        [V_NOM * (1 - math.sqrt(1 - F1p / F1 + Delta_F0 / (F1 * V_NOM))), V_MIN])
    v_star_fast = min(
        [V_NOM * (1 + math.sqrt(1 - F1p / F1 + Delta_F0 / (F1 * V_NOM))), V_MAX])

    ref_end_pos = {'i': len(ref_path_weights) - 1, 'x': ref_path_weights[-1]}
    ref_first_merge_pos = {'i': ref_merge_ind, 'x': 0.}
    ref_last_split_pos = {'i': ref_split_ind, 'x': ref_path_weights[ref_split_ind]}

    ada_end_pos = {'i': len(ada_path_weights) - 1, 'x': ada_path_weights[-1]}
    ada_first_merge_pos = {'i': ada_merge_ind, 'x': 0.}
    ada_last_split_pos = {'i': ada_split_ind, 'x': ada_path_weights[ada_split_ind]}

    if ref_merge_ind <= ref_start_pos['i'] or ada_merge_ind <= ada_start_pos['i']:
        delta = max(ref_start_pos['i'] - ref_merge_ind, ada_start_pos['i'] - ada_merge_ind) + 1
        ref_first_merge_pos['i'] += delta
        ada_first_merge_pos['i'] += delta

    if ada_last_split_pos['i'] < ada_first_merge_pos['i'] or ref_last_split_pos['i'] < ref_first_merge_pos['i']:
        return

    if get_distance2(ref_path_data.path_weights_cum, ref_start_pos, ref_last_split_pos) == 0.:  # we have passed the common part
        if verbose:
            print 'Cannot platoon since the leader has passed the common part!'
        return
    if get_distance2(ada_path_data.path_weights_cum, ada_start_pos, ada_last_split_pos) == 0.:
        if verbose:
            print 'Cannot platoon since the follower has passed the common part!'
        return

    ref_path_L = get_distance2(ref_path_data.path_weights_cum, ref_start_pos, ref_end_pos)
    ada_path_L = get_distance2(ada_path_data.path_weights_cum, ada_start_pos, ada_end_pos)

    # length of the common segment
    d_p = get_distance2(ref_path_data.path_weights_cum, ref_first_merge_pos, ref_last_split_pos)

    # distance from the leaders start to the earliest merge point
    ref_d_s = get_distance2(ref_path_data.path_weights_cum, ref_start_pos, ref_first_merge_pos)
    ada_d_s = get_distance2(ada_path_data.path_weights_cum, ada_start_pos, ada_first_merge_pos)

    # Increase reference speed if needed to make it to the goal.
    ref_v_def = max(ref_v_def, ref_path_L / (ref_t_d - ref_t_s))
    ref_v_def = min(V_MAX, ref_v_def)
    # If leader is going at max speed and is closer to merge point than follower, the follower can never catch up
    if ref_v_def == V_MAX and ref_d_s < ada_d_s:
        return -1

    # Distance from split to end
    ref_d_sp = ref_path_L - ref_d_s - d_p
    ada_d_sp = ada_path_L - ada_d_s - d_p
    if abs(ada_d_sp) < EPS:
        ada_d_sp = 0.
    if abs(ref_d_sp) < EPS:
        ref_d_sp = 0.

    # they might have the same start position
    if ref_path[ref_start_pos['i']] == ada_path[ada_start_pos['i']] and abs(
                    ref_start_pos['x'] - ada_start_pos['x']) < EPS and ref_t_s == ada_t_s:
        ada_d_s_opt = 0.
        t_m_opt = ada_t_s  # merge time is now
        v_star_s = V_NOM  # does not make the difference, just defined to not run into errors later
    else:  # they need to coordinate
        # //////// Merging //////////

        ref_t_m = ref_t_s + ref_d_s / ref_v_def  # arrival time at the earliest merge point of the leader
        ada_t_m_fast = ada_t_s + ada_d_s / v_star_fast
        ada_t_m_slow = ada_t_s + ada_d_s / v_star_slow

        if ada_t_m_fast > ref_t_m:  # cannot catch up at the earliest merge point with max speed
            v_star_s = v_star_fast
            if v_star_s == ref_v_def:
                return -1
            dx = (ref_t_s - ada_t_s + ref_d_s / ref_v_def - ada_d_s / v_star_s) / (1. / v_star_s - 1. / ref_v_def)  # distance to travel past the merge point
        elif ada_t_m_slow < ref_t_m:  # cannot wait in at the earliest merge point with min speed
            v_star_s = v_star_slow
            if v_star_s == ref_v_def:
                return -1
            dx = (ref_t_s - ada_t_s + ref_d_s / ref_v_def - ada_d_s / v_star_s) / (1. / v_star_s - 1. / ref_v_def)
        else:  # can merge at the earliest merge point
            v_star_s = ada_d_s / (ref_t_m - ada_t_s)
            dx = 0.

        # distance between start and optimal merge point
        ada_d_s_opt = ada_d_s + dx
        t_m_opt = ada_t_s + ada_d_s_opt / v_star_s  # merge time

        # if we merge after the common segment, there is no platooning
        if ada_d_s_opt > ada_d_s + d_p:
            if verbose:
                print 'Cannot platoon since the merge point would be past the common segment!'
            return -1 # Return -1 instead of None so that we can catch this as a case where an adapted plan could exist in the future

    # //////// Splitting //////////

    # check if before deadline arrival is possible with platooning
    # --> drive from merge point on with max speed

    total_dist_left = ada_path_L - ada_d_s_opt
    t_earl = total_dist_left / v_star_fast + t_m_opt
    if t_earl > ada_t_d:  # arrival before deadline not possible with platooning
        if verbose:
            print 'Cannot platoon since we cannot arrive before the deadline with platooning!'
        return

    platoon_dist_left = total_dist_left - ada_d_sp

    t_sp_last = t_m_opt + platoon_dist_left / ref_v_def  # arrival time at the split point of the routes of the leader
    # latest time to pass the last split point
    t_sp_last_max = ada_t_d - ada_d_sp / v_star_fast
    if t_sp_last <= t_sp_last_max:  # can platoon until the end
        ada_d_sp_opt = ada_d_sp
        if ada_d_sp > EPS:
            v_d = ada_d_sp_opt / (ada_t_d - t_sp_last)  # speed to arrive at the deadline
            v_star_sp = max([V_MIN, v_d])  # go at least minimal speed
            t_a_opt = t_sp_last + ada_d_sp_opt / v_star_sp
        else:  # platoon until the end of the journey of ada
            t_a_opt = t_sp_last
            v_star_sp = V_NOM  # does not make a difference
    elif t_sp_last >= t_sp_last_max:  # split early
        ada_d_sp_opt = (ada_t_d - t_m_opt - total_dist_left / ref_v_def) / (1. / v_star_fast - 1. / ref_v_def)
        v_star_sp = v_star_fast
        t_a_opt = ada_t_d

    # TODO: Don't do this. Instead figure out how to allow for non-nominal times
    # v_star_sp = max(v_star_sp, V_NOM)
    t_sp_opt = t_a_opt - ada_d_sp_opt / v_star_sp  # optimal split time

    # v_star_s = Speed while going to merge point
    # v_star_sp = Speed after splitting
    # ada_d_s_opt = Distance to optimal merge point
    # ada_d_sp_opt = Distance left after splitting from platoon
    # ada_d_p_opt = Optimal distance in platoon
    # calculate the fuel consumptions
    ada_d_p_opt = ada_path_L - ada_d_s_opt - ada_d_sp_opt
    f = (F0 + F1 * v_star_s) * ada_d_s_opt + (F0 + F1 * v_star_sp) * ada_d_sp_opt + (F0p + F1p * ref_v_def) * ada_d_p_opt

    if verbose:
        t_a_test = ada_t_s + ada_d_s_opt / v_star_s + ada_d_sp_opt / v_star_sp + (ada_path_L - ada_d_s_opt - ada_d_sp_opt) / ref_v_def
        print 'difference between computed arrival time and test arrival time: {}\n (Should be zero!)'.format(
            t_a_test - t_a_opt)

    ada_f_def = (F0 + F1 * ref_v_def) * ada_path_L

    if verbose:
        if f < ada_f_def:
            print 'platooning is better'
        else:
            print 'not platooning is better'

    # Fuel consumption, default fuel, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
    return AdaptedPlan(f, ada_f_def, ada_f_def - f, ada_d_s_opt, ada_d_sp_opt, t_m_opt, t_sp_opt, t_a_opt, v_star_s, ref_v_def, v_star_sp, ref_path_data.id, ada_t_s)


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


def get_distance2(path_weights_cum, start_pos, end_pos):
    #  assumes a list with path weights, start_pos, end_pos as dict of link 'i' index
    #  and position on the link 'x'
    # returns zero the end lies before the start

    si = start_pos['i']
    sx = start_pos['x']
    ei = end_pos['i']
    ex = end_pos['x']

    if ei >= si:
        dist = path_weights_cum[ei] - path_weights_cum[si] - sx + ex
    elif ei < si:
        dist = 0.

    return dist


def build_graph(assignments, previous_graph=None):
    K = len(assignments)
    K_set = [x for x in assignments]  # truck indices set
    graph = ClusterGraph(K_set)

    # graph = None
    if K < 2: return graph

    for i_f in xrange(K):
        for i_l in xrange(i_f + 1, K):
            kl = K_set[i_l]
            kf = K_set[i_f]
            # if previous_graph:
            #     if kl in previous_graph and kf in previous_graph and (kl not in previous_graph[kf].keys() and kf not in previous_graph[kl].keys()):
            #         continue
            if (kl, kf) in PLAN_CACHE:
                graph.add(kl, kf, PLAN_CACHE[(kl, kf)])
            if (kf, kl) in PLAN_CACHE:
                graph.add(kf, kl, PLAN_CACHE[(kf, kl)])
            intersection = find_route_intersection(assignments[kl], assignments[kf])
            #      old_intersection = find_route_intersection_old(path_data_sets[kl]['path'],path_data_sets[kf]['path'])
            if intersection:

                intersection_length = np.sum(
                    assignments[kl].path_weights[intersection[0][0]:intersection[0][1] + 1])
                if intersection_length >= MIN_INTERSECTION_LENGTH:
                    plan = calculate_adaptation(assignments[kl], assignments[kf], intersection)
                    if plan:
                        if plan != -1 and plan.fuel_diff > 0.:
                            PLAN_CACHE[(kf, kl)] = plan
                            graph.add(kf, kl, plan)
                    # swap role
                    intersection = (intersection[1], intersection[0])
                    plan = calculate_adaptation(assignments[kf], assignments[kl], intersection)
                    if plan:
                        if plan != -1 and plan.fuel_diff > 0.:
                            PLAN_CACHE[(kl, kf)] = plan
                            graph.add(kl, kf, plan)
    return graph


def get_default_plans(path_data_sets):
    return {k: calculate_default(path_data_sets[k]) for k in path_data_sets}


def retrieve_adapted_plans(assignments, leaders, G):
    # calculates the selected adapted plans
    plans = {}
    for k in leaders:
        if leaders[k] == LEADER or leaders[k] == NONE:
            plans[k] = assignments[k].default_plan
        else: # follower
            kf = k
            kl = leaders[k]
            plan = G[kf][kl]
            plan.type = 'adapted'
            plan.leader = leaders[k]
            plans[k] = plan

    return plans


def total_fuel_consumption(plans):
    # returns the total fuel consumption of the plans submitted
    f_total = 0.
    for k, plan in plans.iteritems():
        f_total += plan.fuel

    return f_total


def total_fuel_consumption_spontaneous_platooning(assignments):
    # assumes for now that the trucks start at the beginning of the first link
    # TODO: There might be a problem with the nominal velocity.

    def calc_platoon_f(size, w):

        f = w * (F0 + F1 * v_nom)
        f += (size - 1) * w * (F0p + F1p * v_nom)

        return f

    edge_arrival_times = {}

    # collect arrival time at every edge in the paths
    for assignment in assignments:
        v_nom = assignment.default_plan.speed
        path = assignment.path
        path_weights = assignment.path_weights
        t_s = assignment.start_time
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
            if traversal_time - cur_platoon[0] < TIME_GAP:
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


def total_fuel_consumption_no_time_constraints(assignments):
    # assumes for now that the trucks start at the beginning of the first link
    # calculates the fuel consumption if all trucks that share a link platoon

    def calc_platoon_f(size, w):

        f = w * (F0 + F1 * v_nom)
        f += (size - 1) * w * (F0p + F1p * v_nom)

        return f

    edge_platoon_sizes = {}

    # collect arrival time at every edge in the paths
    for assignment in assignments:
        v_nom = assignment.default_plan.speed
        for i in xrange(len(assignment.path)):
            edge = assignment.path[i]
            if edge in edge_platoon_sizes:
                edge_platoon_sizes[edge]['tt_link'] += 1.
            else:
                edge_platoon_sizes[edge] = {'weight': assignment.path_weights[i], 'tt_link': 1.}

    total_f = 0.
    # go through G_e and calculate the fuel consumption per edge
    for edge, edge_data in edge_platoon_sizes.iteritems():
        platoon_size = edge_data['tt_link']
        weight = edge_data['weight']
        total_f += calc_platoon_f(platoon_size, weight)

    return total_f
