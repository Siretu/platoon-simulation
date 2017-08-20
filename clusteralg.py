# -*- coding: utf-8 -*-
"""
Created on Thu Jan 22 10:46:31 2015

@author: sebastian
"""

from constants import LEADER, NONE, MIN_INTERSECTION_LENGTH
import numpy as np


class ClusterGraph:
    def __init__(self, K_set):
        self.nodes = {key: {} for key in K_set}
        self.inverted_nodes = {key: {} for key in K_set}
        self.potential_edges = {}

    def __getitem__(self, item):
        return self.nodes[item]

    def __setitem__(self, key, value):
        self.nodes[key] = value

    def __contains__(self, item):
        return item in self.nodes

    def add(self, follower, leader, plan):
        if follower not in self.nodes:
            self.nodes[follower] = {}
        if leader not in self.inverted_nodes:
            self.inverted_nodes[leader] = {}
        self.nodes[follower][leader] = plan
        self.inverted_nodes[leader][follower] = plan

    def update(self, new_trucks, current_trucks, time):
        from pairwise_planning import find_route_intersection, calculate_adaptation
        # self.clear_old(current_trucks)

        for follower in self.nodes:
            for leader in self.nodes[follower]:
                self.nodes[follower][leader].recalculate_fuel(time)

        for truck in new_trucks:
            self.nodes[truck.id] = {}
            self.inverted_nodes[truck.id] = {}
            self.potential_edges[truck.id] = {}
            for old_truck_id in self.nodes:
                if old_truck_id not in current_trucks.keys() or old_truck_id == truck.id:
                    continue
                old_truck = current_trucks[old_truck_id]
                intersection = find_route_intersection(truck, old_truck)
                if intersection:
                    intersection_length = np.sum(truck.path_weights[intersection[0][0]:intersection[0][1] + 1])
                    if intersection_length >= MIN_INTERSECTION_LENGTH:
                        plan = calculate_adaptation(truck, old_truck, intersection)
                        if plan:
                            if plan == -1 or plan.fuel_diff < 0.:
                                self.potential_edges[old_truck_id][truck.id] = intersection
                            else:
                                self.add(old_truck.id, truck.id, plan)
                        # swap role
                        intersection = (intersection[1], intersection[0])
                        plan = calculate_adaptation(old_truck, truck, intersection)
                        if plan:
                            if plan == -1 or plan.fuel_diff < 0.:
                                self.potential_edges[truck.id][old_truck_id] = intersection
                            else:
                                self.add(truck.id, old_truck.id, plan)

        for kf in current_trucks:
            for kl in self.potential_edges[kf].keys()[:]:
                if kl not in current_trucks:
                    del(self.potential_edges[kf][kl])
                    continue
                intersection = self.potential_edges[kf][kl]
                plan = calculate_adaptation(current_trucks[kl], current_trucks[kf], intersection)
                if plan:
                    if plan != -1 and plan.fuel_diff > 0.:
                        self.add(kf, kl, plan)
                        del(self.potential_edges[kf][kl])

    def clear_old(self, current_trucks):
        self.nodes = {x: self.nodes[x] for x in self.nodes if x in current_trucks}
        self.inverted_nodes = {x: self.inverted_nodes[x] for x in self.inverted_nodes if x in current_trucks}
        for follower in self.nodes:
            self.nodes[follower] = {x: self.nodes[follower][x] for x in self.nodes[follower] if x in current_trucks}


def change_to_follower(node, leaders, G, verbose):
    # updates leaders in place so that node becomes a follower and its previous
    # followers are updated accordingly

    best_l = NONE
    gain_l = 0.
    for neighbor in G[node]:
        if leaders[neighbor] == LEADER and G[node][neighbor].fuel_diff > gain_l:
            best_l = neighbor
            gain_l = G[node][neighbor].fuel_diff
    if verbose:
        print 'node ' + str(node) + ' becomes a follower of node ' + str(best_l)
    leaders[node] = best_l
    for neighbor in G.inverted_nodes[node]:
        if leaders[neighbor] == node:
            gain_ln = 0.
            best_l = NONE
            for nneighbor in G[neighbor]:
                if leaders[nneighbor] == LEADER and G[neighbor][nneighbor].fuel_diff > gain_ln:
                    gain_ln = G[neighbor][nneighbor].fuel_diff
                    best_l = nneighbor
            leaders[neighbor] = best_l

    return


def change_to_leader(node, leaders, G, verbose):
    # updates leaders in place so that node becomes a leader and its neighbors
    # that become its followers are upadted accordingly
    if verbose:
        print 'node ' + str(node) + ' becomes a leader'
    leaders[node] = LEADER
    for neighbor in G.inverted_nodes[node]:
        if leaders[neighbor] != LEADER and leaders[neighbor] != NONE:
            if G[neighbor][node].fuel_diff > G[neighbor][leaders[neighbor]].fuel_diff:
                leaders[neighbor] = node
        elif leaders[neighbor] == NONE:
            leaders[neighbor] = node

    return


def get_delta_u(node, leaders, G):
    delta_u = 0.
    if leaders[node] == LEADER:  # a leader
        # find best leader for this node
        best_l = NONE
        gain_l = 0.
        for neighbor in G[node]:
            if leaders[neighbor] == LEADER and G[node][neighbor].fuel_diff > gain_l:
                best_l = neighbor
                gain_l = G[node][neighbor].fuel_diff
        if best_l != NONE:
            delta_u = gain_l
        # see what happens to the followers
        for neighbor in G.inverted_nodes[node]:
            if leaders[neighbor] == node:
                gain_ln = 0.
                # Check if there's a next_neighbor that neighbor can follow instead
                for nneighbor in G[neighbor]:
                    if nneighbor != node and leaders[nneighbor] == LEADER and G[neighbor][nneighbor].fuel_diff > gain_ln:
                        gain_ln = G[neighbor][nneighbor].fuel_diff
                delta_u += gain_ln - G[neighbor][node].fuel_diff
    else: # a follower
        if leaders[node] != NONE:
            delta_u -= G[node][leaders[node]].fuel_diff
        for neighbor in G.inverted_nodes[node]:
            if leaders[neighbor] != LEADER and leaders[neighbor] != NONE:
                if G[neighbor][node].fuel_diff > G[neighbor][leaders[neighbor]].fuel_diff:
                    delta_u += G[neighbor][node].fuel_diff - G[neighbor][leaders[neighbor]].fuel_diff
            elif leaders[neighbor] == NONE:
                delta_u += G[neighbor][node].fuel_diff

    return delta_u


def get_two_hop_neighbors(n, G):
    # return a set of the two hop neighbors of n

    neighbors = set(G[n].keys() + G.inverted_nodes[n].keys())
    for nb in G[n].keys() + G.inverted_nodes[n].keys():
        neighbors.update(G[nb].keys() + G.inverted_nodes[nb].keys())
    neighbors.add(n)

    return neighbors


def get_upper_bound(G):
    return sum([max([y.fuel_diff for y in G[x].values()]) for x in G.nodes if len(G[x])])
