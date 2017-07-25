# -*- coding: utf-8 -*-
"""
Created on Thu Jan 22 10:46:31 2015

@author: sebastian
"""

from constants import LEADER, NONE


class ClusterGraph:
    def __init__(self, K_set):
        self.nodes = {key: {} for key in K_set}
        self.inverted_nodes = {key: {} for key in K_set}

    def __getitem__(self, item):
        return self.nodes[item]

    def __setitem__(self, key, value):
        self.nodes[key] = value

    def __contains__(self, item):
        return item in self.nodes

    def add(self, follower, leader, plan):
        self.nodes[follower][leader] = plan
        self.inverted_nodes[leader][follower] = plan


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
