# -*- coding: utf-8 -*-
"""
Created on Thu Jan 22 10:46:31 2015

@author: sebastian
"""

import heapq
import random

total_miss = 0
total_update = 0

def build_inverted_graph(G):
    # calculates G with reversed edges
    G_inv = {node: {} for node in G}
    for node1 in G:
        for node2 in G[node1]:
            G_inv[node2][node1] = G[node1][node2]
    return G_inv


def change_to_follower(node, leaders, G, G_inv, verbose):
    # updates leaders in place so that node becomes a follower and its previous
    # followers are updated accordingly

    LEADER = -1
    NONE = -2

    best_l = NONE
    gain_l = 0.
    for neighbor in G[node]:
        if leaders[neighbor] == LEADER and G[node][neighbor] > gain_l:
            best_l = neighbor
            gain_l = G[node][neighbor]
    if verbose:
        print 'node ' + str(node) + ' becomes a follower of node ' + str(best_l)
    leaders[node] = best_l
    for neighbor in G_inv[node]:
        if leaders[neighbor] == node:
            gain_ln = 0.
            best_l = NONE
            for nneighbor in G[neighbor]:
                if leaders[nneighbor] == LEADER and G[neighbor][nneighbor] > gain_ln:
                    gain_ln = G[neighbor][nneighbor]
                    best_l = nneighbor
            leaders[neighbor] = best_l

    return


def change_to_leader(node, leaders, G, G_inv, verbose):
    # updates leaders in place so that node becomes a leader and its neighbors
    # that become its followers are upadted accordingly

    LEADER = -1
    NONE = -2

    if verbose:
        print 'node ' + str(node) + ' becomes a leader'
    leaders[node] = LEADER
    for neighbor in G_inv[node]:
        if leaders[neighbor] != LEADER and leaders[neighbor] != NONE:
            if G[neighbor][node] > G[neighbor][leaders[neighbor]]:
                leaders[neighbor] = node
        elif leaders[neighbor] == NONE:
            leaders[neighbor] = node

    return


def get_delta_u(node, leaders, G, G_inv):
    LEADER = -1
    NONE = -2

    delta_u = 0.
    if leaders[node] != LEADER:  # a follower
        if leaders[node] != NONE:
            delta_u -= G[node][leaders[node]]
        for neighbor in G_inv[node]:
            if leaders[neighbor] != LEADER and leaders[neighbor] != NONE:
                if G[neighbor][node] > G[neighbor][leaders[neighbor]]:
                    delta_u += G[neighbor][node] - G[neighbor][leaders[neighbor]]
            elif leaders[neighbor] == NONE:
                delta_u += G[neighbor][node]
    else:  # a leader
        delta_u = 0.
        # find best leader for this node
        best_l = NONE
        gain_l = 0.
        for neighbor in G[node]:
            if leaders[neighbor] == LEADER and G[node][neighbor] > gain_l:
                best_l = neighbor
                gain_l = G[node][neighbor]
        if best_l != NONE:
            delta_u = gain_l
        # see what happens to the followers
        for neighbor in G_inv[node]:
            if leaders[neighbor] == node:
                gain_ln = 0.
                for nneighbor in G[neighbor]:
                    if nneighbor != node and leaders[nneighbor] == LEADER and G[neighbor][nneighbor] > gain_ln:
                        gain_ln = G[neighbor][nneighbor]
                delta_u += gain_ln - G[neighbor][node]

    return delta_u


def get_two_hop_neighbors(n, G, G_inv):
    # return a set of the two hop neighbors of n

    neighbors = set(G[n].keys() + G_inv[n].keys())
    for nb in G[n].keys() + G_inv[n].keys():
        neighbors.update(G[nb].keys() + G_inv[nb].keys())
    neighbors.add(n)

    return neighbors


def get_upper_bound(G):
    return sum([max(G[x].values()) for x in G if len(G[x])])
