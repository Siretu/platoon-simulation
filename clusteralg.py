# -*- coding: utf-8 -*-
"""
Created on Thu Jan 22 10:46:31 2015

@author: sebastian
"""

import heapq
import random

total_miss = 0
total_update = 0


def clustering(G, node_selection_method='greedy', verbose=False, max_iter=1000000):
    # node selection method: greedy, random

    # Init

    LEADER = -1
    NONE = -2

    G_inv = build_inverted_graph(G)
    nodes = list(G)
    leaders = {node: NONE for node in nodes}
    gains = {node: 0 for node in nodes}  # current gain from platooning
    counter = 0
    # caution: the list use negative gains for since it is a min heap
    # and we want to change the node with biggest gain
    gains = {n: [-get_delta_u(n, leaders, G, G_inv), n] for n in nodes}
    if node_selection_method == 'greedy':
        gain_heap = []
        for key in gains:  # add all nodes with positive gain to the heap
            if gains[key][0] < 0.:
                heapq.heappush(gain_heap, gains[key])

    # loop

    while counter < max_iter:
        counter += 1  # iteration count

        # select a node and calculate its delta_u
        if node_selection_method == 'greedy':
            node = select_node_greedy(nodes, gains, gain_heap)
        elif node_selection_method == 'random':
            node = select_node_random(nodes, gains)
        else:
            print 'invalid node selection method'
            return
        if node == -1:
            break

        # update the role of the node
        if leaders[node] == LEADER:  # become a follower
            change_to_follower(node, leaders, G, G_inv, verbose)
        else:  # become a leader
            change_to_leader(node, leaders, G, G_inv, verbose)
        # update the gains
        if node_selection_method == 'greedy':
            update_u_greedy(node, leaders, gains, gain_heap, G, G_inv)
        elif node_selection_method == 'random':
            update_u_random(node, leaders, gains, G, G_inv)

    # clean-up

    # check if all leaders have followers
    for n in G:
        if leaders[n] == LEADER:
            has_followers = False
            for pot_follower in G_inv[n]:
                if leaders[pot_follower] == n:
                    has_followers = True
                    break
            if not has_followers:
                leaders[n] = NONE
                # convert to sets
    N_l = set([nodel for nodel in nodes if leaders[nodel] == LEADER])
    N_f = set([nodel for nodel in nodes if leaders[nodel] != LEADER])

    return N_f, N_l, leaders, counter


def build_inverted_graph(G):
    # calculates G with reversed edges
    G_inv = {node: {} for node in G}
    for node1 in G:
        for node2 in G[node1]:
            G_inv[node2][node1] = G[node1][node2]
    return G_inv


def select_node_greedy(nodes, gains, gain_heap):
    # returns -1 if the loop should be broken
    if len(gain_heap) == 0:
        node = -1
    else:
        while 1:
            (gain, node) = heapq.heappop(gain_heap)
            if node != -1 or len(gain_heap) == 0:
                break

    return node


def select_node_random(nodes, gains):
    nodes_with_pos_gain = [n for n in nodes if gains[n][0] < 0.]
    if len(nodes_with_pos_gain) == 0:
        node = -1
    else:
        random.seed(0)
        node = random.choice(nodes_with_pos_gain)

    return node


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


def update_u_greedy(n, leaders, gains, gain_heap, G, G_inv):
    global total_miss
    global total_update

    # updates gains and gain_heap according to leader for all two-hop of
    # neighbors of n

    # it would be possible to integrate this with change_to_leader/change_to_follower
    # for improved performance

    # calculate and iterate of two hop neighbors
    neighbors = get_two_hop_neighbors(n, G, G_inv)
    for nb in neighbors:
        gain = get_delta_u(nb, leaders, G, G_inv)

        # update heap
        if gains[nb][0] != -gain:  # gain changed
            total_update += 1
            gains[nb][1] = -1  # old entry is invalid
            gains[nb] = [-gain, nb]  # new entry
            if gain > 0.:  # push on the heap if the gain is positive
                heapq.heappush(gain_heap, gains[nb])
        else:
            total_miss += 1
    return


def update_u_random(n, leaders, gains, G, G_inv):
    # updates gains according to leader for all two-hop of
    # neighbors of n

    # it would be possible to integrate this with change_to_leader/change_to_follower
    # for improved performance

    # calculate and iterate of two hop neighbors
    neighbors = get_two_hop_neighbors(n, G, G_inv)
    for nb in neighbors:
        gain = get_delta_u(nb, leaders, G, G_inv)
        gains[nb] = [-gain, nb]  # new entry

    return


def get_two_hop_neighbors(n, G, G_inv):
    # return a set of the two hop neighbors of n

    neighbors = set(G[n].keys() + G_inv[n].keys())
    for nb in G[n].keys() + G_inv[n].keys():
        neighbors.update(G[nb].keys() + G_inv[nb].keys())
    neighbors.add(n)

    return neighbors


def get_upper_bound(G):
    upper_bound = 0.
    for kf in G:
        best_val = 0.
        for kl in G[kf]:
            if G[kf][kl] > best_val:
                best_val = G[kf][kl]
        upper_bound += best_val

    return upper_bound
