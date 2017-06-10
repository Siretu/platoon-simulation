import heapq
import random

from clusteralg import get_two_hop_neighbors, delta_u, build_inverted_graph, change_to_follower, change_to_leader

total_miss = 0
total_update = 0


class PlatooningMethod():
    def select_node(self, nodes, gains, gain_heap):
        raise NotImplementedError("This method is not implemented")

    def update_u(self, n, leaders, gains, gain_heap, G, G_inv):
        raise NotImplementedError("This method is not implemented")

    def clustering(self, G, verbose=False, max_iter=1000000):
        raise NotImplementedError("This method is not implemented")


class GreedyPlatooning(PlatooningMethod):
    def __str__(self):
        return "greedy"

    def select_node(self, nodes, gains, gain_heap):
        # returns -1 if the loop should be broken
        if len(gain_heap) == 0:
            node = -1
        else:
            while 1:
                (gain, node) = heapq.heappop(gain_heap)
                if node != -1 or len(gain_heap) == 0: break

        return node

    def update_u(self, n, leaders, gains, gain_heap, G, G_inv):
        global total_miss
        global total_update

        # updates gains and gain_heap according to leader for all two-hop of
        # neighbors of n

        # it would be possible to integrate this with change_to_leader/change_to_follower
        # for improved performance

        # calculate and iterate of two hop neighbors
        neighbors = get_two_hop_neighbors(n, G, G_inv)
        for nb in neighbors:
            gain = delta_u(nb, leaders, G, G_inv)

            # update heap
            if gains[nb][0] != -gain:  # gain changed
                total_update += 1
                gains[nb][1] = -1  # old entry is invalid
                gains[nb] = [-gain, nb]  # new entry
                if gain > 0.:  # push on the heap if the gain is positive
                    heapq.heappush(gain_heap, gains[nb])
            else:
                total_miss += 1

    def clustering(self, G, verbose=False, max_iter=1000000):
        ######## init #########

        LEADER = -1
        NONE = -2

        G_inv = build_inverted_graph(G)
        nodes = list(G)
        leaders = {node: NONE for node in nodes}
        gains = {node: 0 for node in nodes}  # current gain from platooning
        counter = 0
        # caution: the list use negative gains for since it is a min heap
        # and we want to change the node with biggest gain
        gains = {n: [-delta_u(n, leaders, G, G_inv), n] for n in nodes}
        gain_heap = []
        for key in gains:  # add all nodes with positive gain to the heap
            if gains[key][0] < 0.: heapq.heappush(gain_heap, gains[key])

        ######## loop #########

        while counter < max_iter:
            counter += 1  # iteration count

            # select a node and calculate its delta_u
            node = self.select_node(nodes, gains, gain_heap)
            if node == -1: break

            # update the role of the node
            if leaders[node] == LEADER:  # become a follower
                change_to_follower(node, leaders, G, G_inv, verbose)
            else:  # become a leader
                change_to_leader(node, leaders, G, G_inv, verbose)
            # update the gains
            self.update_u(node, leaders, gains, gain_heap, G, G_inv)

        ######### clean-up #########

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


class RandomPlatooning(PlatooningMethod):
    def __init__(self, seed):
        self.seed = seed

    def __str__(self):
        return "random"

    def select_node(self, nodes, gains, gain_heap):
        localRandom = random.Random(self.seed)
        nodes_with_pos_gain = [n for n in nodes if gains[n][0] < 0.]
        if len(nodes_with_pos_gain) == 0:
            node = -1
        else:
            node = localRandom.choice(nodes_with_pos_gain)

        return node

    def update_u(self, n, leaders, gains, gain_heap, G, G_inv):
        # updates gains according to leader for all two-hop of
        # neighbors of n

        # it would be possible to integrate this with change_to_leader/change_to_follower
        # for improved performance

        # calculate and iterate of two hop neighbors
        neighbors = get_two_hop_neighbors(n, G, G_inv)
        for nb in neighbors:
            gain = delta_u(nb, leaders, G, G_inv)
            gains[nb] = [-gain, nb]  # new entry

    def clustering(self, G, verbose=False, max_iter=1000000):
        # node selection method: greedy, random

        ######## init #########

        LEADER = -1
        NONE = -2

        G_inv = build_inverted_graph(G)
        nodes = list(G)
        leaders = {node: NONE for node in nodes}
        gains = {node: 0 for node in nodes}  # current gain from platooning
        counter = 0
        # caution: the list use negative gains for since it is a min heap
        # and we want to change the node with biggest gain
        gains = {n: [-delta_u(n, leaders, G, G_inv), n] for n in nodes}

        ######## loop #########

        while counter < max_iter:
            counter += 1  # iteration count

            # select a node and calculate its delta_u
            node = self.select_node(nodes, gains, None)
            if node == -1: break

            # update the role of the node
            if leaders[node] == LEADER:  # become a follower
                change_to_follower(node, leaders, G, G_inv, verbose)
            else:  # become a leader
                change_to_leader(node, leaders, G, G_inv, verbose)
            # update the gains
            self.update_u(node, leaders, gains, None, G, G_inv)

        ######### clean-up #########

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


class SubModularityPlatooning(PlatooningMethod):
    def __str__(self):
        return "sub modularity"

    def clustering(self, G, verbose=False, max_iter=1000000):
        X = [set()]
        Y = [set(G.keys())]

        for i in range(len(Y[0])):
            Xp = X[i].union([i])
            Yp = Y[i].difference([i])
            a = self.f(Xp, G) - self.f(X[i], G)
            b = self.f(Yp, G) - self.f(Y[i], G)
            if a >= b:
                X.append(Xp)
                Y.append(Y[i])
            else:
                X.append(X[i])
                Y.append(Yp)

        leaders = X[-1]
        followers = set(G.keys()).difference(leaders)
        nodes = {}
        real_leaders = set()
        real_followers = set()
        for x in G:
            if x in leaders:
                nodes[x] = -1
            else:
                max_leader = -2
                for neighbor in G[x]:
                    if neighbor in leaders and (max_leader == -2 or G[x][neighbor] > G[x][max_leader]):
                        max_leader = neighbor
                nodes[x] = max_leader
                if max_leader != -2:
                    real_leaders.add(max_leader)
                    real_followers.add(x)

        return real_followers, real_leaders, nodes, 1

    def f(self, leaders, G):
        G_inv = build_inverted_graph(G)

        followers = set(G.keys()).difference(leaders)
        total = 0
        for follower in followers:
            max_gain = -100000
            for neighbor in G_inv[follower]:
                if neighbor in leaders and G_inv[follower][neighbor] > max_gain:
                    max_gain = G_inv[follower][neighbor]
            if max_gain > -10000:
                total += max_gain
        return total
