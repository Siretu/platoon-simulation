import heapq
import random

from constants import NONE, LEADER
from clusteralg import get_two_hop_neighbors, get_delta_u, change_to_follower, change_to_leader


class PlatooningMethod:
    def __init__(self):
        pass

    def clustering(self, G, verbose=False):
        raise NotImplementedError("This method is not implemented")


class GreedyPlatooning(PlatooningMethod):
    def __init__(self):
        pass

    def __str__(self):
        return "greedy"

    def select_node(self, nodes, gains, gain_heap):
        # returns -1 if the loop should be broken
        if len(gain_heap) == 0:
            node = -1
        else:
            while 1:
                (gain, node) = heapq.heappop(gain_heap)
                if node != -1 or len(gain_heap) == 0:
                    break

        return node

    def update_u(self, n, leaders, gains, gain_heap, G):
        # updates gains and gain_heap according to leader for all two-hop of
        # neighbors of n

        # it would be possible to integrate this with change_to_leader/change_to_follower
        # for improved performance

        # calculate and iterate of two hop neighbors
        neighbors = get_two_hop_neighbors(n, G)
        for nb in neighbors:
            gain = get_delta_u(nb, leaders, G)

            # update heap
            if gains[nb][0] != -gain:  # gain changed
                gains[nb][1] = -1  # old entry is invalid
                gains[nb] = [-gain, nb]  # new entry
                if gain > 0.:  # push on the heap if the gain is positive
                    heapq.heappush(gain_heap, gains[nb])

    def clustering(self, G, verbose=False, max_iter=1000000):
        # init

        nodes = list(G.nodes)
        leaders = {node: NONE for node in nodes}
        # caution: the list use negative gains for since it is a min heap
        # and we want to change the node with biggest gain
        gains = {n: [-get_delta_u(n, leaders, G), n] for n in nodes}
        gain_heap = []
        for key in gains:  # add all nodes with positive gain to the heap
            if gains[key][0] < 0.:
                heapq.heappush(gain_heap, gains[key])

        # loop
        counter = 0
        while counter < max_iter:
            counter += 1  # iteration count

            # select a node and calculate its delta_u
            node = self.select_node(nodes, gains, gain_heap)
            if node == -1:
                break

            # update the role of the node
            if leaders[node] == LEADER:  # become a follower
                change_to_follower(node, leaders, G, verbose)
            else:  # become a leader
                change_to_leader(node, leaders, G, verbose)
            # update the gains
            self.update_u(node, leaders, gains, gain_heap, G)

        if counter == max_iter:
            print "Max"

        # clean-up

        # check if all leaders have followers
        for n in G.nodes:
            if leaders[n] == LEADER:
                has_followers = False
                for pot_follower in G.inverted_nodes[n]:
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

    def update_u(self, n, leaders, gains, gain_heap, G):
        # updates gains according to leader for all two-hop of
        # neighbors of n

        # it would be possible to integrate this with change_to_leader/change_to_follower
        # for improved performance

        # calculate and iterate of two hop neighbors
        neighbors = get_two_hop_neighbors(n, G)
        for nb in neighbors:
            gain = get_delta_u(nb, leaders, G)
            gains[nb] = [-gain, nb]  # new entry

    def clustering(self, G, verbose=False, max_iter=1000000):
        # node selection method: greedy, random

        # init

        nodes = list(G.nodes)
        leaders = {node: NONE for node in nodes}
        gains = {node: 0 for node in nodes}  # current gain from platooning
        counter = 0
        # caution: the list use negative gains for since it is a min heap
        # and we want to change the node with biggest gain
        gains = {n: [-get_delta_u(n, leaders, G), n] for n in nodes}

        # loop

        while counter < max_iter:
            counter += 1  # iteration count

            # select a node and calculate its delta_u
            node = self.select_node(nodes, gains, None)
            if node == -1:
                break

            # update the role of the node
            if leaders[node] == LEADER:  # become a follower
                change_to_follower(node, leaders, G, verbose)
            else:  # become a leader
                change_to_leader(node, leaders, G, verbose)
            # update the gains
            self.update_u(node, leaders, gains, None, G)

        # clean-up

        # check if all leaders have followers
        for n in G.nodes:
            if leaders[n] == LEADER:
                has_followers = False
                for pot_follower in G.inverted_nodes[n]:
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
    def __init__(self, deterministic=True, seed=0):
        self.deterministic = deterministic
        self.seed = seed

    def __str__(self):
        return "sub modularity"

    def clustering(self, G, verbose=False):
        X = [set()]
        Y = [set(G.nodes)]

        for i,x in enumerate(G.nodes):
            Xp = X[i].union([x])
            Yp = Y[i].difference([x])
            a = self.f(Xp, G) - self.f(X[i], G)
            b = self.f(Yp, G) - self.f(Y[i], G)
            if self.deterministic:
                keep_i = a >= b
            else:
                ap = max(a, 0.)
                bp = max(b, 0.)
                local_random = random.Random(self.seed)
                keep_i = not ((ap == 0 and bp == 0) or ap/(ap+bp) < local_random.random())

            if keep_i:
                X.append(Xp)
                Y.append(Y[i])
            else:
                X.append(X[i])
                Y.append(Yp)

        leaders = X[-1]
        return self.get_real_leaders(G, leaders)

    @staticmethod
    def get_real_leaders(G, leaders):
        nodes = {}
        real_leaders = set()
        real_followers = set()
        for x in G.nodes:
            if x in leaders:
                nodes[x] = -1
            else:
                max_leader = -2
                for neighbor in G[x]:
                    if neighbor in leaders and (max_leader == -2 or G[x][neighbor].fuel_diff > G[x][max_leader].fuel_diff):
                        max_leader = neighbor
                nodes[x] = max_leader
                if max_leader != -2:
                    real_leaders.add(max_leader)
                    real_followers.add(x)
        return real_followers, real_leaders, nodes, 1

    @staticmethod
    def f(leaders, G):
        followers = set(G.nodes).difference(leaders)
        total = 0
        for follower in followers:
            max_gain = -100000
            for neighbor in G[follower]:
                if neighbor in leaders and G[follower][neighbor].fuel_diff > max_gain:
                    max_gain = G[follower][neighbor].fuel_diff
            if max_gain > 0:
                total += max_gain
        return total
