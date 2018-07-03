import poll_env
import time
import copy
import networkx as nx
from collections import deque
from heapq import heappush, heappop
import itertools
import random
import time
import copy


#
# class PriorityQueue:
# 	def __init__(self):
# 		self.elements = []
#
# 	def empty(self):
# 		return len(self.elements) == 0
#
# 	def put(self, item, priority):
# 		heapq.heappush(self.elements, (priority, item))
#
# 	def get(self):
# 		return heapq.heappop(self.elements)[1]
#
# 	def get_no_pop(self):
# 		return heapq.heappop(self.elements)[0]

class planner(object):
    def __init__(self, env):
        self.env= copy.deepcopy(env)
        self.graph = self.env.graph

    def random_paths(self, origin_node, max_dist, time_max):
        time_start = time.time()
        c = itertools.count()
        G = nx.Graph()
        unique_to_node = {}
        queued = [(origin_node, 0)]
        end_node = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start = queued.pop(0)
            succ = list(self.env.graph.successors(node))
            if len(succ) >= 2:
                succ = random.sample(succ, 2)
            successors = succ
            _, cost, length, priori = unique_to_node[counter_start]
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_cost / new_length
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                print(new_length)
                if new_length >= max_dist:
                    if end_node is None:
                        end_node = counter
                    elif G.nodes[end_node]['weight'] < new_priori:
                        end_node = counter
                else:
                    queued.append((successor_node, counter))
            if time.time() - time_start >= time_max:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node


    def random_paths_unique(self, origin_node, max_dist, time_max):
        time_start = time.time()
        c = itertools.count()
        G = nx.Graph()
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        queued = [(origin_node, 0, nodeset)]
        end_node = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start, nodeset = queued.pop(0)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= 2:
                succ = random.sample(edit_succ, 2)
            successors = edit_succ
            _, cost, length, priori = unique_to_node[counter_start]
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_cost / new_length
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                print(new_length)
                if new_length >= max_dist:
                    if end_node is None:
                        end_node = counter
                    elif G.nodes[end_node]['weight'] < new_priori:
                        end_node = counter
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if time.time() - time_start >= time_max:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node


    def random_paths_unique_random_queue(self, origin_node, max_dist, min_routes_considered):
        time_start = time.time()
        c = itertools.count()
        G = nx.Graph()
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        current_max = 99999999
        queued = [(origin_node, 0, nodeset)]
        end_node = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start, nodeset = queued.pop(random.randrange(len(queued)))
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= 2:
                succ = random.sample(edit_succ, 2)
            successors = edit_succ
            _, cost, length, priori = unique_to_node[counter_start]
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_cost / new_length
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    print("Route!")
                    if end_node is None:
                        end_node = counter_start
                        current_max = priori
                    elif priori < current_max:
                        current_max = priori
                        end_node = counter_start
                    break
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node

    def random_paths_unique_random_queue_multiple_routes(self, origin_node, max_dist, min_routes_considered):
        time_start = time.time()
        c = itertools.count()
        G = nx.Graph()
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        queued = [(origin_node, 0, nodeset)]
        end_node = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start, nodeset = queued.pop(random.randrange(len(queued)))
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= 3:
                succ = random.sample(edit_succ, 3)
            successors = edit_succ
            _, cost, length, priori = unique_to_node[counter_start]
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_cost  # / new_length
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                print(new_length)
                if new_length >= max_dist:
                    route_count += 1
                    if end_node is None:
                        end_node = [counter]
                    elif G.nodes[end_node]['weight'] < new_priori:
                        end_node = counter
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node


    def path_recreator(self, G, start_node, end_node):
        return nx.shortest_simple_paths(G, start_node, end_node)

    def path_converter(self, dict, path, start, end):
        new_path = []
        for node in path:
            for no in node:
                ok = dict[no]
                new_path.append(ok[0])
        return new_path

    def priority(self, node):
        node_var = self.env.get_node_attribute(node, 'var')
        node_pol = self.env.get_node_attribute(node, 'pol')
        node_avg_wt_nbr_dgre = self.env.get_stat(node, 'avg_weighted_neighbor_degree', )
        return (1-self.lambda2) * (self.lambda1 * node_var + (1-self.lambda1)*node_pol) + self.lambda2 * node_avg_wt_nbr_dgre

    def update_edge_weight(self):
        '''Must run set_config before running this'''
        first = 1
        past_node_cost = 0
        for u, v, data in self.graph.edges(keys=False, data=True):
            if first == 1:
                past_node_cost = self.priority(u)
                first = 0
            node_cost = self.priority(v)
            edge_cost = (past_node_cost + node_cost) / 2
            data['weight'] = (edge_cost, data['length'])
            past_node_cost = node_cost

    def get_path(self, source):
        paths = nx.all_pairs_dijkstra_path(self.graph)
        paths = dict(paths)
        return paths[source]

    def set_config(self, iter_max, branch_per_expansion, lambda1, lambda2):
        self.iter_max = iter_max
        self.branch_per_expansion = branch_per_expansion
        self.lambda1 = lambda1
        self.lambda2 = lambda2