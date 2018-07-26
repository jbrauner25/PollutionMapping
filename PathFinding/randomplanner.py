import poll_env
import time
import copy
import networkx as nx
import osmnx as ox
from collections import deque
from heapq import heappush, heappop
import itertools
import random
import time
import copy
import math
import numpy as np
import bisect
import scipy.io as sio


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

    def random_paths_expansion_depth_first(self, origin_node, max_dist, time_max):
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
            node, counter_start, nodeset, node_count = queued.pop(0)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= 2:
                succ = random.sample(edit_succ, 2)
            successors = edit_succ
            node_count += 1
            _, cost, length, priori = unique_to_node[counter_start]
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, node_count)
                G.add_edge(counter_start, counter)
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    if end_node is None:
                        end_node = counter
                        max = cost / node_count
                    elif cost / node_count > max:
                        end_node = counter
                        max = cost / node_count
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if time.time() - time_start >= time_max:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node

    def random_paths_expansion_breadth_first(self, origin_node, max_dist):
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
                    queued.append((successor_node, counter, succ_nodeset))
            if time.time() - time_start >= time_max:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node

    def path_brute_force(self, origin_node, max_dist):
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
            _, cost, length, node_counter = unique_to_node[counter_start]
            node_counter += 1
            for successor_node in edit_succ:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, node_counter)
                G.add_edge(counter_start, counter)
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                print(new_length)
                if new_length >= max_dist:
                    if end_node is None:
                        end_node = counter
                        max_cost = cost/node_counter
                    elif cost/node_counter > max_cost:
                        end_node = counter
                        max_cost = cost/node_counter
                    break
                else:
                    queued.append((successor_node, counter, succ_nodeset))
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
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start, nodeset = queued.pop(random.randrange(len(queued)))
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
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
                if new_length >= max_dist:
                    route_count += 1
                    if end_node is None:
                        end_node = [counter]
                    elif G.nodes[end_node]['weight'] > new_priori:
                        end_node = counter
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node

    def random_paths_random_queue(self, origin_node, max_dist, min_routes_considered):
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
        unique_to_node[counter] = (origin_node, 0, 0, 0, 0)
        next(c)
        while queued:
            node, counter_start, nodeset = queued.pop(random.randrange(len(queued)))
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
            successors = edit_succ
            _, cost, length, priori, nodes_visited_counter = unique_to_node[counter_start]
            nodes_visited_counter += 1
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_cost / new_length
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori, nodes_visited_counter)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    if end_node is None:
                        end_node = counter_start
                        current_max = cost / (nodes_visited_counter - 1)
                    elif cost / (nodes_visited_counter - 1) > current_max:
                        current_max = cost / (nodes_visited_counter - 1)
                        end_node = counter_start
                    break
                else:
                    queued.insert(0, (successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                return end_node, G, unique_to_node
        return end_node, G, unique_to_node

    def dijkstras(self, origin_node, max_dist, min_routes_considered):
        c = itertools.count()
        G = nx.Graph()
        push = heappush
        pop = heappop
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        fringe = []
        push(fringe, (0, origin_node, 0, nodeset))
        end_nodes = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0, 0)
        next(c)
        while fringe:
            _, node, counter_start, nodeset = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
            successors = edit_succ
            _, cost, length, priori, node_counter = unique_to_node[counter_start]
            node_counter += 1
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = new_length / new_cost
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori, node_counter)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    if end_nodes is None:
                        end_nodes = [(counter_start, cost/node_counter)]
                    else:
                        end_nodes.append((counter_start, cost/node_counter))
                    break
                else:
                    push(fringe, (new_priori, successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                break
        sorted_end_nodes = sorted(end_nodes, key=lambda x: x[1], reverse=True)
        edited_sorted_end_nodes = [x[0] for x in sorted_end_nodes]
        return edited_sorted_end_nodes, G, unique_to_node

    def dijkstras_path_recreator(self, G, start_node, end_node):
        return nx.shortest_simple_paths(G, start_node, end_node)

    def dijkstras_obj_sum(self, origin_node, max_dist, min_routes_considered):
        c = itertools.count()
        G = nx.Graph()
        push = heappush
        pop = heappop
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        queue = []
        push(queue, (0, origin_node, 0, nodeset))
        end_nodes = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0, 0)
        next(c)
        while queue:
            _, node, counter_start, nodeset = pop(queue)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
            successors = edit_succ
            _, cost, length, priori, node_count = unique_to_node[counter_start]
            node_count += 1
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = priori + new_edge_cost[0] * new_edge_cost[1]
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori, node_count)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    if end_nodes is None:
                        end_nodes = [(counter_start, new_cost / node_count)]
                    else:
                        end_nodes.append((counter_start, new_cost / node_count))
                    break
                else:
                    push(queue, (new_priori, successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                break
        sorted_end_nodes = sorted(end_nodes, key=lambda x: x[1], reverse=True)
        edited_sorted_end_nodes = [x[0] for x in sorted_end_nodes]
        return edited_sorted_end_nodes, G, unique_to_node

    def dijkstras_obj_sum_test(self, origin_node, max_dist, min_routes_considered):
        c = itertools.count()
        G = nx.Graph()
        push = heappush
        pop = heappop
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        queue = []
        push(queue, (0, origin_node, 0, nodeset))
        end_nodes = None
        route_count = 0
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0, 0)
        next(c)
        while queue:
            _, node, counter_start, nodeset = pop(queue)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
            successors = edit_succ
            _, cost, length, priori, node_count = unique_to_node[counter_start]
            node_count += 1
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = (node_count ** 2 / (new_cost * new_length))
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori, node_count)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                print(new_length)
                if new_length >= max_dist:
                    route_count += 1
                    print(route_count)
                    if end_nodes is None:
                        end_nodes = [(counter_start, new_cost / node_count)]
                    else:
                        end_nodes.append((counter_start, new_cost / node_count))
                    break
                else:
                    push(queue, (new_priori, successor_node, counter, succ_nodeset))
                if route_count >= min_routes_considered:
                    break
        sorted_end_nodes = sorted(end_nodes, key=lambda x: x[1], reverse=True)
        edited_sorted_end_nodes = [x[0] for x in sorted_end_nodes]
        return edited_sorted_end_nodes, G, unique_to_node


    def dijstras_node_counting(self, origin_node, max_dist, min_routes_considered):
        c = itertools.count()
        G = nx.Graph()
        push = heappush
        pop = heappop
        route_count = 0
        unique_to_node = {}
        nodeset = set()
        nodeset.add(origin_node)
        queue = []
        push(queue, (0, origin_node, 0, nodeset))
        end_nodes = None
        counter = next(c)
        G.add_node(counter)
        # nx.add_node(G, counter)
        unique_to_node[counter] = (origin_node, 0, 0, 0)
        next(c)
        while queue:
            _, node, counter_start, nodeset = pop(queue)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            if len(edit_succ) >= self.branch_per_expansion:
                edit_succ = random.sample(edit_succ, self.branch_per_expansion)
            successors = edit_succ
            _, cost, length, priori, nodes_visited_counter = unique_to_node[counter_start]
            nodes_visited_counter += 1
            for successor_node in successors:
                new_edge_cost = self.env.get_edge_data((node, successor_node), 'weight')  # returns tuple (cost, length)
                new_length = length + new_edge_cost[1]
                new_cost = cost + new_edge_cost[0]
                new_priori = nodes_visited_counter / (new_cost * new_length)
                counter = next(c)
                unique_to_node[counter] = (successor_node, new_cost, new_length, new_priori, nodes_visited_counter)
                G.add_edge(counter_start, counter)
                G.node[counter]['weight'] = new_priori
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    if end_nodes is None:
                        end_nodes = [(counter_start, cost / nodes_visited_counter)]
                    else:
                        end_nodes.append((counter_start, cost/nodes_visited_counter))
                    break
                else:
                    push(queue, (new_priori, successor_node, counter, succ_nodeset))
            if route_count >= min_routes_considered:
                break
        sorted_end_nodes = sorted(end_nodes, key=lambda x: x[1], reverse=True)
        edited_sorted_end_nodes = [x[0] for x in sorted_end_nodes]
        return edited_sorted_end_nodes, G, unique_to_node

    def path_converter(self, dict, G, start_node, end_node):
        path = nx.shortest_simple_paths(G, start_node, end_node)
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

    def grid_weight_nodes(self, north, south, east, west, meter_box, n):
        for node in self.env.graph.nodes():
            self.graph.nodes()[node]['grid_weight'] = random.uniform(0, 0.00100)
            print(self.graph.nodes()[node]['grid_weight'])
        r_earth = 6378137
        old_lat = south
        old_long = west
        #
        # d_x = new_longitude - west
        # d_y = new_latitude - south
        # x = east - west
        # y = north - south
        # step_x = x // d_x
        # step_y = y / d_y
        coords = []
        row = 0
        col = 0
        while old_lat <= north:
            print("old lat " + str(old_lat))
            new_latitude = old_lat + (meter_box / r_earth) * (180 / math.pi)
            old_long = west
            row += 1
            while old_long <= east:
                col += 1
                print("old_long " + str(old_long))
                new_longitude = old_long + (meter_box / r_earth) * (180 / math.pi) / math.cos(old_lat * math.pi / 180)
                temp_graph = copy.deepcopy(self.env.graph)
                temp_unproj_graph = copy.deepcopy(self.env.G)
                nodes = []
                dist_sum = 0
                for _ in range(n):
                    node_location, dist = ox.get_nearest_node(temp_unproj_graph, (old_lat, old_long), return_dist=True)
                    nodes.append((self.graph.nodes()[node_location], dist))
                    dist_sum += dist
                    temp_unproj_graph.remove_node(node_location)
                for node, dist in nodes:
                    update = dist_sum ** 2 / (n * dist)
                    # update = dist
                    try:
                        node['grid_weight'] += update
                    except KeyError:
                        node['grid_weight'] = update
                coords.append((old_lat, old_long))
                old_long = new_longitude
            old_lat = new_latitude
        print("(row, col) (" + str(row) + ", " + str(col) + ")")
        return coords




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
            data['cost'] = edge_cost
            past_node_cost = node_cost

    def get_path(self, source):
        paths = nx.all_pairs_dijkstra_path(self.graph)
        paths = dict(paths)
        return paths[source]

    def set_config(self, branch_per_expans, lambda1, lambda2):
        self.branch_per_expansion = branch_per_expans
        self.lambda1 = lambda1
        self.lambda2 = lambda2







