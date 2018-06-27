import poll_env
import Tree
import time
import copy
import networkx as nx
from collections import deque
from heapq import heappush, heappop
from itertools import count

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

    def update_edge_weight(self):
        '''New function'''
        first = 1
        past_node_cost = 0
        for u, v, data in self.env.graph.edges(keys=False, data=True):
            if first == 1:
                past_node_cost = self.priority(u)
                first = 0
            node_cost = self.priority(v)
            edge_cost = (past_node_cost + node_cost) / 2
            data['weight'] = edge_cost
            past_node_cost = node_cost

    def set_config(self, iter_max, branch_per_expansion, lambda1, lambda2):
        self.iter_max = iter_max
        self.branch_per_expansion = branch_per_expansion
        self.lambda1 = lambda1
        self.lambda2 = lambda2

    def node_near_path(self, node, path):
        min_dist = 99999999
        for p in path:
            dist = self.env.dist_two_nodes(node, p) < min_dist
            if dist < min_dist:
                min_dist = dist
        return min_dist


    def node_cost(self, node, path):
        length = min(self.env.G.get_edge_data(node, path[-1]).values(), key=lambda x: x['length'])
        node_var = self.env.get_node_attribute(node, 'var')
        node_avg_wt_nbr_dgre = self.env.get_stat('avg_weighted_neighbor_degree', node)
        closeness = self.node_near_path(node, path)
        return length, node_var, node_avg_wt_nbr_dgre, closeness

    def priority(self, node):
        node_var = self.env.get_node_attribute(node, 'var')
        node_pol = self.env.get_node_attribute(node, 'pol')
        node_avg_wt_nbr_dgre = self.env.get_stat(node, 'avg_weighted_neighbor_degree', )
        return (1-self.lambda2) * (self.lambda1 * node_var + (1-self.lambda1)*node_pol) + self.lambda2 * node_avg_wt_nbr_dgre

    def find_path(self, starting_coord, dist_max, time_max):
        starting_node = self.env.get_nearest_node(starting_coord)
        return self.random_tree_ahh(starting_node, dist_max, time_max)



    def dijkstras(self, sources, pred=None, paths=None,
                                  cutoff=None, target=None):
            """Uses Dijkstra's algorithm to find shortest weighted paths

            Parameters
            ----------
            G : NetworkX graph

            sources : non-empty iterable of nodes
                Starting nodes for paths. If this is just an iterable containing
                a single node, then all paths computed by this function will
                start from that node. If there are two or more nodes in this
                iterable, the computed paths may begin from any one of the start
                nodes.

            weight: function
                Function with (u, v, data) input that returns that edges weight

            pred: dict of lists, optional(default=None)
                dict to store a list of predecessors keyed by that node
                If None, predecessors are not stored.

            paths: dict, optional (default=None)
                dict to store the path list from source to each node, keyed by node.
                If None, paths are not stored.

            target : node label, optional
                Ending node for path. Search is halted when target is found.

            cutoff : integer or float, optional
                Depth to stop the search. Only return paths with length <= cutoff.

            Returns
            -------
            distance : dictionary
                A mapping from node to shortest distance to that node from one
                of the source nodes.

            Notes
            -----
            The optional predecessor and path dictionaries can be accessed by
            the caller through the original pred and paths objects passed
            as arguments. No need to explicitly return pred or paths.

            """
            nx = self.env.graph
            push = heappush
            pop = heappop
            dist = {}  # dictionary of final distances
            seen = {}
            # fringe is heapq with 3-tuples (distance,c,node)
            # use the count c to avoid comparing nodes (may not be able to)
            c = count()
            fringe = []
            for source in sources:
                seen[source] = 0
                push(fringe, (0, 0, 0, next(c), source))
            while fringe:
                (p, l, c, _, v) = pop(fringe)
                if v in dist:
                    continue  # already searched this node.
                dist[v] = (l, c, p)
                if v == target:
                    break  # We aren't doing a targeted search, so #TODO find a way to make this untargeted
                for u, e in nx.successors(v).items():
                    cost = self.env.get_edge_data((v, u), 'weight')  # Make this return a tuple of (Length, Cost, 0)
                    if cost is None:
                        continue  # Lets assume we don't have any nodes skipped
                    length = dist[v][0] + cost[0]
                    cost = dist[v][1] + cost[1]
                    priori = length / cost
                    if cutoff is not None:
                        if length > cutoff:  # Checks distance in vu_distance tuple in case it is above cutoff distance.
                            #TODO Make the program save the paths (or find out how to access them) once above the cutoff. This is our limiting factor, along with processing time, hopefully.
                            continue
                    # if u in dist:  # We can see a decrease in cost
                    #     if vu_dist < dist[u]:
                    #         raise ValueError('Contradictory paths found:',
                    #                          'negative weights?')
                    elif u not in seen or priori < seen[u][2]:
                        seen[u] = (length, cost, priori)
                        push(fringe, (priori, length, cost, next(c), u))
                        if paths is not None:
                            paths[u] = paths[v] + [u]
                        if pred is not None:
                            pred[u] = [v]
                    elif priori == seen[u][2]:
                        if pred is not None:
                            pred[u].append(v)

            # The optional predecessor and path dictionaries can be accessed
            # by the caller via the pred and paths objects passed as arguments.
            # Dist will be a dictionary of nodes as key, and tuple (Length, cost, and priori).
            # Because this function probably (hopefully) normalizes distances, priori will be the value wanted to grab the min of.
            return dist, paths, pred

    # def put_node_to_queue(self, node):
    # 	heapq.heappush(node)
    #
    # def expand_tree:
    #
    # def a_star_search(self, start_node, time, distance):
    # 	frontier = PriorityQueue()
    # 	frontier.put(start_node, 0)
    # 	came_from = {}
    # 	cost_so_far = {}
    # 	came_from[start_node] = None
    # 	cost_so_far[start_node] = (0,0)
    # 	t = time.time()
    #
    # 	while not frontier.empty():
    # 		current = frontier.get()
    #
    # 		if time.time() - t < time:
    # 			break
    #
    # 		for next in l
    # 			new_cost = cost_so_far[current] + graph.cost(current, next)
    # 			if next not in cost_so_far or new_cost < cost_so_far[next]:
    # 				cost_so_far[next] = new_cost
    # 				priority = new_cost + heuristic(goal, next)
    # 				frontier.put(next, priority)
    # 				came_from[next] = current
    #
    # 	return came_from, cost_so_far
    #
    #
    # def dijkstra_search(self, graph, start, distance_max):
    # 	frontier = PriorityQueue()
    # 	frontier.put(start, 0)
    # 	came_from = {}
    # 	cost_so_far = {}
    # 	came_from[start] = None
    # 	cost_so_far[start] = 0,0
    # 	path_list = []
    #
    #
    #
    # 	while not frontier.empty():
    # 		current = frontier.get_no_pop()
    #
    # 		if cost_so_far[current][1] >= distance_max:  # change to time
    # 			cost = cost_so_far[current]
    # 			path_list.append([frontier.get(), cost])
    #
    # 		for next in list(self.env.graph.successors(current)) + list(self.env.graph.predecessors(current)):
    # 			distance = self.env.get_path_length([current, next])
    # 			new_cost = cost_so_far[current][0] + self.cost(current, next, distance)
    # 			new_distance = cost_so_far[current][1] + distance
    # 			if next not in cost_so_far or new_cost < cost_so_far[next]:
    # 				cost_so_far[next] = new_cost
    # 				priority = new_cost
    # 				frontier.put(next, priority)
    # 				came_from[next] = current
    #
    # 	return came_from, cost_so_far

    def random_tree_ahh(self, start, distance_max, time_max):
        frontier = PriorityQueue()
        frontier.put([start], 0)
        cost_so_far = {}
        cost_so_far[start] = [0, 0]
        path_list = []
        time_start = time.time()

        while not frontier.empty() and time.time() - time_start <= time_max:
            current_path = [frontier.get_no_pop()]
            print("CURRENT PATH " + str(current_path))
            current_node = current_path[-1]

            if cost_so_far[current_path][1] >= distance_max:  # change to time
                cost = cost_so_far[current_path]
                path_list.append([frontier.get(), cost])

            neighbors = list(self.env.graph.successors(current_node)) + list(self.env.graph.predecessors(current_node))

            for next in neighbors:
                if next != current_path[-2]:
                    distance = self.env.get_path_length([current_node, next])
                    new_priority = (cost_so_far[current_path][0] + self.priority(current_node, next, distance))/2
                    print(new_priority)
                    print(cost_so_far[current_path])
                    new_distance = cost_so_far[current_path][1] + distance
                    frontier.put(next, new_priority)
                    current_path.append(next)
                    cost_so_far[current_path] = new_priority, new_distance
        return path_list





