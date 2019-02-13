import poll_env
import time
import copy
import networkx as nx
import osmnx as ox
from collections import deque
import heapq
import ObjectiveFunctions
import itertools
import random
import time
import copy
import math
import signal
import time
import threading
import _thread
from time import sleep
import os
import numpy as np
import bisect
import scipy.io as sio
import matplotlib.pyplot as plt
import gridandcell

class planner(object):
    def __init__(self, env):
        self.env = copy.copy(env)
        self.graph = self.env.graph
        self.grid = self.env.grid





    def Coverage(self, origin_node, max_dist, min_routes_considered, loopcounting=False):
        #Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        nodeset.add(origin_node)
        fringe = []
        push(fringe, (0, origin_node, nodeset, 0, [origin_node], [self.env.get_node_attribute(origin_node, 'distances')]))
        paths = []
        randompop = False
        while fringe:
            _, node, nodeset, length, path, distancesarray = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                newdistancesarray = copy.deepcopy(distancesarray)
                newdistancesarray.append(self.env.get_node_attribute(successor_node, 'distances'))
                objective = ObjectiveFunctions.o_Coverage(newdistancesarray)
                average_objective_flipped = 1/objective  # average_objective  # Do this because pop returns lowest element
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, objective))
                    break
                else:
                    push(fringe, (average_objective_flipped, successor_node, succ_nodeset, new_length, unique_path, newdistancesarray))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            # print("objective =" + str(paths[0][1]))
            # return paths[0][0]
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]

    def NormalizedCoverage(self, origin_node, max_dist, min_routes_considered, loopcounting=False):
        #Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        nodeset.add(origin_node)
        fringe = []
        push(fringe, (0, origin_node, nodeset, 0, [origin_node], [self.env.get_node_attribute(origin_node, 'distances')]))
        paths = []
        while fringe:
            _, node, nodeset, length, path, distancesarray = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                newdistancesarray = copy.deepcopy(distancesarray)
                newdistancesarray.append(self.env.get_node_attribute(successor_node, 'distances'))
                objective = ObjectiveFunctions.o_Coverage(newdistancesarray)/new_length
                average_objective_flipped = 1/objective  # average_objective  # Do this because pop returns lowest element
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, objective * new_length))
                    break
                else:
                    push(fringe, (average_objective_flipped, successor_node, succ_nodeset, new_length, unique_path, newdistancesarray))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            # print("objective =" + str(paths[0][1]))
            # return paths[0][0]
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]

    def SimAnnealCoverage(self, origin_node, max_dist, min_routes_considered, loopcounting=False):
        #Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        nodeset.add(origin_node)
        fringe = []
        push(fringe, (0, origin_node, nodeset, 0, [origin_node], [self.env.get_node_attribute(origin_node, 'distances')]))
        paths = []
        randompop = False
        while fringe:
            if randompop:
                i = random.randint(0, len(fringe)-1)
                _, node, nodeset, length, path, distancesarray = fringe[i]
                fringe[i] = fringe[-1]
                fringe.pop()
                if i < len(fringe):
                    heapq._siftup(fringe, i)
                    heapq._siftdown(fringe, 0, i)
            else:
                _, node, nodeset, length, path, distancesarray = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                newdistancesarray = copy.deepcopy(distancesarray)
                newdistancesarray.append(self.env.get_node_attribute(successor_node, 'distances'))
                objective = ObjectiveFunctions.o_Coverage(newdistancesarray)
                editobj = 1/objective
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, objective))
                    break
                else:
                    push(fringe, (editobj, successor_node, succ_nodeset, new_length, unique_path, newdistancesarray))
            routePercentforRandom = (length / max_dist)**(1/2) - 0.35
            randomValue = random.uniform(0, 1)
            if randomValue >= routePercentforRandom:
                randompop = True
            else:
                randompop = False
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            # print("objective =" + str(paths[0][1]))
            # return paths[0][0]
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]


    def RandomCoverage(self, origin_node, max_dist, min_routes_considered, loopcounting=False):
        #Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        nodeset.add(origin_node)
        queue = [(0, origin_node, nodeset, 0, [origin_node], [self.env.get_node_attribute(origin_node, 'distances')])]
        paths = []
        while queue:
            _, node, nodeset, length, path, distancesarray = queue.pop(random.randrange(0,len(queue)))
            succ = list(self.env.graph.successors(node))
            successors = [x for x in succ if x not in nodeset]
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                newdistancesarray = copy.deepcopy(distancesarray)
                newdistancesarray.append(self.env.get_node_attribute(successor_node, 'distances'))
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, distancesarray))
                    break
                else:
                    queue.append(('whatever', successor_node, succ_nodeset, new_length, unique_path, newdistancesarray))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            # print("objective =" + str(paths[0][1]))
            # return paths[0][0]
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            sum_one_over_distance = 0
            for z in range(len(paths[x][1][0]) - 1):  # Distances array
                min_dist = 999999999999
                for y in range(len(paths[x][1]) - 1):
                    current_distance = paths[x][1][y][z]
                    if paths[x][1][y][z] < min_dist:
                        min_dist = current_distance
                sum_one_over_distance += 1 / min_dist  # Maximize this
            print(sum_one_over_distance)
            average_objective = sum_one_over_distance / self.grid.gridsize
            if average_objective > max_objective:
                max_objective = average_objective
                max_index = x
        return paths[max_index][0], max_objective

    ####################################################################################################################
    ####################################################################################################################

    def intelligentsampling_incremental(self, origin_node, max_dist, min_routes_considered, lambda_1,
                                    loopcounting=False):
        # Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        cellset = set()
        starting_cell_loc = self.grid.whichCellAmIIn_loc(self.env.get_node_to_cart(origin_node))
        nodeset.add(origin_node)
        cellset.add(starting_cell_loc)
        fringe = []
        starting_objective = self.grid.get_cell_from_index(starting_cell_loc).o_i(lamba_1)
        push(fringe, (starting_objective, origin_node, nodeset, cellset, 0, [origin_node], starting_objective, 1))
        paths = []
        while fringe:
            _, node, nodeset, prev_cellset, length, path, prev_sum, prev_sum_len = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                cell_loc = self.grid.whichCellAmIIn_loc(self.env.get_node_to_cart(origin_node))
                cell_obj = self.grid.get_cell_from_index(cell_loc).o_i(lamba_1)
                new_objective, new_objective_sum, new_sum_length = ObjectiveFunctions.o_IntelligentSampling_incremental(
                    cellset, cell_obj, cell_loc, prev_sum, prev_sum_len)
                new_objective_flipped = 1 / new_objective
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                succ_cellset = copy.copy(prev_cellset)
                succ_cellset.add(cell_loc)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, new_objective))
                    break
                else:
                    push(fringe, (
                        new_objective_flipped, successor_node, succ_nodeset, succ_cellset, new_length, unique_path,
                        new_objective_sum, new_sum_length))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]


    ####################################################################################################################
    ####################################################################################################################


    def intelligentsampling(self, origin_node, max_dist, min_routes_considered, lambda_1, loopcounting=False):
        # Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        cellset = set()
        starting_point = self.env.get_node_to_cart(origin_node)
        starting_cell_loc = self.grid.whichCellAmIIn_index(starting_point[0], starting_point[1])
        nodeset.add(origin_node)
        cellset.add(starting_cell_loc)
        cell_size = self.grid.gridsize
        fringe = []
        starting_objective = self.grid.get_cell_from_index(int(starting_cell_loc[0]), int(starting_cell_loc[1])).o_i(lambda_1)
        push(fringe, (starting_objective, origin_node, nodeset, cellset, 0, [origin_node], starting_objective))
        paths = []
        while fringe:
            _, node, nodeset, prev_cellset, length, path, prev_sum = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                position = self.env.get_node_to_cart(origin_node)
                cell_loc = self.grid.whichCellAmIIn_index(position[0], position[1])
                cell_obj = self.grid.get_cell_from_index(int(cell_loc[0]), int(cell_loc[1])).o_i(lambda_1)
                new_objective, new_objective_sum = ObjectiveFunctions.o_IntelligentSampling(cellset, cell_obj, cell_loc, prev_sum, cell_size)
                new_objective_flipped = 1/new_objective
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                succ_cellset = copy.copy(prev_cellset)
                succ_cellset.add(cell_loc)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, new_objective))
                    break
                else:
                    push(fringe, (
                    new_objective_flipped, successor_node, succ_nodeset, succ_cellset, new_length, unique_path, new_objective_sum))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]

    def informationgain(self, origin_node, max_dist, min_routes_considered, lambda_1, loopcounting=False):
        # Initilize starting variables
        push = heapq.heappush
        pop = heapq.heappop
        route_count = 0
        loopcount = 0
        nodeset = set()
        cellset = set()
        starting_point = self.env.get_node_to_cart(origin_node)
        starting_cell_loc = self.grid.whichCellAmIIn_index(starting_point[0], starting_point[1])
        nodeset.add(origin_node)
        cellset.add(starting_cell_loc)
        cell_size = self.grid.gridsize
        fringe = []
        starting_objective = self.grid.get_cell_from_index(int(starting_cell_loc[0]), int(starting_cell_loc[1])).o_i(lambda_1)
        push(fringe, (starting_objective, origin_node, nodeset, cellset, 0, [origin_node], starting_objective))
        paths = []
        while fringe:
            _, node, nodeset, prev_cellset, length, path, prev_sum = pop(fringe)
            succ = list(self.env.graph.successors(node))
            edit_succ = [x for x in succ if x not in nodeset]
            successors = edit_succ
            loopcount += 1
            for successor_node in successors:
                new_length = length + self.env.get_edge_data((node, successor_node), 'length')
                position = self.env.get_node_to_cart(origin_node)
                cell_loc = self.grid.whichCellAmIIn_index(position[0], position[1])
                cell_obj = self.grid.get_cell_from_index(int(cell_loc[0]), int(cell_loc[1])).o_i(lambda_1)
                new_objective, new_objective_sum = ObjectiveFunctions.o_IntelligentSampling(cellset, cell_obj, cell_loc, prev_sum, cell_size)
                new_objective_flipped = 1/new_objective
                succ_nodeset = copy.copy(nodeset)
                succ_nodeset.add(successor_node)
                succ_cellset = copy.copy(prev_cellset)
                succ_cellset.add(cell_loc)
                unique_path = copy.copy(path)
                unique_path.append(successor_node)
                if new_length >= max_dist:
                    route_count += 1
                    paths.append((unique_path, new_objective))
                    break
                else:
                    push(fringe, (
                    new_objective_flipped, successor_node, succ_nodeset, succ_cellset, new_length, unique_path, new_objective_sum))
            if route_count >= min_routes_considered:
                break
        if len(paths) <= 0:
            if loopcounting:
                return "error", "error", "error"
            return "error", "error"
        max_index = 0
        max_objective = 0
        for x in range(len(paths)):
            if max_objective < paths[x][1]:
                max_index = x
                max_objective = paths[x][1]
        if loopcounting:
            return paths[max_index][0], paths[max_index][1], loopcount
        return paths[max_index]
