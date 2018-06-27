##########################################################################
# Module: planner.py
# Description: Fleet Planner for env.py
# Copyright: Zenith Robotics Inc.
##########################################################################

# Python Imports
from copy import deepcopy
import random
import osmnx as ox
from utils import distance
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt


class Planner(object):
    """Planner object to plan for next step of vehicles in vehicle list.
    Planner should update each vehicle's next_nodes list.
    Takes in environment and vehicles.
    Can choose between different strategies."""

    def __init__(self, env):
        """Initialize Planner object with vehicles and env, and conditions?"""
        self.env = env
        self.vehicles = env.vehicles
        self.planned_node_list = []  # List of planned next_nodes for vehicles


    def auction_planner(self):
        """Start a auction planner that calls a auction multiple times"""
        # Simulation variables
        max_num_plan_calls = 1000
        best_max_path_cost = 9999999
        best_cost = 99999999
        best_planned_paths = []
        costs = []  # see how cost decreases as time goes
        best_max_path_cost_list = []
        best_overlap_list = []
        best_planned_assigned_edges = []

        # reset vehicle assigned edges object
        for vehicle in self.vehicles:
            vehicle.assigned_edges = []

        # call the planner many times since there is random edge allocations
        for call_num in range(max_num_plan_calls):
            if call_num % 1000 == 0:
                print("call_num", call_num)
            # initialize planned path list with vehicle's current node for
            # current planner call
            planned_paths = []
            for vehicle in self.vehicles:
                # if still in the middle of an edge, have to finish current edge
                if len(vehicle.next_nodes) > 0 and vehicle.curr_node is not vehicle.next_nodes[0]:
                    planned_paths.append([vehicle.curr_node, vehicle.next_nodes[0]])

                else:
                    planned_paths.append([vehicle.curr_node])
            planned_assigned_edges = [[] for _ in range(len(self.vehicles))]
            # initialize list of plowed edges from new plan
            planned_plowed_edge_list = deepcopy(self.env.get_plowed_list())
            edges = self.env.get_all_edges()
            random.shuffle(edges)  # shuffles in place
			# TODO Why shuffle?

            # TODO: make as fleet function later
            curr_vehicle_edges = [vehicle.curr_edge for vehicle in self.vehicles]

            # auction all edges
            for edge_num, edge in enumerate(edges):
                # if edge_num % 100 == 0:
                #     print "edge_num", edge_num
                total_overlap = 0
                if edge not in planned_plowed_edge_list and \
                    edge not in self.env.plowed_list and edge not in curr_vehicle_edges:
                    min_path_snowplow, planned_paths, planned_plowed_edge_list = self.auction_edge(
                        edge, planned_paths, planned_plowed_edge_list)
                    # print "after auction edge", edge, min_path_snowplow, planned_paths, planned_plowed_edge_list
                    planned_assigned_edges[min_path_snowplow].append(edge)

            # check if this plan is better than the current best
            planned_paths_cost = [self.get_path_total_cost(
                path) for path in planned_paths]
            planned_path_overlap = self.env.calc_overlaps_in_list(planned_plowed_edge_list)
            max_path_cost = max(planned_paths_cost)
            max_cost = self.calc_auction_cost(max_path_cost, planned_path_overlap)

            # if so, replace variables
            if max_cost < best_cost:
                best_max_path_cost = max_path_cost
                best_planned_paths = planned_paths
                best_overlap = planned_path_overlap
                best_cost = max_cost
                best_planned_assigned_edges = planned_assigned_edges


            # print "cost, overlap", best_cost, best_overlap
            # keep track of metric
            costs.append(best_cost)
            best_max_path_cost_list.append(best_max_path_cost)
            best_overlap_list.append(best_overlap)

        # update vehicle next nodes to best plan
        for veh_id, vehicle in enumerate(self.vehicles):
            # find path from last planned node to end node
            path_to_end = self.env.get_shortest_path(best_planned_paths[veh_id][-1], vehicle.end_node)
            vehicle.next_nodes = best_planned_paths[veh_id] + path_to_end[1:]
            print("vehicle next nodes after auction", vehicle.vehicle_id, vehicle.next_nodes)
            vehicle.assigned_edges = best_planned_assigned_edges[veh_id]
            print("vehicle assigned edges", vehicle.vehicle_id, vehicle.assigned_edges)
            if len(vehicle.next_nodes) > 1:
                # TODO: somehow move this to advance vehicle
                vehicle.curr_edge = (vehicle.next_nodes[0], vehicle.next_nodes[1])

        # ax = plt.subplot(111)
        # plt.plot(costs, label='Cost')
        # plt.plot(best_max_path_cost_list, label='Max Path Cost')
        # plt.plot(best_overlap_list, label='Min Overlap')
        # plt.legend(('Cost', 'Max Path Cost', 'Max Overlap'))
        # plt.xlabel('Iteration')
        # plt.show()

    @staticmethod
    def edge_to_list(path):
        """Make a list of edges (x,y) from a path.
        """
        return [(path[i], path[i + 1]) for i in range(len(path) - 1)]

    def auction_edge(self, edge, planned_paths, planned_plowed_edges):
        """Start auction for one edge. Return list of planned paths after auction."""
        # TODO What is a snow plow. Why are we auctioning off an edge. How do we decide an edge? How do we auction off an edge to multiple vehicles?
        # loop through snow plows and find sp with shortest path
        min_path_length = 9999999
        min_path_snowplow = -1
        min_overlap = 9999999
        min_cost = 999999999
        min_additional_path = []

        # get impedance due to traffic
        traffic_imped = self.env.get_edge_traffic(edge)
        # find plow with shortest path if edge is added
        for vehicle_id, vehicle in enumerate(self.vehicles):
            veh_planned_path = planned_paths[vehicle_id]  # TODO Why are there multiple planned paths for 1 vehicle?
            last_planned_node = veh_planned_path[-1]

            try:
                 # minimum path length if node added
                additional_short_path = self.env.get_shortest_path(
                    last_planned_node, edge[0])
                additional_short_path += [edge[1]]
                new_path_cost = self.get_path_total_cost(
                    veh_planned_path) + \
                    self.get_path_total_cost(additional_short_path)
                vehicle_edge_list_planned = self.edge_to_list(
                    additional_short_path)
                new_overlap = self.env.calc_overlaps_in_list(
                    planned_plowed_edges +
                    vehicle_edge_list_planned)  # new total overlap
                # calculate cost if this vehicle takes edge
                cost = self.calc_auction_cost(new_path_cost, new_overlap)
            except:  # if no paths exist
                # print "no path exists"
                cost = 999999999

            if cost < min_cost:  # if cost is better than before, save variable
                min_cost = cost
                min_path_length = self.get_path_total_length(veh_planned_path) + \
                    self.get_path_total_length(additional_short_path)
                min_path_snowplow = vehicle.vehicle_id
                min_additional_path = additional_short_path

        if min_path_snowplow != -1:  # if a snowplow won auction
            # add new path to snowplow that won auction
            planned_paths[min_path_snowplow] = planned_paths[min_path_snowplow] + \
                min_additional_path[1:]
            # add edges to planned plowed edges list
            planned_plowed_edges += self.edge_to_list(min_additional_path)
        return min_path_snowplow, planned_paths, planned_plowed_edges

    @staticmethod
    def calc_auction_cost(new_length, new_overlap):
        """calculate cost given new length and overlap"""
        alpha_cost = 0.5
        # TODO: update cost functions (make into a python fucntion)
        cost = alpha_cost * new_length + (1 - alpha_cost) * 100 * new_overlap
        return cost

    def get_path_cost(self, path):
        """Get cost of each edge in path. Currently: length * traffic."""
        path_traffic = self.env.get_path_traffic(path)
        path_length = self.env.get_path_length(path)
        return np.multiply(path_traffic, path_length)

    def get_path_total_cost(self, path):
        """Get total cost of path."""
        path_costs = self.get_path_cost(path)
        return np.sum(path_costs)

    # TODO: move to env.py
    def get_path_total_length(self, path):
        """Get total length in meter of path."""
        path_lengths = self.env.get_path_length(path)
        return np.sum(path_lengths)

    def get_next_node(self, vehicle):
        """Chooses the next node for the vehicle."""
        # TODO why does this iterate 3 times? Wouldn't this find a node from the initial node with a max level of 3
        neighbors = self.find_neighbor_nodes(vehicle.getLocation())
        new_node = random.choice(neighbors)
        count = 0

        while new_node in vehicle.prev_nodes and count < 3:
            new_node = random.choice(neighbors)
            count += 1

        return new_node

    def init_next_nodes(self):
        """Initialize next nodes list."""
        print("init_next_nodes")
        any_done = False
        for vehicle in self.vehicles:
            if vehicle.done:
                any_done=True

        if any_done:
            self.auction_planner()
