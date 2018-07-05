import osmnx as ox
import networkx as nx
import requests
import matplotlib.cm as cm
import matplotlib.colors as colors
import pandas as pd
import pandas as pd
import numpy as np
from utils import saveImagesToGif
from planner import Planner
from poll_env import *
from Kalman import *
import Planning
import planning2
ox.config(use_cache=True, log_console=True)
import randomplanner
import random

class tester(object):
    def __init__(self, env=None):
        if env is not None:
            self.env = env
            self.kalman = kalman(env)
            self.graph = env.graph
        self.north, self.south, self.east, self.west = None, None, None, None
        self.points = []
        self.updated_nodes = []
        self.planner = None
        self.unproj = None

    def create_bounds(self, north, south, east, west):
        self.north, self.south, self.east, self.west = north, south, east, west

    def create_graph(self):
        G = ox.graph_from_bbox(self.north, self.south, self.east, self.west, network_type='drive', simplify=True, clean_periphery=True)
        self.env = PolEnv(G)
        self.graph = self.env.graph
        self.kalman = kalman(self.env)
        self.unproj = G

    def random_kalman(self, pol_count, pol_min, pol_max):
        # self.kalman.wipe_initilize()
        #random.seed(100)
        for x in range(pol_count):
            point = (random.uniform(self.south, self.north), random.uniform(self.east, self.west))
            print(point)
            self.points.append(point)
            self.updated_nodes.append(ox.get_nearest_node(self.env.G, point))
            # self.kalman.update(random.randint(pol_min, pol_max), point, 100)  # r
            self.kalman.kalman_loop(random.randint(pol_min, pol_max), point)
        nc = ox.get_node_colors_by_attr(self.kalman.env.graph, 'pol', cmap='plasma', num_bins=20)
        ns = [50 if node in self.updated_nodes else 8 for node in self.kalman.env.graph.nodes()]
        fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, node_color=nc, node_size=ns, node_zorder=2,
                                edge_color='#dddddd', use_geom=True)
    def plan_random(self, branch_per_expans, lambda_1, lambda_2, length):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.update_edge_weight()
        end, graph, dict = self.planner.random_paths_unique_random_queue(start_node, length, 300)
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        path_converted = self.planner.path_converter(dict, graph, 0, end)
        fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec)

    def plan_dijstras(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijstras(start_node, length, paths_to_consider)
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec)

    def plan_dijstras_multi(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijstras(start_node, length, paths_to_consider)
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec)

    def plan_dijstras_node_counting(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijstras(start_node, length, paths_to_consider)
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec)

    def plan_random_cost_per_node(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.random_paths_cost_per_node(start_node, length, paths_to_consider)
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list)
        fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, edge_color=ec, node_zorder=2,
                                 use_geom=True)
        fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec, route_linewidth=4, route_alpha=0.2)



test = tester()
test.create_bounds(34.0987490224, 34.0918462621, -117.7147334535, -117.7205464802)  # Testing region
#test.create_bounds(34.108257, 34.091846, -117.703695, -117.720559)  # Testing region

#test.create_bounds(34.1, 34, -117, -117.1)

test.create_graph()
test.random_kalman(25, 15000, 20000)
test.plan_dijstras_node_counting(2, 0.09, 0.1, 2200, 100)
test.plan_random_cost_per_node(2, 0.09, 0.1, 2200, 100)
test.plan_dijstras_multi(2, 0.09, 0.1, 2200, 100)
test.plan_dijstras(2,0.9,0.1,2200,100)

# print("pol")
# #test.plan_random(4, 0.4, 0.1, 3000)
# # print("half")
# # test.plan(3, 0.5, 0.0)
# # print("var")
# # test.plan(3, 1, 0.0)
# #
# test.plan_dijstras_multi(4, 0.0, 0.0, 20000, 30)
# test.plan_dijstras(4,0.0,0.0,20000,30)
# test.plan_dijstras(4,0.2,0.0,20000,30)
# print("Dijstras")
# test.plan_dijstras(4, 0.5, 0.1, 1000)
# print("random")
#test.plan_random(2, 0.5, 0.1, 2000)


# north, south, east, west = 34.05, 34, -117, -117.05
# #north, south, east, west = 34.108161, 34.099192, -117.710652, -117.721394
#
# G = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify=True, truncate_by_edge=True)
#
# env = PolEnv(G)
#
# kalman = kalman(env)
#
# kalman.update(30, (34.05, -117.0), 300)  # bot right
# kalman.update(603, (34.05, -117.05), 300)  # bot left
# kalman.update(550, (34.025, -117.025), 300)
#
# kalman.update(495, (34.0, -117.0), 300)  # top right
# kalman.update(400, (34.0, -117.05), 300)  # top left
# planner = randomplanner.planner(env)
# planner.set_config(10, 5, 0.5, 0.4)
# planner.update_edge_weight()
# end, graph, dict = planner.random_paths_unique_random_queue(54570674, 7000, 200)
# path = planner.path_recreator(graph, 0, end)
# path_converted = planner.path_converter(dict, path, 0, end)
# print(path_converted)
# fig, ax = ox.plot_graph_route(env.graph, path_converted, node_size=0)