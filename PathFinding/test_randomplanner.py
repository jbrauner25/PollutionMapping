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

    def create_bounds(self, north, south, east, west):
        self.north, self.south, self.east, self.west = north, south, east, west

    def create_graph(self):
        G = ox.graph_from_bbox(self.north, self.south, self.east, self.west, network_type='drive', simplify=True, clean_periphery=True)
        self.env = PolEnv(G)
        self.graph = self.env.graph
        self.kalman = kalman(self.env)

    def random_kalman(self, pol_count, pol_min, pol_max):
        for x in range(pol_count):
            point = (random.uniform(self.south, self.north), random.uniform(self.east, self.west))
            self.points.append(point)
            self.updated_nodes.append(self.env.get_nearest_node(point))
            self.kalman.update(random.randint(pol_min, pol_max), point, 200)

    def plan(self):
        start_node = self.env.get_nearest_node(random.choice(self.points))
        self.planner = randomplanner.planner(self.env)
        self.planner.set_config(10, 5, 1.0, 0.0)
        self.planner.update_edge_weight()
        end, graph, dict = self.planner.random_paths_unique_random_queue(54570674, 6000, 50)
        path = self.planner.path_recreator(graph, 0, end)
        path_converted = self.planner.path_converter(dict, path, 0, end)
        print(path_converted)
        nc = ox.get_node_colors_by_attr(self.env.graph, 'var', cmap='plasma', num_bins=20)
        fig, ax = ox.plot_graph_route(self.env.graph, path_converted, node_color=nc)


test = tester()
test.create_bounds(34.05, 34, -117.01, -117.05)
test.create_graph()
test.random_kalman(30, 300, 500)
test.plan()




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