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



north, south, east, west = 34.05, 34, -117, -117.05
#north, south, east, west = 34.108161, 34.099192, -117.710652, -117.721394

G = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify=True, truncate_by_edge=True)

env = PolEnv(G)

kalman = kalman(env)

kalman.update(30, (34.05, -117.0), 300)  # bot right
kalman.update(603, (34.05, -117.05), 300)  # bot left
kalman.update(550, (34.025, -117.025), 300)

kalman.update(495, (34.0, -117.0), 300)  # top right
kalman.update(400, (34.0, -117.05), 300)  # top left

planner = randomplanner.planner(env)
planner.set_config(10, 5, 0.9, 0.1)
planner.update_edge_weight()
end, graph, dict = planner.random_paths_unique_random_queue(54570674, 7000, 500)
path = planner.path_recreator(graph, 0, end)
path_converted = planner.path_converter(dict, path, 0, end)
print(path_converted)
fig, ax = ox.plot_graph_route(env.graph, path_converted, node_size=0)