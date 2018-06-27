import Planning
import osmnx as ox
import networkx as nx
import requests
import matplotlib.cm as cm
import matplotlib.colors as colors
import pandas as pd
import pandas as pd
import numpy as np
from utils import saveImagesToGif
from fleet import Fleet
from planner import Planner
from poll_env import *
from Kalman import *
ox.config(use_cache=True, log_console=True)


north, south, east, west = 34.07, 34, -117, -117.07
G = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify=True, truncate_by_edge=True)

snowplows = Fleet(num_vehicle=1)
env = PolEnv(G, snowplows)

kalman = kalman(env)

kalman.update(600, (34.01, -117.01), 10)

planner = Planning.planner(env)
planner.set_config(100, 100, 0.5, 0.1)

path = planner.find_path((34.05, -117.06), 5000, 50)
print(path)
