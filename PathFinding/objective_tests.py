import osmnx as ox
import new_planner
import networkx as nx
import requests
import matplotlib.cm as cm
import matplotlib.colors as colors
import pandas as pd
import pandas as pd
import csv
import numpy as np
from utils import saveImagesToGif
from planner import Planner
import itertools
import signal
from poll_env import *
from Kalman import *
import Planning
import planning2

ox.config(use_cache=True, log_console=True)
import randomplanner
import random
import matplotlib.pyplot as plt
import time
import utm
from shapely.geometry import Polygon
from shapely.geometry import LineString, MultiPolygon
import geopandas as gpd
import copy


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
        self.node = None


    def create_bounds(self, north, south, east, west):
        self.north, self.south, self.east, self.west = north, south, east, west

    def create_planner(self):
        self.planner = new_planner.planner(self.env)

    def update_kalman(self, kalman):
        self.kalman = kalman

    def random_kalman(self, pol_count, pol_min, pol_max):
        self.planner.env.random_kalman(pol_count, pol_min, pol_max)

    def set_start_node(self):
        self.node = self.randomStartNode()

    def create_graph(self):
        G = ox.graph_from_bbox(self.north, self.south, self.east, self.west, network_type='drive', simplify=True,
                               clean_periphery=True)
        self.env = PolEnv(G)
        self.graph = self.env.graph
        self.kalman = kalman(self.env)
        self.unproj = G

    def randomStartNode(self):
        latDifference = abs(self.north - self.south)
        longDifference = abs(self.east - self.west)
        newLat = self.south + ((random.uniform(0, 10)) / 10) * latDifference
        newLong = self.west + ((random.uniform(0, 10)) / 10) * longDifference
        return ox.get_nearest_node(self.unproj, (newLat, newLong))

    def plot_graph_route(self, path, filename=None):
        fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, node_zorder=2,
                                use_geom=True)
        if filename:
            fig, ax = ox.plot_graph_route(self.planner.graph, path, route_alpha=0.3, save=True,
                                          filename=filename)
        else:
            fig, ax = ox.plot_graph_route(self.planner.graph, path, route_alpha=0.3)

    def create_map(self):
        self.create_bounds(34.1018951292, 34.0963869167, -117.712251498, -117.7250724571)
        self.create_graph()
        self.planner = new_planner.planner(self.env)

    def test(self):
        self.create_bounds(34.1018951292, 34.0963869167, -117.712251498, -117.7250724571)
        self.create_graph()
        node = self.randomStartNode()
        dist = 500
        routes = 1
        route = self.planner.Coverage(origin_node=node, max_dist=dist, min_routes_considered=routes, loopcounting=True)
        print("one" + str(route[1]) + 'count: ' + str(route[2]))
        self.plot_graph_route(route[0])
        print("one" + str(route[1]) + 'count: ' + str(route[2]))
        route = self.planner.NormalizedCoverage(origin_node=node, max_dist=dist, min_routes_considered=routes, loopcounting=True)
        print("two: " + str(route[1]) + 'count: ' + str(route[2]))
        self.plot_graph_route(route[0])
        print("two: " + str(route[1]) + 'count: ' + str(route[2]))


    def script(self, routes, loop):
        if not self.node:
            node = self.randomStartNode()
        else:
            node = self.node
        dist = 1000
        distance_1000_objective = []
        for _ in range(loop):
            route = self.planner.Coverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_1000_objective.append(route[1])
        distance_2000_objective = []
        dist = 2000
        for _ in range(loop):
            route = self.planner.Coverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_2000_objective.append(route[1])
        distance_3000_objective = []
        dist = 3000
        for _ in range(loop):
            route = self.planner.Coverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_3000_objective.append(route[1])
        with open('BFS' + str(routes) + 'route' + 'compare.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, dialect='excel')
            spamwriter.writerow(['1000 objective', '2000 objective', '3000 objective'])
            for x in range(0, loop):
                spamwriter.writerow([distance_1000_objective[x], distance_2000_objective[x], distance_3000_objective[x]])

    def script_platau(self, routes):
        if not self.node:
            node = self.randomStartNode()
        else:
            node = self.node
        objective = []
        for x in range(1, 49):
            route1k = self.planner.RandomCoverage(origin_node=node, max_dist=1000, min_routes_considered=x)
            route2k = self.planner.RandomCoverage(origin_node=node, max_dist=2000, min_routes_considered=x)
            route3k = self.planner.RandomCoverage(origin_node=node, max_dist=3000, min_routes_considered=x)
            objective.append((x, route1k[1], route2k[1], route3k[1]))
            print(x)
        for x in range(routes):
            route1k = self.planner.RandomCoverage(origin_node=node, max_dist=1000, min_routes_considered=x*50)
            route2k = self.planner.RandomCoverage(origin_node=node, max_dist=2000, min_routes_considered=x*50)
            route3k = self.planner.RandomCoverage(origin_node=node, max_dist=3000, min_routes_considered=x*50)
            objective.append((x*50, route1k[1], route2k[1], route3k[1]))
            print(str(x*50))
        with open('Random1k2k3kPath40mcellData1Through50' + str(routes * 50) + 'route' + 'compare.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, dialect='excel')
            spamwriter.writerow(['route', 'O_1k', 'O_2k', 'O_3k'])
            for x in range(len(objective)):
                spamwriter.writerow(
                    [objective[x][0], objective[x][1], objective[x][2], objective[x][3]])

    def randomscript(self, routes, loop):
        if not self.node:
            node = self.randomStartNode()
        else:
            node = self.node
        dist = 1000
        distance_1000_objective = []
        for _ in range(loop):
            route = self.planner.RandomCoverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_1000_objective.append(route[1])
        distance_2000_objective = []
        dist = 2000
        for _ in range(loop):
            route = self.planner.RandomCoverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_2000_objective.append(route[1])
        distance_3000_objective = []
        dist = 3000
        for _ in range(loop):
            route = self.planner.RandomCoverage(origin_node=node, max_dist=dist, min_routes_considered=routes)
            distance_3000_objective.append(route[1])
        with open('Random' + str(routes) + 'route' + 'compare.csv', 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, dialect='excel')
            spamwriter.writerow(['1000 objective', '2000 objective', '3000 objective'])
            for x in range(0, loop):
                spamwriter.writerow([distance_1000_objective[x], distance_2000_objective[x], distance_3000_objective[x]])


planner = tester()
planner.create_map()
planner.set_start_node()
planner.create_planner()
planner.random_kalman(50, 100, 1000)
planner.test()
#planner.script_platau(10)
# planner.randomscript(500, 100)
# planner.randomscript(1, 100)
# planner.randomscript(1000, 100)
# planner.script(1, 1)
# planner.script(500, 1)
# planner.script(1000, 1)





