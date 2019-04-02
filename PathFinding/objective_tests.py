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
import sys

sys.setrecursionlimit(100000)

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
        return



    def create_map(self):
        self.create_bounds(34.1018951292, 34.0963869167, -117.712251498, -117.7250724571)
        self.create_graph()
        self.planner = new_planner.planner(self.env)

    def test(self):
        self.create_bounds(34.1018951292, 34.0963869167, -117.712251498, -117.7250724571)
        self.create_graph()
        node = self.randomStartNode()
        dist = 1100
        routes = 1
        # route = self.planner.informationgain(origin_node=node, max_dist=dist, min_routes_considered=routes, lambda_1=.000001, loopcounting=True)
        # self.plot_graph_route(route[0])
        # self.planner.env.save_mat(route[0])
        # print("The average difference is:")
        # self.planner.env.compare_truth(route[0])
        #route = self.planner.informationGain(origin_node=node, max_dist=dist, min_routes_considered=routes, lambda_1=1, loopcounting=True)
        route = self.planner.objectiveCellSampling(origin_node=node, max_dist=dist, min_routes_considered=routes, lambda_1=0.00001,
                                             )
        self.planner.env.save_mat(route[0])
        print("The average difference is:")
        self.planner.env.compare_truth(route[0])

        # time.sleep(5)
        # dist = 2000
        # route = self.planner.Coverage(origin_node=node, max_dist=dist, min_routes_considered=routes, loopcounting=True)
        # #route = self.planner.intelligentsampling(origin_node=node, max_dist=dist, min_routes_considered=routes, lambda_1=1.0, loopcounting=True)
        # print("2****ONE" + str(route[1]) + 'count: ' + str(route[2]))
        # self.plot_graph_route(route[0])
        # #self.planner.env.compare_truth(route[0])
        # self.planner.env.save_mat(route[0])
        # print("2********one" + str(route[1]) + 'count: ' + str(route[2]))
        # time.sleep(5)
        # route = self.planner.NormalizedCoverage(origin_node=node, max_dist=dist, min_routes_considered=routes, loopcounting=True)
        # self.planner.env.compare_truth(route[0])
        # print("two: " + str(route[1]) + 'count: ' + str(route[2]))
        # self.plot_graph_route(route[0])
        # print("two: " + str(route[1]) + 'count: ' + str(route[2]))


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

    def script_truth(self):
        min_routes = 1
        for y in range(1, 41, 5):
            objective = []
            routeDistance = y*100
            print("CURRENT ROUTE DISTANCE: " + str(routeDistance))
            for x in range(1, 20):
                node = self.randomStartNode()
                route1 = self.planner.Coverage(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes)
                route2 = self.planner.SimAnnealCoverage(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes)
                route3 = self.planner.RandomCoverage(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes)
                # route4 = self.planner.objectiveCellSampling(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes, lambda_1=0.5)
                # route5 = self.planner.objectiveCellSampling(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes, lambda_1=0.99999)
                # route6 = self.planner.objectiveCellSampling(origin_node=node, max_dist=routeDistance, min_routes_considered=min_routes, lambda_1=0.00001)
                objective.append((self.planner.env.compare_truth(route1[0]), self.planner.env.compare_truth(route2[0]), self.planner.env.compare_truth(route3[0])))
                print(str(x) + " point in loop. Goal: 20")

            with open(str(y*100) + 'PathDistanceCompareTruthMaxMin.csv', 'w', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, dialect='excel')
                spamwriter.writerow(['coverage', 'max', 'min', 'simannealcoverage', 'max', 'min', 'randomcoverage', 'max', 'min'])
                for x in range(len(objective)):
                    spamwriter.writerow(
                        [objective[x][0][0], objective[x][0][1], objective[x][0][2], objective[x][1][0], objective[x][1][1], objective[x][1][2], objective[x][2][0],objective[x][2][1],objective[x][2][2]])

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
#planner.test()
planner.script_truth()
# planner.randomscript(500, 100)
# planner.randomscript(1, 100)
# planner.randomscript(1000, 100)
# planner.script(1, 1)
# planner.script(500, 1)
# planner.script(1000, 1)





