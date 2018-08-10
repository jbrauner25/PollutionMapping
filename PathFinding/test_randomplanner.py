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

    def create_bounds(self, north, south, east, west):
        self.north, self.south, self.east, self.west = north, south, east, west

    def update_kalman(self, kalman):
        self.kalman = kalman

    def create_downtown_riverside_graph(self):
        calif = gpd.read_file('ZillowNeighborhoods-CA.shx')
        downtown = calif[(calif['City'] == 'Riverside') & (calif['Name'] == 'Downtown')]
        polygon = downtown['geometry'].iloc[0]
        # a = [[0.86602540378, -0.48480962024], [0.48480962024, 0.86602540378]]
        # polygon = Polygon([(33.974187, -117.374780), (33.980621, -117.388575), (33.998736, -117.376124), (33.992102, -117.360573)])
        # shape2 = ox.round_shape_coords(polygon, 3)
        # shape1 = MultiPolygon([polygon, shape2])
        G = ox.graph_from_polygon(polygon, network_type='drive', simplify=False)
        for node_loc in G.nodes():
            lat = G.nodes()[node_loc]['y'] - 33.9859
            lon = G.nodes()[node_loc]['x'] - -117.375
            rot = rot_matrix(25)
            print(rot)
            result = np.matmul(rot, [lon, lat])
            print(result)
            G.nodes()[node_loc]['y'] = result[1] + 33.9859
            G.nodes()[node_loc]['x'] = result[0] + -117.375
        g_simplify = ox.simplify_graph(G)
        nodelist = copy.deepcopy(g_simplify.nodes())
        # for node_loc in nodelist:
        #     if g_simplify.nodes()[node_loc]['y'] > 33.991 or g_simplify.nodes()[node_loc]['y'] < 33.9752 or g_simplify.nodes()[node_loc]['x'] < -117.367 or g_simplify.nodes()[node_loc]['x'] > -117.385:
        #         g_simplify.remove_node(node_loc)
        self.env = PolEnv(g_simplify)
        self.graph = self.env.graph
        self.kalman = kalman(self.env)
        self.unproj = g_simplify
        self.north = 34.0006
        self.south = 33.9679
        self.east = -117.362
        self.west = -117.392


    def create_graph(self):
        G = ox.graph_from_bbox(self.north, self.south, self.east, self.west, network_type='drive', simplify=True, clean_periphery=True)
        self.env = PolEnv(G)
        self.graph = self.env.graph
        self.kalman = kalman(self.env)
        self.unproj = G

    def random_kalman(self, pol_count, pol_min, pol_max):
        self.kalman.wipe_initilize()
        # random.seed(100)
        for x in range(pol_count):
            point = (random.uniform(self.south, self.north), random.uniform(self.east, self.west))
            print(point)
            #self.points.append(point)
            #self.updated_nodes.append(ox.get_nearest_node(self.env.G, point))
            # self.kalman.update(random.randint(pol_min, pol_max), point, 100)  # r
            self.kalman.kalman_loop(random.randint(pol_min, pol_max), point)
        # nc = ox.get_node_colors_by_attr(self.kalman.env.graph, 'pol', cmap='plasma', num_bins=20)
        # ns = [50 if node in self.updated_nodes else 8 for node in self.kalman.env.graph.nodes()]
        # fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, node_color=nc, node_size=ns, node_zorder=2,
        #                         edge_color='#dddddd', use_geom=True)
        # nc = ox.get_node_colors_by_attr(self.kalman.env.graph, 'var', cmap='plasma', num_bins=20)
        # fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, node_color=nc, node_size=ns, node_zorder=2,
        #                         edge_color='#dddddd', use_geom=True)

    def plot_graph_route_over_cost(self, path_converted, filename=None):
        ec = ox.get_edge_colors_by_attr(self.planner.graph, 'cost', cmap='plasma', num_bins=100)
        fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, edge_color=ec, node_zorder=2,
                                 use_geom=True)
        if filename:
            fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec, route_alpha=0.3, save=True, filename=filename, )
        else:
            fig, ax = ox.plot_graph_route(self.planner.graph, path_converted, edge_color=ec, route_alpha=0.3)

    def plan_random(self, branch_per_expans, lambda_1, lambda_2, length):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.update_edge_weight()
        end, graph, dict = self.planner.random_paths_unique_random_queue(start_node, length, 300)
        path_converted = self.planner.path_converter(dict, graph, 0, end)
        self.plot_graph_route_over_cost(path_converted)

    def plan_dijkstras(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider, from_data=False, filename=None):
        if from_data:
            start_node = ox.get_nearest_node(self.unproj, self.kalman.last_loc)
        else:
            print(self.env.origin)
            start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        start = time.time()
        sorted_end_list, graph, dict = self.planner.dijkstras(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        # nc = ox.get_node_colors_by_attr(self.kalman.env.graph, 'pol', cmap='plasma', num_bins=20)
        # ns = [50 if node in self.updated_nodes else 8 for node in self.kalman.env.graph.nodes()]
        # fig, ax = ox.plot_graph(self.kalman.env.graph, fig_height=6, node_color=nc, node_size=ns, node_zorder=2, edge_color='#dddddd', use_geom=True)
        self.plot_graph_route_over_cost(path_converted, filename=filename)
    def plan_dijkstras_edge_length(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider, from_data=False, filename=None):
        if from_data:
            start_node = ox.get_nearest_node(self.unproj, self.kalman.last_loc)
        else:
            print(self.env.origin)
            start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        start = time.time()
        sorted_end_list, graph, dict = self.planner.dijkstras_edge_lengths(start_node, length, paths_to_consider)
        print(time.time() - start)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted, filename=filename)


    def plan_dijkstras_obj_sum(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijkstras_obj_sum(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted)


    def plan_dijkstras_node_counting(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijkstras_node_counting1(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted)
        sorted_end_list, graph, dict = self.planner.dijkstras_node_counting2(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted)
        self.plot_graph_route_over_cost(path_converted)
        sorted_end_list, graph, dict = self.planner.dijkstras_node_counting3(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted)

    def plan_random_cost_per_node(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider, from_data=False):
        if from_data:
            start_node = ox.get_nearest_node(self.unproj, self.kalman.last_loc)
        else:
            print(self.env.origin)
            start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.random_paths_random_queue(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list)
        self.plot_graph_route_over_cost(path_converted)



    def plan_brute_force(self, lambda_1, lambda_2, length):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(4, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.path_brute_force(start_node, length)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list)
        self.plot_graph_route_over_cost(path_converted)

    def plan_dijkstras_obj_sum_test(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        sorted_end_list, graph, dict = self.planner.dijkstras_obj_sum_test(start_node, length, paths_to_consider)
        path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
        self.plot_graph_route_over_cost(path_converted)

    def plan_diff(self, lambda_1, lambda_2, length):
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(3, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        path = self.planner.difference_pathing(start_node, length)
        self.plot_graph_route_over_cost(path)

    def plan_difference(self, branch_per_expans, lambda_1, lambda_2, length, paths_to_consider,
                                  from_data=False):
        if from_data:
            start_node = ox.get_nearest_node(self.graph, self.kalman.last_loc)
        else:
            print(self.env.origin)
            start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(branch_per_expans, lambda_1, lambda_2)
        self.planner.update_edge_weight()
        path = self.planner.difference_pathing(start_node, length)
        self.plot_graph_route_over_cost(path)

    def grid_weighting_nodes(self, meters, n):
        self.planner = randomplanner.planner(self.kalman.env)
        coords = self.env.grid_weight_nodes(self.north, self.south, self.east, self.west, meters, n)
        nc = ox.get_node_colors_by_attr(self.env.graph, 'grid_weight', cmap='plasma', num_bins=20)
        fig, ax = ox.plot_graph(self.env.G, fig_height=6, node_color=nc, node_zorder=2,
                                edge_color='#dddddd', use_geom=True, show=False, close=False)
        for lat, long in coords:
            print("(" + str(lat) + ", " + str(long) +")")
            ax.scatter(long, lat, c='red', alpha=0.2, s=2)
        ax.set_title("Weighted nodes from Grid")
        plt.show()

    def grid_weighting_nodes_dist(self, meters, n, dist):
        self.planner = randomplanner.planner(self.kalman.env)
        coords = self.planner.grid_weight_nodes_max_dist(self.north, self.south, self.east, self.west, meters, n, dist)
        nc = ox.get_node_colors_by_attr(self.planner.graph, 'grid_weight', cmap='plasma', num_bins=20)
        fig, ax = ox.plot_graph(self.planner.env.G, fig_height=6, node_color=nc, node_zorder=2,
                                edge_color='#dddddd', use_geom=True, show=False, close=False)
        for lat, long in coords:
            print("(" + str(lat) + ", " + str(long) +")")
            ax.scatter(long, lat, c='red', alpha=0.3, s=3)
        ax.set_title("Weighted nodes from Grid")
        for node in self.planner.graph.nodes():
            lat = self.planner.env.G.node[node]['lat']
            long = self.planner.env.G.node[node]['long']
            data = self.planner.graph.node[node]['grid_weight']
            print(str(long) + ", " + str(lat) + ", " + str(data))
            ax.annotate(long, lat, str(data))
        plt.show()

    def timer_test(self):
        times = []
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        self.kalman.wipe_initilize()
        self.random_kalman(200, 20000, 50000)
        self.planner = randomplanner.planner(self.kalman.env)
        self.planner.set_config(2, 0.1, 0.4)
        self.planner.update_edge_weight()
        print("running")
        for x in range(50):
            start = time.time()
            y, x = self.planner.dij_timer(start_node, 21000, 200, 4)
            times.append(time.time() - start)
            print(times)
            print(str(x))
            print(times)
            time.sleep(x)

    def timer_test2(self):
        times = []
        start_node = ox.get_nearest_node(self.unproj, self.env.origin)
        print("running")
        for x in range(200):
            self.kalman.wipe_initilize()
            self.random_kalman(200, 20000, 50000)
            self.planner = randomplanner.planner(self.kalman.env)
            self.planner.set_config(2, 0.1, 0.4)
            self.planner.update_edge_weight()
            start = time.time()
            sorted_end_list, graph, dict, z = self.planner.dij_timer(start_node, 21000, 200, 0.5)
            path_converted = self.planner.path_converter(dict, graph, 0, sorted_end_list[0])
            times.append(time.time() - start)
            print(times)
            print(times)
            print(times)
            print(times)
            print(times)
            time.sleep(z)
            print(times)
            print(times)
            print(times)
            print(times)
        print(times)







def data_collection_loop(test):
    file_name = 2
    testerr = test
    count = itertools.count()
    testerr.kalman.collect_data(file_name)
    testerr.plan_dijkstras(3, 0.2, 1.0, 3500, 500, from_data=True, filename=str(next(count)))
    while True:
        start = ""
        start = input("ready to start recording? (y|n)")
        if start == "n":
            break
        if start != "":
            # testerr.kalman.wipe_initilize()
            testerr.kalman.collect_data(file_name)
            #testerr.kalman.manual_collect_data()
            testerr.plan_dijkstras(4, 0.2, 0.15, 3000, 500, from_data=True, filename= str(next(count)))

def main():
    test = tester()
    test.create_bounds(34.099918, 34.090174, -117.7142334535, -117.723778)
    # Testing region
    test.create_graph()
    test.env.grid_weight_nodes(test.north, test.south, test.east, test.west, 50, 2)
    test.kalman.wipe_initilize()
    data_collection_loop(test)






def UCRdata_collection_loop(test):
    testerr = test
    count = itertools.count()
    testerr.kalman.set_last_loc(33.989097, -117.375862)
    testerr.plan_dijkstras(3, 0.2, 1.0, 5000, 500, from_data=True)
    while True:
            testerr.kalman.wipe_initilize()
            filename1 = input("filename of gps to load")
            filename2 = input("filename of methane to load")
            #testerr.kalman.manual_collect_data()
            testerr.kalman.UCR_Parse(filename1, filename2)
            here = input("pause")
            testerr.plan_dijkstras(4, 0.1, 0.05, 4000, 500, from_data=True)

def UCRmain():
    test = tester()
    test.create_bounds(33.99264, 33.9725, -117.3632322, -117.3861644)
    # Testing region
    test.create_graph()
    # test.create_downtown_riverside_graph()
    test.env.grid_weight_nodes(test.north, test.south, test.east, test.west, 250, 3)
    test.kalman.wipe_initilize()
    UCRdata_collection_loop(test)

def rot_matrix(theta):
    theta = np.deg2rad(theta)
    c, s = np.cos(theta), np.sin(theta)
    return np.array(((c,-s), (s, c)))

# fig, ax = ox.plot_graph(graph)

#main()
#test.create_bounds(34.108257, 34.091846, -117.703695, -117.720559)  # Testing region
#test.create_bounds(34.073797, 34.040584, -118.099624, -118.161938)
#test.create_bounds(34.090749, 34.075594, -117.718872, -117.737704)

UCRmain()
# test.create_bounds(34.1, 34, -117, -117.1)
# #
# test = tester()
# # test.create_bounds(34.056514, 34.011466, -117.669868, -117.724124)
# # #test.create_bounds(34.099918, 34.090174, -117.7142334535, -117.723778) # -117.723543)#  Testing region
# # #test.create_bounds(34.019617, 33.982164, -117.592865, -117.651027)
# # #test.create_bounds(34.019617, 33.982164, -117.592865, -117.651027)
# test.create_downtown_riverside_graph()
# test.create_bounds(34.019617, 33.982164, -117.592865, -117.651027)
# test.create_graph()
# test.grid_weighting_nodes(800, 2)
# # # # test.kalman.test_gps()
# test.grid_weighting_nodes(1000, 4)
# test.
# # test.timer_test()
# test.timer_test2()
# print("kalman")

# test.random_kalman(30, 18000, 70000)
#test.kalman.set_last_loc(33.9851, -117.373)
# print("starting")
# test.plan_diff(0.1, 0.1, 20000)
# test.plan_random_cost_per_node(3, 0.2, 1.0, 18000, 5)
#
# test.plan_dijkstras(2, 0.1, 1.0, 17000, 400)
# x = input("o")
# test.random_kalman(350, 22000, 50000)
# x = input("o")
# test.plan_dijkstras(2, 0.1, 0.4, 21000, 200)
# test.random_kalman(350, 22000, 50000)
# x = input("o")
# test.plan_dijkstras(2, 0.1, 0.4, 21000, 200)
# test.random_kalman(350, 22000, 50000)
# x = input("o")
# test.plan_dijkstras(2, 0.1, 0.4, 21000, 200)
# test.random_kalman(350, 22000, 50000)
# x = input("o")
# test.plan_dijkstras(2, 0.1, 0.4, 21000, 200)
# test.random_kalman(100, 22000, 50000)
# x = input("o")
# test.plan_dijkstras(3, 0.1, 0.0, 8000, 1)
# test.random_kalman(100, 22000, 50000)

# # test.kalman.last_loc = (34.055338, -117.431591)
# test.env.load_data('dat.mat', (34.092116, -117.719905))
# test.plan_difference(3, 0.1, 0.4, 2000, 1, from_data=True)
# #

# main()
#


#test.grid_weighting_nodes(100, 2)
# test.grid_weighting_nodes_dist(100, 4, 200)
# test.grid_weighting_nodes_dist(100, 4, 300)
#test.plan_brute_force(0.09, 0.1, 2200)
#test.plan_random_cost_per_node(2, 0.09, 0.01, 4000, 200)
# test.plan_dijkstras_node_counting(2, 0.05, 0.01, 8000, 100)
# test.plan_dijkstras_node_counting(2, 0.05, 0.01, 8000, 50)
# test.plan_dijkstras_node_counting(2, 0.05, 0.01, 8000, 10)
# test.plan_dijkstras_node_counting(2, 0.05, 0.01, 8000, 5)
#test.plan_dijkstras_node_counting(4, 0.09, 0.01, 10000, 2000)


#test.plan_dijkstras(3, 0.05, 0.01, 8000, 2)

# test.plan_dijkstras_obj_sum(3, 0.09, 0.1, 1500, 100)
# test.plan_dijkstras_obj_sum_test(3, 0.09, 0.1, 1500, 100)

# print("pol")
# #test.plan_random(4, 0.4, 0.1, 3000)
# # print("half")
# # test.plan(3, 0.5, 0.0)
# # print("var")
# # test.plan(3, 1, 0.0)
# #
# test.plan_dijkstras_multi(4, 0.0, 0.0, 20000, 30)
# test.plan_dijkstras(4,0.0,0.0,20000,30)
# test.plan_dijkstras(4,0.2,0.0,20000,30)
# print("Dijstras")
# test.plan_dijkstras(4, 0.5, 0.1, 1000)
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