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



north, south, east, west = 34.05, 34, -117, -117.05
#north, south, east, west = 34.108161, 34.099192, -117.710652, -117.721394

G = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify=True, truncate_by_edge=True)

env = PolEnv(G)

kalman = kalman(env)
kalman.wipe_initilize()

kalman.update(500, (34.05, -117.0), 100)  # bot right
nc = ox.get_node_colors_by_attr(env.graph, 'pol', cmap='plasma', num_bins=None)
fig, ax = ox.plot_graph(kalman.env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
kalman.update(500, (34.05, -117.05), 100)  # bot left
nc = ox.get_node_colors_by_attr(env.graph, 'pol', cmap='plasma', num_bins=None)
fig, ax = ox.plot_graph(kalman.env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
kalman.update(20, (34.025, -117.025), 100)
nc = ox.get_node_colors_by_attr(kalman.env.graph, 'pol', cmap='plasma', num_bins=None)
fig, ax = ox.plot_graph(env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
kalman.update(500, (34.0, -117.0), 100)  # top right
nc = ox.get_node_colors_by_attr(env.graph, 'pol', cmap='plasma', num_bins=20)
fig, ax = ox.plot_graph(kalman.env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
kalman.update(500, (34.0, -117.05), 100)  # top left
nc = ox.get_node_colors_by_attr(env.graph, 'pol', cmap='plasma', num_bins=20)
fig, ax = ox.plot_graph(env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
# planner = planning2.planner(env)
# planner.set_config(10, 5, 0.5, 0.9)
# planner.update_edge_weight()
# paths = planner.get_path(54543043)
# print(paths)
# lists = []
# for k in sorted(paths, key=lambda k: len(paths[k]), reverse=True):
#     lists.append(k)
# # routes = planner.dijkstras2([54543043], pred={}, paths={54543043: [54543043]}, cutoff=10000)
# # print(routes)
# for k in lists:
#     fig, ax = ox.plot_graph_route(env.graph, paths[k], node_size=0)
# max = None
# for key, value in dist.items():
#     if max is None or max < value[0]:
#         max = value[0]
#         storedkey = key
# path = [storedkey]
# print(storedkey)
# while True:
#     y = pred[storedkey][0]
#     print(y)
#     path.append(y)
#     if y == 54543043:
#         break
#     storedkey = y

#####fig, ax = ox.plot_graph_route(env.graph, path, node_size=0)
# GRAPHING DATA
# nc = ox.get_node_colors_by_attr(env.graph, 'pol', cmap='plasma', num_bins=20)
#
# fig, ax = ox.plot_graph(env.graph, fig_height=6, node_color=nc, node_size=12, node_zorder=2, edge_color='#dddddd', use_geom=True)
#

# Plotting node centrality
# node_centrality = nx.betweenness_centrality(G)
# ox.stats.extended_stats(G, connectivity=False, anc=False, ecc=False, bc=False, cc=False)
#
# # plot it
# df = pd.DataFrame(data=pd.Series(node_centrality).sort_values(), columns=['cc'])
# df['colors'] = ox.get_colors(n=len(df), cmap=cm.coolwarm, start=0.2)
# df = df.reindex(G.nodes())
# nc = df['colors'].tolist()
# fig, ax = ox.plot_graph(G, bgcolor='k', node_size=30, node_color=nc, node_edgecolor='none', node_zorder=2,
#                         edge_color='#555555', edge_linewidth=1.5, edge_alpha=1)
# snowplows = Fleet(num_vehicle=1)
# env = PolEnv(G, snowplows)
# env.update_cart_coords()
#
# for n in env.nodes:
# 	print(env.get_node_attribute(n, 'x_cart'))
#
# ###############################################

# calculate basic and extended network stats, merge them together, and display
# stats = ox.basic_stats(G)
# extended_stats = ox.extended_stats(G, ecc=True, bc=True, cc=True)
# for key, value in extended_stats.items():
#     stats[key] = value
#
# stats = ox.basic_stats(G)
# for k, count in stats['streets_per_node_counts'].items():
#     stats['int_{}_count'.format(k)] = count
# for k, proportion in stats['streets_per_node_proportion'].items():
#     stats['int_{}_prop'.format(k)] = proportion
#
# # delete the no longer needed dict elements
# del stats['streets_per_node_counts']
# del stats['streets_per_node_proportion']
#
# G_projected = ox.project_graph(G)
# max_node, max_bc = max(extended_stats['clustering_coefficient_weighted'].items(), key=lambda x: x[1])
# max_node, max_bc
#
#
#
# def get_color_list(n, color_map='hot', start=0, end=1):
#     return [cm.get_cmap(color_map)(x) for x in np.linspace(start, end, n)]
#
# def get_node_colors_by_stat(G, data, start=0, end=1):
#     df = pd.DataFrame(data=pd.Series(data).sort_values(), columns=['value'])
#     df['colors'] = get_color_list(len(df), start=start, end=end)
#     df = df.reindex(G.nodes())
#     return df['colors'].tolist()
#
# nc = get_node_colors_by_stat(G_projected, data=extended_stats['avg_weighted_neighbor_degree'])
# fig, ax = ox.plot_graph(G_projected, node_color=nc, node_edgecolor='gray', node_size=20, node_zorder=2)
from networkx.utils import groups
