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
import Planning
ox.config(use_cache=True, log_console=True)



# north, south, east, west = 34.07, 34, -117, -117.07
north, south, east, west = 34.108161, 34.099192, -117.710652, -117.721394

G = ox.graph_from_bbox(north, south, east, west, network_type='drive', simplify=True, truncate_by_edge=True)

env = PolEnv(G)

kalman = kalman(env)

kalman.update(505, (34.108161, -117.710652), 15)  # bot right
kalman.update(503, (34.108161, -117.721394), 15)  # bot left
kalman.update(550, (34.103676500000006, -117.719853), 15)

kalman.update(495, (34.099192, -117.710652), 15)  # top right
kalman.update(497, (34.099192, -117.721394), 15)  # top left

planner = Planning.planner(env)
planner.set_config(10, 5, 0.3, 0.1)
planner.update_edge_weight()
planner.dijkstras(4683226686, pred={}, paths={}, cutoff=None)


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