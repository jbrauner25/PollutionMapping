from env import *
import numpy as np
import utils
import copy
import osmnx as ox
import networkx as nx
import scipy.io as sio
import math
import utm
import matplotlib.pyplot as plt



class PolEnv(Env):
    def __init__(self, unproj_graph, node_location_list=None, origin=None, origin_id=None):
        Env.__init__(self, unproj_graph, node_location_list)
        self.origin = origin
        self.origin_id = origin_id
        if self.origin is None:
            self.set_origin()
        self.update_cart_coords()
        self.stats = ox.extended_stats(self.graph, ecc=False, bc=True, cc=False)

    def get_stat(self, node, stat):
        return self.stats[stat][node]

    def get_edge_data(self, edge, data):
        """Get length of edge. Currently uses shortest edge if there are multiple edges that share origin/dest."""
        shortest_edge = min(self.graph.get_edge_data(edge[0], edge[1]).values(), key=lambda x: x['length'])
        return shortest_edge[data]

    def dist_two_nodes(self, node1, node2):
        return utils.distance(self.get_node_to_cart(node1), self.get_node_to_cart(node2))

    def set_origin(self, origin_coord=None):
        """If the origin coordinate is none, chooses the southernmost node in the graph to be the origin"""
        if origin_coord:
            self.originID = None
            self.origin = origin_coord
        else:
            lat_coords = [coord[0] for coord in self.get_lat_long_coords()]  # For finding the min/origin
            index = lat_coords.index(min(
                lat_coords))  # Finds southmost node because Northern hemispere has latitude coordinates decreasing towards equator
            self.origin_id = list(self.graph.nodes())[index]
            self.origin = self.get_node_lat_long(self.origin_id)

    def update_cart_coords(self):
        """Update environment class with cart coord information. Update graph nodes.
        If no origin is previously set, it sets an origin."""
        earthRadius = 6.371e6
        origin = self.origin
        if self.origin is None:
            self.set_origin()
        for n, data in self.graph.nodes(data=True):
            node = self.graph.nodes()[n]  # Returns the node attribute's dictionary.
            # print(node)
            if self.origin_id and n == self.origin_id:  # Sets (0, 0) meters as value of origin node.
                data['x_cart'] = 0.0
                data['y_cart'] = 0.0
            else:
                data['y_cart'] = (node['lat'] - self.origin[0]) * (np.pi / 180) * np.cos(
                    (node['lon'] - self.origin[1]) / 2 * np.pi / 180) * earthRadius
                data['x_cart'] = (node['lon'] - self.origin[1]) * np.pi / 180 * earthRadius

    def get_lat_long_coords(self):
        """returns list of tuples giving latitude, longitude pair"""
        return [self.get_node_lat_long(n) for n in self.get_all_nodes()]

    def get_node_lat_long(self, node_location):
        """Returns lat long coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return (node['lat'], node['lon'])

    def get_node_to_cart(self, node_location):
        """Returns stored cart coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return (node['y_cart'], node['x_cart'])

    def get_node_attribute(self, node_location, node_attribute):
        """Returns lat long coordinates given node location of graph"""
        try:
            node = self.graph.nodes()[node_location]
            return node[node_attribute]
        except:
            return None

    def set_node_attribute(self, node_location, node_attribute, data):
        self.graph.nodes()[node_location][node_attribute] = data

    def load_data(self, file_path, origin):
        r_earth = 6378137
        mat_contents = sio.loadmat(file_path)
        array_mat = mat_contents
        var = array_mat['var_pred']
        pol = array_mat['Y_pred']
        coord = mat_contents['central_coord']
        min0 = min(coord[0])
        min1 = min(coord[1])
        max0 = max(coord[0])
        max1 = max(coord[1])
        bottom_left = (min0, min1)
        bottom_right = (min0, max1)
        top_right = (max0, max1)
        top_left = (max0, min1)
        grid = nx.Graph()
        nodes_visualized = []
        for n in range(len(coord)):
            # Origin[0] = lat Origin[1] = long
            new_latitude = origin[0] + (coord[n][1] / r_earth) * (180 / math.pi)
            new_longitude = origin[1] + (coord[n][0] / r_earth) * (180 / math.pi) / math.cos(new_latitude * math.pi / 180)
            nodes_visualized.append((new_latitude, new_longitude))
            x, y, _, _ = utm.from_latlon(new_latitude, new_longitude)
            grid.add_node(n, lat=new_latitude, lon=new_longitude, pol=pol[n], var=var[n], y=y, x=x)
        for n, data in self.graph.nodes(data=True):
            node = self.graph.nodes()[n]  # Returns the node attribute's dictionary.
            grid_copy = copy.deepcopy(grid)
            pol_var_dist = []
            total_dist = 0.0
            for grid_node in range(4):
                node_location, dist = ox.get_nearest_node(grid_copy, (node['y'], node['x']), method='euclidean', return_dist=True)
                print(dist)
                pol = grid_copy.node[node_location]['pol']
                var = grid_copy.node[node_location]['var']
                pol_var_dist.append((pol, var, dist))
                total_dist += dist
                grid_copy.remove_node(node_location)
            summed_pol = 0.0
            summed_var = 0.0
            for m in range(4):
                summed_pol += (1 - pol_var_dist[m][2]) * pol_var_dist[m][0]
                summed_var += (1 - pol_var_dist[m][2]) * pol_var_dist[m][1]
            summed_var = summed_var / 4
            summed_pol = summed_pol / 4
            data['pol'] = summed_pol
            data['var'] = summed_var
        print("finished")
        # nc = ox.get_node_colors_by_attr(self.graph, 'pol', cmap='plasma', num_bins=20)
        fig, ax = ox.plot_graph(self.G, fig_height=6, node_zorder=2,
                                edge_color='#dddddd', use_geom=True, show=False, close=False)  # node_color=nc
        for lat, long in nodes_visualized:
            ax.scatter(long, lat, c='red', alpha=0.3, s=3)
        ax.set_title("Weighted nodes from Grid")
        plt.show()



