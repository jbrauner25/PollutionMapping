from env import *
import numpy as np
import gridandcell
import utils
import copy
import osmnx as ox
import networkx as nx
import scipy.io as sio
import math
import utm
import matplotlib.pyplot as plt
import random
import time



class PolEnv(Env):
    def __init__(self, unproj_graph, node_location_list=None, origin=None, origin_id=None):
        Env.__init__(self, unproj_graph, node_location_list)
        self.origin = origin
        self.origin_id = origin_id
        self.cart_x_width = 0
        self.cart_y_width = 0
        self.grid = None
        if self.origin is None:
            self.set_origin(self.find_bottom_left())
        self.update_cart_coords()
        self.create_2d_grid()
        self.grid.plot_coords()
        print('ENVIRONMENT LOADED')
        pos = nx.get_node_attributes(self.graph, 'pos')
        nx.draw(self.graph, pos)
        plt.show()


        #self.stats = ox.extended_stats(self.graph, ecc=False, bc=True, cc=False)

    def get_stat(self, node, stat):
        return self.stats[stat][node]

    def compare_truth(self, route):
        self.truthGrid = gridandcell.Grid2DCartesian(self.cart_x_width, self.cart_y_width, 120)
        for node in route:
            cartesian_position = self.get_node_attribute(node, 'pos')
            pollution = self.get_node_attribute(node, 'pol')
            self.truthGrid.add_pollution(pollution, cartesian_position)




    def create_2d_grid(self):
        self.grid = gridandcell.Grid2DCartesian(self.cart_x_width, self.cart_y_width, 120)
        max_distance = math.sqrt(self.grid.width**2 + self.grid.height**2)
        for n, data in self.graph.nodes(data=True):
            node = self.graph.nodes()[n]  # Returns the node attribute's dictionary.
            data['gridindex'] = self.grid.whichCellAmIIn(data['x_cart'], data['y_cart'])
            distances = []
            currentNode = self.grid.list.myHead
            node_position = data['pos']  # returns x, y
            while currentNode.myNext:
                coord = (currentNode.myData[0], currentNode.myData[1])  # x,y
                distance = math.sqrt((node_position[0] - coord[0]) ** 2 + (node_position[1] - coord[1]) ** 2)
                if distance > max_distance:
                    raise Exception('Distance greater than max distance, error in code')
                distances.append(distance)

                currentNode = currentNode.myNext
            data['distances'] = distances
            print("Finished node distance calculations, onto next node")
        print("done")




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
                lat_coords))  # Finds southmost node because Northern hemispere has latitude coordinates
            # decreasing towards equator
            self.origin_id = list(self.graph.nodes())[index]
            self.origin = self.get_node_lat_long(self.origin_id)

    def find_bottom_left(self):
        lat_coords = [coord[0] for coord in self.get_lat_long_coords()]  # For finding the min/origin
        lon_coords = [coord[1] for coord in self.get_lat_long_coords()]  # For finding the min/origin
        print('Minimum coordinate: ' + str(min(lat_coords)) + ', ' + str(min(lon_coords)))
        return (min(lat_coords), min(lon_coords))

    def update_cart_coords(self):
        """Update environment class with cart coord information. Update graph nodes.
        If no origin is previously set, it sets an origin."""
        earthRadius = 6.371e6
        origin = self.origin
        max_x_cart = 0
        max_y_cart = 0
        x_list = []
        y_list = []
        if self.origin is None:
            self.set_origin(self.find_bottom_left())
        for n, data in self.graph.nodes(data=True):
            node = self.graph.nodes()[n]  # Returns the node attribute's dictionary.
            # print(node)
            if self.origin_id and n == self.origin_id:  # Sets (0, 0) meters as value of origin node.
                data['x_cart'] = 0.0
                data['y_cart'] = 0.0
                data['pos'] = 0.0
            else:
                x_cart = (node['lon'] - self.origin[1]) * (np.pi / 180) * np.cos(
                    (node['lat'] - self.origin[0]) / 2 * np.pi / 180) * earthRadius

                y_cart = (node['lat'] - self.origin[0]) * np.pi / 180 * earthRadius
                data['x_cart'] = x_cart
                data['y_cart'] = y_cart
                x_list.append(x_cart)
                y_list.append(y_cart)
                data['pos'] = (x_cart, y_cart)
                if x_cart > max_x_cart:
                    max_x_cart = x_cart
                if y_cart > max_y_cart:
                    max_y_cart = y_cart
        self.cart_x_width = max_x_cart
        self.cart_y_width = max_y_cart
        plt.scatter(x_list, y_list)
        print("plotting")
        plt.show()

    def get_lat_long_coords(self):
        """returns list of tuples giving latitude, longitude pair"""
        return [self.get_node_lat_long(n) for n in self.get_all_nodes()]

    def get_node_lat_long(self, node_location):
        """Returns lat long coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return (node['lat'], node['lon'])

    # def get_node_to_cart(self, node_location):
    #     """Returns stored cart coordinates given node location of graph"""
    #     node = self.graph.nodes()[node_location]
    #     return (node['y_cart'], node['x_cart'])

    def get_node_to_cart(self, node_location):
        """Returns stored cart coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return node['pos']

    def get_node_attribute(self, node_location, node_attribute):
        """Returns lat long coordinates given node location of graph"""
        try:
            node = self.graph.nodes()[node_location]
            return node[node_attribute]
        except:
            return None

    def set_node_attribute(self, node_location, node_attribute, data):
        self.graph.nodes()[node_location][node_attribute] = data

    def grid_weight_nodes(self, north, south, east, west, meter_box, n):
        for node in self.graph.nodes():
            self.graph.nodes()[node]['grid_weight'] = random.uniform(0, 0.00100)
            print(self.graph.nodes()[node]['grid_weight'])
        r_earth = 6378137
        old_lat = south
        old_long = west
        #
        # d_x = new_longitude - west
        # d_y = new_latitude - south
        # x = east - west
        # y = north - south
        # step_x = x // d_x
        # step_y = y / d_y
        coords = []
        row = 0
        col = 0
        # north += (meter_box / r_earth) * (180 / math.pi)
        while old_lat <= north:
            print("old lat " + str(old_lat))
            new_latitude = old_lat + (meter_box / r_earth) * (180 / math.pi)
            old_long = west
            row += 1
            while old_long <= east:
                col += 1
                print("old_long " + str(old_long))
                new_longitude = old_long + (meter_box / r_earth) * (180 / math.pi) / math.cos(old_lat * math.pi / 180)
                temp_graph = copy.deepcopy(self.graph)
                temp_unproj_graph = copy.deepcopy(self.G)
                nodes = []
                dist_sum = 0
                for _ in range(n):
                    node_location, dist = ox.get_nearest_node(temp_unproj_graph, (old_lat, old_long), return_dist=True)
                    nodes.append((self.graph.nodes()[node_location], dist, node_location))
                    dist_sum += dist
                    temp_unproj_graph.remove_node(node_location)
                for node, dist, node_loc in nodes:
                    # update = dist_sum ** 2 / (n * dist)
                    # update = (dist* n )/(dist_sum ** 2)
                    update = dist
                    try:
                        if node['grid_weight'] < update:
                            node['grid_weight'] = update
                            self.env.set_node_attribute(node_loc, 'grid_weight', update)
                    except:
                         node['grid_weight'] = update
                         self.set_node_attribute(node_loc, 'grid_weight', update)
                coords.append((old_lat, old_long))
                old_long = new_longitude
            old_lat = new_latitude
        print("(row, col) (" + str(row) + ", " + str(col) + ")")
        for node in self.graph.nodes():
            print(str(self.graph.nodes()[node]['grid_weight']))
        return coords

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
            new_latitude = origin[0] + (1*coord[n][0] / r_earth) * (180 / math.pi)
            new_longitude = origin[1] + (1*coord[n][1] / r_earth) * (180 / math.pi) / math.cos(new_latitude * math.pi / 180)
            print(str(new_latitude) + str(new_longitude))
            nodes_visualized.append((new_latitude, new_longitude))
            x, y, _, _ = utm.from_latlon(new_latitude, new_longitude)
            grid.add_node(n, lat=new_latitude, lon=new_longitude, pol=pol[n][-1], var=var[n][-1], y=y, x=x)
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
                summed_pol += (1 - (pol_var_dist[m][2] / total_dist)) * pol_var_dist[m][0]
                summed_var += (1 - pol_var_dist[m][2] / total_dist) * pol_var_dist[m][1]
            summed_var = summed_var / 3
            summed_pol = summed_pol / 3
            data['pol'] = summed_pol
            data['var'] = summed_var
            print("pol " + str(summed_pol))
        print("finished")
        nc = ox.get_node_colors_by_attr(self.graph, 'pol', cmap='plasma', num_bins=20)
        fig, ax = ox.plot_graph(self.G, fig_height=6, node_zorder=2,
                                edge_color='#dddddd', node_color=nc, use_geom=True, show=False, close=False)  # node_color=nc
        for lat, long in nodes_visualized:
            ax.scatter(long, lat, c='red', alpha=0.1, s=2)
        ax.set_title("Imported FRF Pollution")
        plt.show()

    '''''''*******************************************************************'''

    def kalman_loop(self, meas_pollution, location_point):
        '''Loops through every node in given graph and runs kalman filtering on it's data'''
        for node in self.nodes():
            # node_loc = self.env.get_node_to_cart(node)
            # node_to_loc = utils.distance(node_loc, location_point)
            meas_dist_var = self.meas_var_dist(node, location_point)
            priori_node_var = self.get_node_attribute(node, 'var')  # Calls before to check if it exists
            if priori_node_var:
                priori_node_pol = self.get_node_attribute(node, 'pol')
                node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution,
                                                        meas_dist_var)
            else:
                node_pol = meas_pollution
                node_var = meas_dist_var
                print("stop")
            self.env.set_node_attribute(node, 'pol', node_pol)
            self.env.set_node_attribute(node, 'var', node_var)

    def meas_var_dist(self, node1, loc):
        node1Loc = self.get_node_attribute(node1, 'pos')
        distance = math.sqrt((node1Loc[0] - loc[0]) ** 2 + (node1Loc[1] - loc[2]) ** 2)
        var = (1 / 5) * distance
        return var

    @staticmethod
    def kalman_filter(node_pol, node_var, meas_pol, meas_var):
        '''Function returns a filtered pollution and variance through kalman filtering'''
        kg = node_var / (node_var + meas_var)
        new_pol = float(node_pol) + kg * (float(meas_pol) - float(node_pol))
        new_var = (1 - kg) * node_var
        return new_pol, new_var

    def random_kalman(self, pol_count, pol_min, pol_max):
        '''Updates grid, returns points and polltion,
        then runs kalman filtering on graph nodes for use in truth table '''
        pointlocationlist = self.grid.random_kalman(pol_count, pol_min, pol_max)
        for element in pointlocationlist:
            self.kalman_loop(element[1], element[0]) #pollution, point
