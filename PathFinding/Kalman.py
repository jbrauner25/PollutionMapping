import osmnx as ox
import math
import networkx as nx
from CoordCart import coord_cart
from poll_env import PolEnv
import utils
import random


class kalman(object):
    def __init__(self, PolEnv):
        self.env = PolEnv

    def wipe_initilize(self):
        '''Sets all pol to 0 and variance to 9999'''
        for node in self.env.graph.nodes():
            self.env.set_node_attribute(node, 'var', random.randrange(999999999999999,99999999999999999999))
            self.env.set_node_attribute(node, 'pol', random.randrange(100,200))

    @staticmethod
    def kalman_filter(node_pol, node_var, meas_pol, meas_var):
        kg = node_var / (node_var + meas_var)
        new_pol = float(node_pol) + kg * (meas_pol - node_pol)
        new_var = (1 - kg) * node_var
        return new_pol, new_var

    @staticmethod
    def flatten(array):
        for x in range(len(array)):
            if type(array[x]) == list:
                return array[:x] + flatten(array[x]) + flatten(array[x + 1:])
        return array

    @staticmethod
    def remove_duplicates(array):
        """Could just use set()"""
        new_array = []
        for x in array:
            if x not in new_array:
                new_array.append(x)
        return new_array

    @staticmethod
    def meas_var_dist(distance):
        """calculates the measured value variance for a given point given
        the variance is linear with respect to distance"""
        var = 500 * distance
        return var

    def update(self, pollution, location_point, count_max):  # Loc_point in lat/long
        count = 0
        start_node = ox.get_nearest_node(self.env.G, location_point)  # Location point must be in lat/long
        cart_loc = coord_cart(location_point,
                              self.env.origin)  # Changes location point into X-Y based on origin for calculating distance
        # Finds the pol data's cartesian coord from grabbing the origin from the start node.
        self.expand_itter([start_node], [], pollution, cart_loc, count, count_max)

    @staticmethod
    def distance(node_loc, loc):  # Uses Cart Points
        dx = node_loc[0] - loc[0]
        dy = node_loc[1] - loc[1]
        return dx ^ 2 + dy ^ 2

    #
    # def expand(self, queued, finished, meas_pollution, location_point, count, count_max):
    # 	queued = queued
    # 	finished = finished
    # 	if count < count_max:
    # 		for node in queued:
    # 			if node not in finished:
    # 				node_loc = self.env.get_node_attribute('cartesian_coords')
    # 				node_to_loc = distance(node_loc, location_point)
    # 				dist_var = meas_var_dist(node_to_loc)
    # 				varr = self.env.get_node_attribute('var')
    # 				if varr:
    # 					priori_node_var = varr
    # 					priori_node_pol = self.env.get_node_attribute('pol')
    # 					node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution, dist_var)
    # 				else:
    # 					node_pol = meas_pollution
    # 					node_var = dist_var
    # 				self.env.set_node_attribute('pol', node_pol)
    # 				self.env.set_node_attribute('var', node_var)
    # 			finished.append(node)
    # 			queued.remove(node)
    # 			queued += set(self.flatten(list(map(lambda x: list(G.successors(node)) + list(G.predecessors(node)), queued))))
    # 			# Using a directed graph, so need to find nodes with edges going to and going away from node expanded, while removing duplicates.
    # 			count += 1
    # 			finished += self.expand(queued, finished, meas_pollution, location_point, count, count_max)
    #
    # 	return finished

    def get_distance(self, node2, lat1=None, long1=None, node1=None):
        if node1 is not None:
            lat1, long1 = self.env.get_node_lat_long(node1)
        lat2, long2 = self.env.get_node_lat_long(node2)
        return ox.utils.great_circle_vec(lat1, long1, lat2, long2)



    def kalman_loop(self, meas_pollution, location_point):
        for node in self.env.nodes():
            # node_loc = self.env.get_node_to_cart(node)
            # node_to_loc = utils.distance(node_loc, location_point)
            node_to_loc = self.get_distance(node, location_point[0], location_point[1])
            meas_dist_var = self.meas_var_dist(node_to_loc)
            priori_node_var = self.env.get_node_attribute(node, 'var')  # Calls before to check if it exists
            if priori_node_var:
                priori_node_pol = self.env.get_node_attribute(node, 'pol')
                node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution,
                                                        meas_dist_var)
            else:
                node_pol = meas_pollution
                node_var = meas_dist_var
                print("stop")
            self.env.set_node_attribute(node, 'pol', node_pol)
            self.env.set_node_attribute(node, 'var', node_var)

def expand_itter(self, queued, finished, meas_pollution, location_point, count, count_max):
        # TODO Write this again based on time, so we go to as far as a depth we can hit until a time limit is reached.
        queued.append("0")
        while count < count_max and queued:
            print(queued)
            if queued[0] == "0":
                count += 1
                queued.append("0")
                queued.pop(0)
                continue
            if queued[0] not in finished:
                node = queued[0]
                node_loc = self.env.get_node_to_cart(node)
                node_to_loc = utils.distance(node_loc, location_point)
                meas_dist_var = self.meas_var_dist(node_to_loc)
                priori_node_var = self.env.get_node_attribute(node, 'var')  # Calls before to check if it exists
                if priori_node_var:
                    priori_node_pol = self.env.get_node_attribute(node, 'pol')
                    node_pol, node_var = self.kalman_filter(priori_node_pol, priori_node_var, meas_pollution,
                                                            meas_dist_var)
                else:
                    node_pol = meas_pollution
                    node_var = meas_dist_var
                    print("stop")
                self.env.set_node_attribute(node, 'pol', node_pol)
                self.env.set_node_attribute(node, 'var', node_var)  # May have to call env as PolEnv._....
            finished.append(queued[0])
            queued.pop(0)
            neighbors = list(self.env.graph.successors(node)) + list(self.env.graph.predecessors(node))
            for neigh in neighbors:
                if neigh not in finished and neigh not in queued:
                    queued.append(neigh)
            if len(queued) == 1:
                return
    # Using a directed graph, so need to find nodes with edges going to and going away from node expanded, while removing duplicates.
    # Can loop through all recorded data points for areas that have None as var at the end.
