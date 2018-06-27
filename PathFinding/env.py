# Adapted for Pol Mapping from Zenith Robotics

# Python Imports
import osmnx as ox, networkx as nx
from shapely.geometry.linestring import LineString
from pyproj import Proj
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
import time
import copy


class Env(object):
    def __init__(self, unproj_graph, node_location_list=None):
        """Return an Environment."""
        self.graph = ox.project_graph(unproj_graph)
        self.G = copy.deepcopy(unproj_graph)
        self.nodes = unproj_graph.nodes()

    @staticmethod
    def calc_overlaps_in_list(edge_list):
        """Calculate number of overlaps in edge_list."""
        return len(edge_list) - len(set(edge_list))

    def get_edge_length(self, edge):
        """Get length of edge. Currently uses shortest edge if there are multiple edges that share origin/dest."""
        shortest_edge =  min(self.graph.get_edge_data(edge[0],edge[1]).values(),key=lambda x: x['length'])
        return shortest_edge['length']

    def node_to_utm(self, node_location):
        """Returns lat long coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return (node['y'], node['x'])

    def get_path_length(self, path):
        """Get length of each edge of path. (meters)"""
        return ox.get_route_edge_attributes(self.graph, path, 'length')

    def get_nearest_node(self, point):
        """
        Return the graph node nearest to some specified (y, x) point of the object's graph,
        and optionally the distance between the node and the point.
        """
        return ox.get_nearest_node(self.graph, point)

    def get_all_nodes(self):
        """Return list of all nodes in env's graph."""
        return list(self.graph.nodes())

    def get_all_edges(self):
        """Return list of all edges in env's graph."""
        return list(self.graph.edges())

    def find_neighbor_nodes(self, gps_location):
        """Find neighboring nodes given gps location.
        Returns a list of neighboring nodes. (sucessors?)"""
        # TODO Does this not return duplicate nodes for successors and predecessors? Doesn't this skip the actual closest node?
        location = ox.get_nearest_node(
            self.graph, gps_location, method='euclidean')
        return list(self.graph.successors(location)) + \
            list(self.graph.predecessors(location))

    def get_all_edges(self):
        """Return list of all edges in env's graph."""
        return list(self.graph.edges())
