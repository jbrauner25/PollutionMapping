from env import *
import numpy as np
import utils


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
		"""Get traffic of edge. Currently uses shortest edge if there are multiple edges that share origin/dest."""
		shortest_edge =  min(self.graph.get_edge_data(edge[0],edge[1]).values(),key=lambda x: x['length'])
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
			"""Update environment class with cart coord information. Update graph
			nodes. If no origin is previously set, it sets an origin."""
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

	'''NEW####################################################################################################'''




