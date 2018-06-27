import osmnx as ox, networkx as nx, matplotlib.cm as cm, pandas as pd, numpy as np
from collections import defaultdict, deque
import sys
# ox.config(log_file=True, log_console=True, use_cache=True)
import math as m, random, csv
earthRadius = 6.371e6

def load_graph():
	while True:
		text = input('Import a graph? (y|n)')
		if text=='y' or text =='Y' or text == "Yes" or text == "yes":
			path = input('Enter graph path')
			graph = ox.graph_from_file(path, network_type='drive', simplify=True, retain_all=True, name='unnamed')
			G_projected = ox.project_graph(graph)
			ox.plot_graph(G_projected)
			return graph
		text = input("Create new graph? (y|n)")
		if text == 'y' or text == 'Y' or text == "Yes" or text == "yes":
			while True:
				# west = float(input("Top left coordinate latitude (west bounding)"))
				# north = float(input("Top left coordinate longitude (north bounding)"))
				# east = float(input("Bottom right coordinate latitude (east bounding)"))
				# south = float(input("Bottom right coordinate longitude (south bounding)"))

				north, south, east, west = 37.79, 37.78, -122.41, -122.43
				graph = ox.graph_from_bbox(north, south, east, west, network_type='drive',
											truncate_by_edge=True, simplify=True)
				# G_projected = ox.project_graph(graph)
				# ox.plot_graph(G_projected)
				text = input("Does this look correct?")
				if text == 'y' or text == 'Y' or text == "Yes" or text == "yes":
					text = str(input("Name the graph file"))
					ox.save_graphml(graph, filename=text + '.graphml')
					return graph

def get_lat_long_coords(graph):
	'''returns list of tuples giving latitude, longitude pair'''
	return [(graph.node[n]['y'], graph.node[n]['x']) for n in graph.nodes()]

def set_cart_coords(graph, originCoord = None):
	'''Converts latitude longitude coordinates to cartesian coordinates (in meters) based off their distance from
	the origin. If the origin coordinate is none, chooses the southernmost node in the graph to be the origin'''
	print(graph.nodes())
	if originCoord:
		originID = None
		origin = originCoord
	else:
		latCoords = [coord[0] for coord in get_lat_long_coords(graph)]  # For finding the min/origin
		index = latCoords.index(min(latCoords)) #Finds southmost node because Northern hemispere has latitude coordinates decreasing towards equator.
		originID = list(graph.nodes())[index]
		origin = (graph.node[originID]['y'], graph.node[originID]['x'])

	print("Origin set at:", origin)
	cartesianCoords = {}
	originDict = {}
	print(graph.nodes())
	for n in graph.nodes():
		# originDict.update(({n, origin}))
		if n == originID:  # Sets (0, 0) meters as value of origin node.
			cartesianCoords.update({n: (0.0, 0.0)})
		else:
			xval = (graph.node[n]['x'] - origin[1])*(np.pi/180)*np.cos((graph.node[n]['y'] - origin[0])/2*np.pi/180)*earthRadius
			yval = (graph.node[n]['y'] - origin[0])*np.pi/180*earthRadius
			cartesianCoords.update({n: (xval, yval)})

	nx.set_node_attributes(graph, cartesianCoords, 'cartesian_coords')
	nx.set_node_attributes(graph, originDict, 'origin')
	return graph

load_graph()