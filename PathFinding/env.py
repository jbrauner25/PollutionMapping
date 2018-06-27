# Adapted for Pol Mapping

##########################################################################
# Module: env.py (Simulation Environment Class)
# Description: Create road network environment and plot to png
# Copyright: Zenith Robotics Inc.
##########################################################################

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
    def __init__(self, unproj_graph, fleet, node_location_list=None):
        """Return an Environment."""
        self.graph = ox.project_graph(unproj_graph)
        self.G = copy.deepcopy(unproj_graph)
        self.nodes = unproj_graph.nodes()
        self.fleet = fleet
        fleet.init_locations(self, node_location_list)
        self.vehicles = self.fleet.vehicles
        self.traffic_dict = {} # TODO: delete
        self.plowed_list = []

    def advance_fleet_and_plot(self, planner, i, plot_freq):
        """Advance fleet by one step and then plot."""
        # Plotting constants
        filename_prefix='temp'

        congest_limit = 20 # If impedance above or equal this number, an edge is congested
        # Find list of congested edge
        congested_list = [edge for edge in self.graph.edges() if min(self.graph.get_edge_data(edge[0],edge[1]).values(), key=lambda x:x['length'])['traffic'] >= 20]

        # Advance Fleet
        self.fleet.advance_fleet(self, planner)

        # Determine metrics
        overlap = self.get_overlaps()
        edges_left = self.calc_edges_left()

        # Graph
        if i % plot_freq == 0:  # Show every 10 timesteps
            # outputs as images/*.png
            self.plot_env(
                i,
                filename_prefix=filename_prefix,
                overlaps=overlap,
                edges_left=edges_left,
                plowed_edge_list=self.plowed_list,
                traffic_list=congested_list,
                show=False,
                debug_label=False,
                plot_folium=False)

        return edges_left, overlap


    def get_plowed_list(self):
        """Return self.plowed_list."""
        return self.plowed_list

    def add_edge_to_plowed_list(self, edge):
        """Add edge to plowed list. TODO: also return num_overlaps."""
        self.plowed_list.append(edge)

    def calc_edges_left(self):
        return len(self.get_all_edges()) - len(set(self.plowed_list))

    def get_overlaps(self):
        """Get number of overlaps."""
        return self.calc_overlaps_in_list(self.plowed_list)

    @staticmethod
    def calc_overlaps_in_list(edge_list):
        """Calculate number of overlaps in edge_list."""
        return len(edge_list) - len(set(edge_list))

    def update_traffic_from_source(self, source):
        """Update environment class with traffic information. Update graph
        edges. Source is a dictionary of edges with traffic as key."""
        self.traffic_dict = source
        for u,v,data in self.graph.edges(keys=False, data=True):
            traffic_edge = self.traffic_dict[(u,v)]
            data['traffic'] = traffic_edge
            data['impedance'] = traffic_edge * self.get_edge_length((u,v))


    def get_edge_traffic(self, edge):
        """Get traffic of edge. Currently uses shortest edge if there are multiple edges that share origin/dest."""
        shortest_edge =  min(self.graph.get_edge_data(edge[0],edge[1]).values(),key=lambda x: x['length'])
        return shortest_edge['traffic']

    def get_edge_length(self, edge):
        """Get length of edge. Currently uses shortest edge if there are multiple edges that share origin/dest."""
        shortest_edge =  min(self.graph.get_edge_data(edge[0],edge[1]).values(),key=lambda x: x['length'])
        return shortest_edge['length']

    def node_to_utm(self, node_location):
        """Returns lat long coordinates given node location of graph"""
        node = self.graph.nodes()[node_location]
        return (node['y'], node['x'])

    def route_to_gps(self, nodes):
        """ Returns lat-long given an array of nodes """

        coord = []

        # loop through all nodes
        for node in nodes:
            utm_coord = self.node_to_utm(node)
            coord.append(node)
            # # set up pyproj
            # myProj = Proj("+proj=utm +zone=13s, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
            # lon, lat = myProj(utm_coord[0], utm_coord[1], inverse=True)
            # coord.append([lon, lat])

        return coord

    def interpolate_edge(self, origin, dest, distance):
        """Interpolate UTM coordinates between origin and dest node
        given distance (m)."""
        geometry = self.get_edge_geometry(origin, dest)
        return geometry.interpolate(distance).xy

    def get_path_traffic(self, path):
        """Get impedance of each edge of path due to traffic."""
        return ox.get_route_edge_attributes(self.graph, path, 'traffic')

    def get_path_length(self, path):
        """Get length of each edge of path. (meters)"""
        return ox.get_route_edge_attributes(self.graph, path, 'length')

    def get_edge_geometry(self, origin, dest):
        """Given origin and dest nodes (neighboring nodes), return
        shapely linestring object of the edge geometry.
        Currently, uses edge of min length. TODO: change to multiple edges."""
        try:
            shortest_edge = min(
                self.graph.get_edge_data(
                    origin,
                    dest).values(),
                key=lambda x: x['length'])
        except BaseException:
            print("need to go back!!")
            shortest_edge = min(
                self.graph.get_edge_data(
                    dest,
                    origin).values(),
                key=lambda x: x['length'])
        if 'geometry' in shortest_edge.keys():  # if geometry exist in edge
            return shortest_edge['geometry']
        else:  # if not return a straight line
            return LineString([(self.graph.nodes()[origin]['x'], self.graph.nodes()[origin]['y']),
                               (self.graph.nodes()[dest]['x'], self.graph.nodes()[dest]['y'])])

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

    def get_shortest_path(self, origin_node, dest_node, weight=None):
        """find list of nodes that make up shortest path between origin_node
        and dest_node
        TODO: change weighting"""
        return nx.shortest_path(
            self.graph,
            origin_node,
            dest_node,
            weight=weight)

    def move_vehicles_back(self):
        """Send vehicles to end node."""
        print("move vehicle back")
        for vehicle in self.vehicles:
            shortest_path = nx.shortest_path(self.graph,vehicle.curr_node,vehicle.end_node,'impedance')
            # if vehicle is already at end node or not path exists, do nothing
            if len(shortest_path) <= 1:
                vehicle.next_nodes = []
            else:
                print(vehicle.curr_node, vehicle.end_node, shortest_path)
                vehicle.curr_edge = (shortest_path[0], shortest_path[1])
                vehicle.next_nodes =  shortest_path[1:]

    def get_all_edges(self):
        """Return list of all edges in env's graph."""
        return list(self.graph.edges())

    def plot_env(self, i, filename_prefix, overlaps, edges_left,
        plowed_edge_list, traffic_list=[], show=True, debug_label=False,
        plot_folium=False):
        """Plots environment (incl. map, vehicles, planned routes).
        Args:
        i: int, current timestep, used for save to png,
        filename_prefix: str, prefix to png filename, used for savetopng
        show: boolean, if true, show the figure. (need to manually close each step)
        overlaps: int, current number of overlaps
        edges_left: int, current number of edges left to be plowed
        plowed_edge_list: list of edges, list of plowed edges
                          ex. [(100, 101), (102,101)].
                          black dashed line when one-side is plowed, solid black
                          when both sides are plowed
        traffic_list: list of edges, list of congested edges
                      ex. [(100, 101), (102,101)].
                      displayed as solid red line
        show: bool, if true, shows plot in matplotlib window. need to close at every step.
        debug_label: bool, if true, displays node number and vehicle number
        plot_folium: bool, if true, plots folium map in data/*.html
        """
        lists_of_routes = [vehicle.prev_nodes + \
                           vehicle.next_nodes for vehicle in self.vehicles]
        vehicle_color_list = [vehicle.color for vehicle in self.vehicles]
        vehicle_loc_list = [vehicle.location for vehicle in self.vehicles]
        self.plot_graph_routes(self.graph,
                             lists_of_routes,
                             vehicle_loc_list,
                             self.vehicles,
                             plowed_edge_list,
                             traffic_list,
                             show=show,
                             route_color_list=vehicle_color_list,
                             file_format='png',
                             filename=filename_prefix + str('%04d' % i),
                             debug_label=debug_label,
                             overlaps=overlaps,
                             edges_left=edges_left,
                             time_step=i)

        # added by Vai for folium TODO
        # convert lists of routes to utm
        route = self.route_to_gps(lists_of_routes[0])
        # TODO fiX
        # route = [53090322, 53090323, 53036558, 317339074, 53122757, 53023513, 683050219, 683050221, 53021750, 53085380, 53027457, 53027455, 53040495, 53033660, 432431093]

        if plot_folium:
            print("Route:")
            print(route)
            print("Graph:")
            print(self.nodes)
            route_folium = ox.plot_route_folium(self.G, route)

            filepath = 'data/route.html'
            route_folium.save(filepath)

            graph_folium = ox.plot_graph_folium(self.G)

            filepath = 'data/graph.html'
            graph_folium.save(filepath)

    def plot_graph_routes(self, G, routes_list, vehicle_loc_list, vehicle_list,
                        plowed_edge_list=[], traffic_list=[], show=True, save=True,
                        file_format='png', filename='temp', route_color_list=['r'],
                        debug_label=False, overlaps=0, edges_left=0, time_step=0):
        """
        Plot upcoming routes for all vehicles, vehicle locations, plowed edges along a networkx
        spatial graph.

        Adapted from OSMNX.

        Parameters
        ----------
        G : networkx multidigraph
        routes_list : list
            list of routes where the route are a list of nodes. In the same order as
            vehicles in vehicle list.
        vehicle_loc_list : list
            list of UTM location of vehicles. In the same order as
            vehicles in vehicle list.
        vehicle_list : list
            list of vehicles
        plowed_edge_list : list
            list of plowed edges
        traffic_list : list
            list of congested edges
        show : bool
            if True, show the figure
        save : bool
            if True, save the figure as an image file to disk
        file_format : string
            the format of the file to save (e.g., 'jpg', 'png', 'svg')
        filename : string
            the name of the file if saving
        route_color_list : list
            list of colors for vehicle routes, in the order of vehicle list
        debug_label : bool
            if true, annotate nodes and vehicles
        overlaps : int
            current number of overlaps
        edges_left : int
            current number of edges left to be plowed
        time_step : int
            current time step
        Returns
        -------
        fig, ax : tuple
        """

        # constants
        bbox = None
        fig_height = 6
        fig_width = None
        margin = 0.02
        bgcolor = 'w'
        axis_off = True
        dpi = 300
        annotate = False
        node_size=15
        node_alpha=1
        node_edgecolor='none'
        node_zorder=1
        edge_color='#999999'
        edge_linewidth=1
        edge_alpha=1
        veh_edgecolor='k'
        orig_dest_node_alpha=0.5
        orig_dest_node_size=100
        route_linewidth=4
        route_alpha=0.25
        use_geom=True
        close=True


        # get top left corner for displaying. TODO: efficient?
        xy = [(G.nodes()[node]['x'], G.nodes()[node]['y'])
              for node in list(G.nodes())]
        min_x = min(xy, key=lambda x: x[0])[0]
        max_y = max(xy, key=lambda x: x[1])[1]
        min_y = min(xy, key=lambda x: x[1])[1]

        # img = plt.imread("denver_zoo.png")
        # plot the graph but not the route
        fig, ax = self.plot_base_graph(G, bbox=bbox, fig_height=fig_height, fig_width=fig_width,
                             margin=margin, axis_off=axis_off, bgcolor=bgcolor,
                             show=False, save=False, close=False, filename=filename,
                             dpi=dpi, annotate=annotate, node_color='#999999',
                             node_size=node_size, node_alpha=node_alpha,
                             node_edgecolor=node_edgecolor, node_zorder=node_zorder,
                             edge_color=edge_color, edge_linewidth=edge_linewidth,
                             edge_alpha=edge_alpha, use_geom=use_geom, debug_label=debug_label)

        # display metrics (overlaps, edges left)
        time = int(time_step)/15
        plt.text(min_x, max_y - (max_y - min_y) / 70, 'Time (min): %d' %
                 time, fontsize=10)  # Current time step
        spacing = ((max_y - min_y) / 20) - ((max_y - min_y) / 70)
        plt.text(min_x, max_y - (max_y - min_y) / 70 - spacing,
                 'Overlaps: %d' % overlaps, fontsize=10)  # overlaps
        plt.text(min_x, max_y - (max_y - min_y) / 70 - spacing - spacing,
                 'Edges left: %d' % edges_left, fontsize=10)  # edges left

        # label vehicles
        if debug_label:
            vehicle_id = 0
            for a, b in zip([veh_loc[1] for veh_loc in vehicle_loc_list], [
                            veh_loc[0] for veh_loc in vehicle_loc_list]):
                plt.text(
                    a,
                    b,
                    str(vehicle_id),
                    fontsize=7,
                    color='red',
                    horizontalalignment='left')
                vehicle_id += 1
        for counter, vehicle in enumerate(vehicle_list):

            dest_lats = vehicle_loc_list[counter][0]
            dest_lons = vehicle_loc_list[counter][1]
            # scatter the current vehicle points
            ax.scatter(
                dest_lons,
                dest_lats,
                s=node_size,
                c=route_color_list[counter],
                alpha=node_alpha,
                edgecolor=veh_edgecolor,
                zorder=4)
            ax.scatter(
                dest_lons,
                dest_lats,
                s=orig_dest_node_size,
                c=route_color_list[counter],
                alpha=orig_dest_node_alpha,
                edgecolor=node_edgecolor,
                zorder=4)

            # plot routes
            upcoming_lines = []

            # plot the route lines on graph
            upcoming_lines = self.plot_route(G, [vehicle.curr_node]+vehicle.next_nodes, upcoming_lines)


            # add the lines to the axis as a linecollection
            upcoming_lc = LineCollection(
                upcoming_lines,
                colors=route_color_list[counter],
                linewidths=route_linewidth,
                alpha=0.5,
                zorder=3)
            ax.add_collection(upcoming_lc)

        # Show traffic as red
        for assigned_line in traffic_list:
            assigned_lines = []
            assigned_lines = self.plot_route(G, assigned_line, assigned_lines)
            assigned_lc = LineCollection(assigned_lines,
                        colors='r',
                        linewidths=route_linewidth,
                        alpha=1,
                        zorder=3)
            ax.add_collection(assigned_lc)



        # plot plowed lines on top
        for edge_num, plowed_edge in enumerate(set(plowed_edge_list)):
            plowed_lines = []
            plowed_lines = self.plot_route(G, list(plowed_edge), plowed_lines)
            # if both sides are plowed, black line
            if (plowed_edge[1],plowed_edge[0]) in list(set(plowed_edge_list))[: edge_num+1] and (plowed_edge[1],plowed_edge[0]) in list(G.edges()):
                plowed_lc = LineCollection(plowed_lines, colors='k', linewidths=route_linewidth, alpha=1, zorder=3)
            # if this is first side plowed, red line
            else:
                plowed_lc = LineCollection(plowed_lines, colors='k', linestyle='--', linewidths=route_linewidth, alpha=1, zorder=3)
            ax.add_collection(plowed_lc)
        # save and show the figure as specified
        fig, ax = ox.save_and_show(fig, ax, save, show, close,
                                filename, file_format, dpi, axis_off)
        return fig, ax

    def plot_route(self, G, route, lines):
        """Get plottable lines for plot route functions."""
        # plot the route lines
        edge_nodes = list(zip(route[:-1], route[1:]))
        for u, v in edge_nodes:
            try:
                data = min(
                    G.get_edge_data(
                        u,
                        v).values(),
                    key=lambda x: x['length'])
            except BaseException:
                data = {}

            # if it has a geometry attribute (ie, a list of line segments)
            if 'geometry' in data:
                # add them to the list of lines to plot
                xs, ys = data['geometry'].xy
                lines.append(list(zip(xs, ys)))
            else:
                # if it doesn't have a geometry attribute, the edge is a straight
                # line from node to node
                x1 = G.nodes[u]['x']
                y1 = G.nodes[u]['y']
                x2 = G.nodes[v]['x']
                y2 = G.nodes[v]['y']
                line = [(x1, y1), (x2, y2)]
                lines.append(line)
        return lines

    def plot_base_graph(self, G, bbox=None, fig_height=6, fig_width=None, margin=0.02,
                   axis_off=True, equal_aspect=False, bgcolor='w', show=True,
                   save=False, close=True, file_format='png', filename='temp',
                   dpi=300, annotate=False, node_color='#66ccff', node_size=15,
                   node_alpha=1, node_edgecolor='none', node_zorder=1,
                   edge_color='#999999', edge_linewidth=1, edge_alpha=1,
                   use_geom=True, debug_label=False):
        """
        Plot a base networkx spatial graph with appropriate geometry and locations.

        Parameters
        ----------
        G : networkx multidigraph
        bbox : tuple
            bounding box as north,south,east,west - if None will calculate from
            spatial extents of data
        fig_height : int
            matplotlib figure height in inches
        fig_width : int
            matplotlib figure width in inches
        margin : float
            relative margin around the figure
        axis_off : bool
            if True turn off the matplotlib axis
        equal_aspect : bool
            if True set the axis aspect ratio equal
        bgcolor : string
            the background color of the figure and axis
        show : bool
            if True, show the figure
        save : bool
            if True, save the figure as an image file to disk
        close : bool
            close the figure (only if show equals False) to prevent display
        file_format : string
            the format of the file to save (e.g., 'jpg', 'png', 'svg')
        filename : string
            the name of the file if saving
        dpi : int
            the resolution of the image file if saving
        annotate : bool
            if True, annotate the nodes in the figure
        node_color : string
            the color of the nodes
        node_size : int
            the size of the nodes
        node_alpha : float
            the opacity of the nodes
        node_edgecolor : string
            the color of the node's marker's border
        node_zorder : int
            zorder to plot nodes, edges are always 2, so make node_zorder 1 to plot
            nodes beneath them or 3 to plot nodes atop them
        edge_color : string
            the color of the edges' lines
        edge_linewidth : float
            the width of the edges' lines
        edge_alpha : float
            the opacity of the edges' lines
        use_geom : bool
            if True, use the spatial geometry attribute of the edges to draw
            geographically accurate edges, rather than just lines straight from node
            to node

        Returns
        -------
        fig, ax : tuple
        """

        # log('Begin plotting the graph...')
        node_Xs = [float(x) for _, x in G.nodes(data='x')]
        node_Ys = [float(y) for _, y in G.nodes(data='y')]

        # get north, south, east, west values either from bbox parameter or from the
        # spatial extent of the edges' geometries
        if bbox is None:
            edges = ox.graph_to_gdfs(G, nodes=False, fill_edge_geometry=True)
            west, south, east, north = edges.total_bounds
        else:
            north, south, east, west = bbox

        # if caller did not pass in a fig_width, calculate it proportionately from
        # the fig_height and bounding box aspect ratio
        bbox_aspect_ratio = (north - south) / (east - west)
        if fig_width is None:
            fig_width = fig_height / bbox_aspect_ratio

        # create the figure and axis
        fig, ax = plt.subplots(figsize=(fig_width, fig_height), facecolor=bgcolor)
        ax.set_facecolor(bgcolor)

        # draw the edges as lines from node to node
        start_time = time.time()
        lines = []
        for u, v, data in G.edges(keys=False, data=True):
            if 'geometry' in data and use_geom:
                # if it has a geometry attribute (a list of line segments), add them
                # to the list of lines to plot
                xs, ys = data['geometry'].xy
                lines.append(list(zip(xs, ys)))
            else:
                # if it doesn't have a geometry attribute, the edge is a straight
                # line from node to node
                x1 = G.nodes[u]['x']
                y1 = G.nodes[u]['y']
                x2 = G.nodes[v]['x']
                y2 = G.nodes[v]['y']
                line = [(x1, y1), (x2, y2)]
                lines.append(line)

        # add the lines to the axis as a linecollection
        lc = LineCollection(
            lines,
            colors=edge_color,
            linewidths=edge_linewidth,
            alpha=edge_alpha,
            zorder=2)
        ax.add_collection(lc)

        # scatter plot the nodes
        ax.scatter(
            node_Xs,
            node_Ys,
            s=node_size,
            c=node_color,
            alpha=node_alpha,
            edgecolor=node_edgecolor,
            zorder=node_zorder)

        # label nodes
        if debug_label:
            label_count = 0
            for a, b in zip(node_Xs, node_Ys):
                plt.text(a, b, str(list(G.nodes())[label_count]), fontsize=3)
                label_count += 1

        # set the extent of the figure
        margin_ns = (north - south) * margin
        margin_ew = (east - west) * margin
        ax.set_ylim((south - margin_ns, north + margin_ns))
        ax.set_xlim((west - margin_ew, east + margin_ew))

        # configure axis appearance
        ax.get_xaxis().get_major_formatter().set_useOffset(False)
        ax.get_yaxis().get_major_formatter().set_useOffset(False)

        # if axis_off, turn off the axis display set the margins to zero and point
        # the ticks in so there's no space around the plot
        if axis_off:
            ax.axis('off')
            ax.margins(0)
            ax.tick_params(which='both', direction='in')
            fig.canvas.draw()

        if equal_aspect:
            # make everything square
            ax.set_aspect('equal')
            fig.canvas.draw()

        # annotate the axis with node IDs if annotate=True
        if annotate:
            for node, data in G.nodes(data=True):
                ax.annotate(node, xy=(data['x'], data['y']))

        # save and show the figure as specified
        fig, ax = ox.save_and_show(fig, ax, save, show, close,
                                filename, file_format, dpi, axis_off)
        return fig, ax
