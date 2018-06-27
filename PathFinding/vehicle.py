##########################################################################
# Module: vehicle.py
# Description: Low level control and internal states of a vehicle.
# Copyright: Zenith Robotics Inc.
##########################################################################

from utils import random_distinct_colors, distance


class Vehicle(object):

    def __init__(self, vehicle_id, location=None, end_node=None):
        """Return a snowplow with a given GPS location.
        Other States to include later:
        salt_amount
        depot_location
        state (e.g. turning, on road)
        """
        self.vehicle_id = vehicle_id
        self.location = location
        self.end_node = end_node
        self.color = random_distinct_colors(self.vehicle_id)
        self.prev_nodes = []
        self.speed = 10  # m/s default for now
        self.next_nodes = []
        self.dist_from_last_point = 0
        self.curr_node = end_node
        self.assigned_edges = []
        self.curr_edge = None
        self.done = None

    def set_location(self, location):
        """Set vehicle to new node location."""
        self.location = location

    def get_location(self):
        """Get vehicle's current node location."""
        return self.location

    def update_state(self, node_location):
        """Update vehicle state based on node location (for now) and vehicle state."""
        self.prev_nodes.append(node_location)

    def advance_vehicle(self, env,planner):
        """Advances a vehicle based on its current node and next nodes. """
        # when not initialized

        if len(self.next_nodes) == 0:
            if env.calc_edges_left():
                print("case 1", self.vehicle_id)
                planner.init_next_nodes()
            else:
                print("vehicle donezosss!", self.vehicle_id)
                self.done = True
                pass
        # when vehicle is close to the next node
        elif distance(self.location, env.node_to_utm(self.next_nodes[0])) < (self.speed / env.get_edge_traffic(self.curr_edge)) :
            # put into plowed list
            print("case 2")

            # add edge to plowed after it is plowed
            if self.curr_node is not self.next_nodes[0]:
                env.add_edge_to_plowed_list(
                    (self.curr_node, self.next_nodes[0]))

            # set next node/pos as current node/pos
            new_utm_location = env.node_to_utm(self.next_nodes[0])
            self.curr_node = self.next_nodes[0]
            self.set_location(new_utm_location)
            self.dist_from_last_point = 0
            # advance list. TODO: change to queue/pop
            self.next_nodes = self.next_nodes[1:]
            self.update_state(self.curr_node)
            # TODO: update curr edge

        # if current and upcoming node are the same
        elif self.curr_node == self.next_nodes[0]:
            # print "case 3"
            self.next_nodes = self.next_nodes[1:]
            impedance_ratio = env.get_edge_traffic(self.curr_edge)# ratio to slow down vehicle speed because of traffic
            self.dist_from_last_point += self.speed / impedance_ratio
            if not self.next_nodes:
                planner.init_next_nodes()
                self.next_nodes[0], self.dist_from_last_point
            new_utm = env.interpolate_edge(
                self.curr_node,
                self.next_nodes[0],
                self.dist_from_last_point)
            new_utm_location = [new_utm[1][0], new_utm[0][0]]
            self.set_location(new_utm_location)
            # update current edge
            self.curr_edge = (self.curr_node,self.next_nodes[0])

        # If between two nodes, interpolate given vehicle speed
        else:
            # print "case 4"
            impedance_ratio = env.get_edge_traffic(self.curr_edge)# ratio to slow down vehicle speed because of traffic
            self.dist_from_last_point += self.speed / impedance_ratio


            new_utm = env.interpolate_edge(
                self.curr_node,
                self.next_nodes[0],
                self.dist_from_last_point)
            new_utm_location = [new_utm[1][0], new_utm[0][0]]
            self.set_location(new_utm_location)

            # update current edge
            self.curr_edge = (self.curr_node,self.next_nodes[0])
