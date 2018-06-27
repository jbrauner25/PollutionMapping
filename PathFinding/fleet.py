################################################################################
# Module: fleet.py
# Description: Manage fleets of vehicles.
# Copyright: Zenith Robotics Inc.
################################################################################

import random
from vehicle import Vehicle


class Fleet(object):
    """Fleet object to describe vehicle lists"""
    def __init__(self, num_vehicle):
        """Initialize num_vehicle vehicles."""
        self.vehicles = []
        for count in range(num_vehicle):
            self.vehicles.append(Vehicle(count))

    def init_locations(self, env, node_location_list=None):
        """Initialize each vehicle in a random location on env's graph, or if node_location_list given.
        Currently start = end node."""
        for i, vehicle in enumerate(self.vehicles):
            if node_location_list:
                node_location = node_location_list[i]
            else:
                node_location = random.choice(list(env.graph.nodes()))

            gps_location = env.node_to_utm(node_location)
            # TODO: utm
            vehicle.curr_node = node_location
            vehicle.end_node = node_location
            vehicle.set_location(gps_location)

    def advance_fleet(self, env, planner):
        """Advance vehicles in fleet."""
        for vehicle in self.vehicles:
            vehicle.advance_vehicle(env, planner)

    def check_fleet_done(self):
        """Check if all vehicles in fleet are done."""
        for vehicle in self.vehicles:
            if not vehicle.done:
                return False
        return True
