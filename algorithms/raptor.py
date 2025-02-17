import math
from collections import defaultdict
import time
import sys
from typing import List

from data_structures.raptor_data_struct import CustomizedRAPTOR_DS
from helpers.utils import to_milliseconds
from helpers.binary_search import bisect_left
from helpers.path import Path


class CustomizedRAPTOR:
    def __init__(self, graph: CustomizedRAPTOR_DS, start_time: int, start_node: int, end_node: int, walking_speed: int,
                 max_rounds: int = sys.maxsize):
        """

        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.earliest_arrival_star = {}
        self.earliest_arrival = defaultdict(dict)
        self.earliest_arrival_seq = {}
        self.earliest_arrival_routes = {}
        self.trip_finder_dep_time = self.graph.trip_finder['dep_time_ut']
        self.trip_finder_trip = self.graph.trip_finder['trip_I']
        self.max_rounds = max_rounds
        self.source = start_node
        self.target = end_node
        self.walking_speed = walking_speed

        # Initialize the data structures
        self.earliest_arrival[0][start_node] = start_time  # Set the departure stop's time to the start time
        self.earliest_arrival_star[start_node] = start_time
        self.earliest_arrival_seq[start_node] = [start_node]
        self.earliest_arrival_routes[start_node] = []

    def departure_trip(self, route, stop, arrival_time):
        key = (route, stop)
        if key not in self.trip_finder_dep_time:
            return False
        departure_times = self.trip_finder_dep_time[key]
        start_index = bisect_left(departure_times, arrival_time)
        return self.trip_finder_trip[key][start_index] if start_index < len(departure_times) else False

    def shortest_path(self):
        start_time = time.monotonic()
        marked_stops = {self.source}
        new_marked_stops = set()
        for p_i in marked_stops:
            for node2, f in self.graph.walk_graph[p_i].items():
                walk_duration = f.w(walking_speed=self.walking_speed)
                new_arrival = self.earliest_arrival[0].get(p_i, math.inf) + walk_duration
                if new_arrival < self.earliest_arrival_star.get(node2, math.inf):
                    self.earliest_arrival[0][node2] = new_arrival
                    self.earliest_arrival_star[node2] = new_arrival
                    self.earliest_arrival_seq[node2] = self.earliest_arrival_seq[p_i] + [node2]
                    self.earliest_arrival_routes[node2] = self.earliest_arrival_routes[p_i] + ['walk']
                    new_marked_stops.add(node2)
        marked_stops = marked_stops.union(new_marked_stops)

        # Iterate through rounds
        for round_num in range(1, self.max_rounds + 1):
            # Initialize an empty dict for updates in this round
            Q = {}
            Q_routes = {}

            while marked_stops:
                p = marked_stops.pop()
                for route in self.graph.stop_to_route.get(p, []):
                    stp_idx_new = self.graph.idx_by_route_stop_dict.get((route, p), 10000000)

                    if route in Q_routes.keys():
                        stp_idx = self.graph.idx_by_route_stop_dict.get((route, Q_routes[route]), 10000000)
                        if stp_idx_new < stp_idx:
                            Q[(route, p)] = stp_idx_new
                            Q_routes[route] = p

                    else:
                        Q[(route, p)] = stp_idx_new
                        Q_routes[route] = p
            for (route, p), stp_idx in Q.items():

                current_trip_id = False
                for i in range(stp_idx, self.graph.routes_lens[route] + 1):
                    p_i = self.graph.stops_dict[(route, i - 1)]
                    p_i_arr = self.graph.stops_dict[(route, i)]
                    if current_trip_id:
                        arr_t = self.graph.trip_to_time.get((current_trip_id, i), math.inf)
                        if arr_t < min(self.earliest_arrival_star.get(p_i_arr, math.inf),
                                       self.earliest_arrival_star.get(self.target, math.inf)):
                            self.earliest_arrival[round_num][p_i_arr] = arr_t
                            self.earliest_arrival_seq[p_i_arr] = self.earliest_arrival_seq[p_i] + [p_i_arr]
                            self.earliest_arrival_star[p_i_arr] = arr_t
                            self.earliest_arrival_routes[p_i_arr] = (self.earliest_arrival_routes[p_i]
                                                                     + [self.graph.route_match[route]])
                            marked_stops.add(p_i_arr)
                    if current_trip_id is False:
                        dep_time = math.inf
                    else:
                        dep_time = self.graph.departure_times.get((current_trip_id, p_i_arr), math.inf)
                    if self.earliest_arrival[round_num - 1].get(p_i_arr, math.inf) <= dep_time:
                        current_trip_id = self.departure_trip(route, p_i_arr,
                                                              self.earliest_arrival[round_num - 1].get(p_i_arr,
                                                                                                       math.inf))

            new_marked_stops = set()
            for p_i in marked_stops:
                for node2, f in self.graph.walk_graph[p_i].items():

                    walk_duration = f.w(walking_speed=self.walking_speed)
                    new_arrival = self.earliest_arrival[round_num].get(p_i, math.inf) + walk_duration

                    if new_arrival < self.earliest_arrival_star.get(node2, math.inf):
                        self.earliest_arrival[round_num][node2] = new_arrival
                        self.earliest_arrival_star[node2] = new_arrival
                        self.earliest_arrival_seq[node2] = self.earliest_arrival_seq[p_i] + [node2]
                        self.earliest_arrival_routes[node2] = self.earliest_arrival_routes[p_i] + ['walk']
                        new_marked_stops.add(node2)
            marked_stops = marked_stops.union(new_marked_stops)
            if not new_marked_stops:
                break
        if self.target in self.earliest_arrival_star.keys():
            return {
                'path': self.earliest_arrival_seq[self.target],
                'routes': self.earliest_arrival_routes[self.target],
                'arrival': self.earliest_arrival_star[self.target],
                'duration': to_milliseconds(time.monotonic() - start_time)
            }
        else:
            return {
                'path': [],
                'routes': [],
                'arrival': math.inf,
                'duration': to_milliseconds(time.monotonic() - start_time)
            }


class McCustomizedRAPTOR:
    def __init__(self, graph: CustomizedRAPTOR_DS, start_time: int, start_node: int, end_node: int, walking_speed: int,
                 max_rounds: int = sys.maxsize):
        """

        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.earliest_arrival_star = {}
        self.earliest_arrival = defaultdict(dict)
        self.trip_finder_dep_time = self.graph.trip_finder['dep_time_ut']
        self.trip_finder_trip = self.graph.trip_finder['trip_I']
        self.max_rounds = max_rounds
        self.source = start_node
        self.target = end_node
        self.walking_speed = walking_speed

        # Initialize the data structures
        new_path = Path(sequence_nodes=[start_node],
                        sequence_route_names=[],
                        cost=(start_time, 0))
        self.earliest_arrival[0][start_node] = [new_path]
        self.earliest_arrival_star[start_node] = [new_path]

    def departure_trip(self, route, stop, arrival_time):
        key = (route, stop)
        if key not in self.trip_finder_dep_time:
            return False
        departure_times = self.trip_finder_dep_time[key]
        start_index = bisect_left(departure_times, arrival_time)
        return self.trip_finder_trip[key][start_index] if start_index < len(departure_times) else False

    @staticmethod
    def add_to_non_dominated_list(pareto_paths: List[Path], new_path: Path):
        """
        Adds a new (arrival_time, walking_duration) tuple to the list, ensuring that
        no tuple dominates another.

        :param current_list: List of existing tuples [(arrival_time, walking_duration), ...]
        :param new_tuple: Tuple (arrival_time, walking_duration) to be added
        :return: Updated list of tuples

        Parameters
        ----------
        pareto_paths
        """
        # Filter out tuples that are dominated by the new tuple
        updated_list = []

        for t in pareto_paths:
            # If the current tuple dominates the new tuple, discard the new tuple
            if t.cost[0] <= new_path.cost[0] and t.cost[1] <= new_path.cost[1]:
                return pareto_paths, False
            # If the new tuple dominates the current tuple, skip the current tuple
            if new_path.cost[0] <= t.cost[0] and new_path.cost[1] <= t.cost[1]:
                continue
            # Otherwise, keep the current tuple
            updated_list.append(t)

        # Add the new tuple since it is not dominated
        updated_list.append(new_path)

        return updated_list, True

    def shortest_path(self):
        start_time = time.monotonic()
        marked_stops = {self.source}
        new_marked_stops = set()

        for node2, f in self.graph.walk_graph[self.source].items():
            walk_duration = f.w(walking_speed=self.walking_speed)
            new_arrival = self.earliest_arrival[0].get(self.source)[0].cost[0] + walk_duration
            new_path = Path(sequence_nodes=[self.source] + [node2],
                            sequence_route_names=['walk'],
                            cost=(new_arrival, walk_duration))
            self.earliest_arrival_star[node2] = self.earliest_arrival[0][node2] = [new_path]
            new_marked_stops.add(node2)
        marked_stops = marked_stops.union(new_marked_stops)

        # Iterate through rounds
        for round_num in range(1, self.max_rounds + 1):
            # Initialize an empty dict for updates in this round
            Q = {}
            Q_routes = {}

            while marked_stops:
                p = marked_stops.pop()
                for route in self.graph.stop_to_route.get(p, []):
                    Q[(route, p)] = self.graph.idx_by_route_stop_dict.get((route, p), 0)
                    Q_routes[route] = p
            for (route, p), stp_idx in Q.items():
                current_trip_ids = dict()
                for i in range(stp_idx, self.graph.routes_lens[route] + 1):
                    p_i_arr = self.graph.stops_dict[(route, i)]
                    current_trip_ids_new = dict()
                    for path, current_trip_id in current_trip_ids.items():

                        arr_t = self.graph.trip_to_time.get((current_trip_id, i))

                        if arr_t is not None:
                            new_path = Path(sequence_nodes=path.sequence_nodes + [p_i_arr],
                                            sequence_route_names=(path.sequence_route_names
                                                                  + [self.graph.route_match[route]]),
                                            cost=(arr_t, path.cost[1]))

                            self.earliest_arrival[round_num][p_i_arr], add_new_element = self.add_to_non_dominated_list(
                                self.earliest_arrival_star.get(p_i_arr, []), new_path)

                            _, add_new_element_target = self.add_to_non_dominated_list(
                                self.earliest_arrival_star.get(self.target, []), new_path)

                            if add_new_element and add_new_element_target:
                                current_trip_ids_new[new_path] = current_trip_id
                                self.earliest_arrival_star[p_i_arr] = self.earliest_arrival[round_num][p_i_arr]
                                marked_stops.add(p_i_arr)
                    paths = self.earliest_arrival[round_num - 1].get(p_i_arr, [])
                    for path in paths:
                        current_trip_id = self.departure_trip(route, p_i_arr, path.cost[0])

                        if current_trip_id is not False:
                            current_trip_ids_new[path] = current_trip_id

                    current_trip_ids = current_trip_ids_new.copy()

            new_marked_stops = set()
            for p_i in marked_stops:
                for node2, f in self.graph.walk_graph[p_i].items():
                    walk_duration = f.w(walking_speed=self.walking_speed)
                    for path in self.earliest_arrival[round_num].get(p_i, []):

                        new_arrival = path.cost[0] + walk_duration

                        new_path = Path(sequence_nodes=path.sequence_nodes + [node2],
                                        sequence_route_names=(path.sequence_route_names + ['walk']),
                                        cost=(new_arrival, path.cost[1] + walk_duration))

                        self.earliest_arrival[round_num][node2], add_new_element = self.add_to_non_dominated_list(
                            self.earliest_arrival_star.get(node2, []), new_path)

                        if add_new_element:
                            self.earliest_arrival_star[node2] = self.earliest_arrival[round_num][node2]
                            new_marked_stops.add(node2)
            marked_stops = marked_stops.union(new_marked_stops)
            if not marked_stops:
                break
        if self.target in self.earliest_arrival_star.keys():
            return {
                'paths': self.earliest_arrival_star[self.target],
                'duration': to_milliseconds(time.monotonic() - start_time)
            }
        else:
            return {
                'paths': [],
                'duration': to_milliseconds(time.monotonic() - start_time)
            }
