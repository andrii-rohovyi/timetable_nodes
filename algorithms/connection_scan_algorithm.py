import math
from helpers.binary_search import bisect_left
import logging
import time
from typing import List

from data_structures.timetable import Timetable, CustomizedTimetable
from helpers.utils import to_milliseconds
from helpers.path import Path


class ConnectionScanAlgorithm:
    def __init__(self, graph: Timetable, start_time: int, start_node: int, end_node: int):
        """
        Implementation of the connection scan algorithm. Link on the paper: https://arxiv.org/pdf/1703.05997
        Parameters
        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.start_time = start_time
        self.s = {start_node: start_time}
        self.routes = {start_node: []}
        self.path = {start_node: [start_node]}

    def shortest_path(self):
        """
        Finding shortest path with CSA algorithm Figure 4 in paper.
        Returns
        -------

        """
        start_time = time.monotonic()
        for node, f in self.graph.walk_graph[self.source].items():
            self.s[node] = self.start_time + f
            self.routes[node] = ['walk']
            self.path[node] = [self.source, node]

        index = bisect_left(self.graph.departure_times, self.start_time)
        row = self.graph.transport_connections_df[index]
        zero_len_old = False
        zero_len_first_index = 0

        while True:
            reiterate = False

            if self.s.get(self.target, math.inf) < row['dep_time_ut']:
                return {
                    'path': self.path[self.target],
                    'routes': self.routes[self.target],
                    'arrival': self.s[self.target],
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }

            if self.s.get(row['from_stop_I'], math.inf) <= row['dep_time_ut']:
                arrival_to_stop = self.s.get(row['to_stop_I'], math.inf)
                if row['arr_time_ut'] < arrival_to_stop:
                    if row['arr_time_ut'] == row['dep_time_ut']:
                        reiterate = True
                    self.s[row['to_stop_I']] = row['arr_time_ut']
                    self.routes[row['to_stop_I']] = self.routes[row['from_stop_I']] + [row['route_I']]
                    self.path[row['to_stop_I']] = self.path[row['from_stop_I']] + [row['to_stop_I']]

                    for node, f in self.graph.walk_graph[row['to_stop_I']].items():
                        walk_arr = row['arr_time_ut'] + f
                        if walk_arr < self.s.get(node, math.inf):
                            self.s[node] = walk_arr
                            self.routes[node] = self.routes[row['to_stop_I']] + ['walk']
                            self.path[node] = self.path[row['to_stop_I']] + [node]

            zero_len = row['arr_time_ut'] == row['dep_time_ut']
            if (zero_len) & (not zero_len_old):
                zero_len_first_index = index
            zero_len_old = zero_len

            if reiterate:
                index = zero_len_first_index
            else:
                index += 1

            if index < len(self.graph.transport_connections_df):
                row = self.graph.transport_connections_df[index]
            else:
                if self.s.get(self.target):
                    return {
                        'path': self.path[self.target],
                        'routes': self.routes[self.target],
                        'arrival': self.s[self.target],
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
                else:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }


class CustomizedConnectionScanAlgorithmBicycle:
    def __init__(self, graph: CustomizedTimetable, start_time: int, start_node: int, end_node: int, walking_speed: int,
                 bicycle_speed: int, ):
        """
        Implementation of the connection scan algorithm. Link on the paper: https://arxiv.org/pdf/1703.05997
        Parameters
        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.start_time = start_time
        self.walking_speed = walking_speed
        self.bicycle_speed = bicycle_speed
        self.s = {start_node: start_time}
        self.s_bus = {start_node: start_time}
        self.routes = {start_node: []}
        self.path = {start_node: [start_node]}

    def shortest_path(self):
        """
        Finding shortest path with CSA algorithm Figure 4 in paper.
        Returns
        -------

        """
        start_time = time.monotonic()
        for node, f in self.graph.transfer_graph[self.source].items():
            self.s[node] = self.start_time + f.w(walk_speed=self.walking_speed,
                                                 bicycle_speed=self.bicycle_speed)
            self.routes[node] = f.route_names
            self.path[node] = f.nodes

        index = bisect_left(self.graph.departure_times, self.start_time)
        row = self.graph.transport_connections_df[index]
        zero_len_old = False
        zero_len_first_index = 0

        while True:
            reiterate = False

            if self.s.get(self.target, math.inf) < row['dep_time_ut']:
                return {
                    'path': self.path[self.target],
                    'routes': self.routes[self.target],
                    'arrival': self.s[self.target],
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }
            if self.s.get(row['from_stop_I'], math.inf) <= row['dep_time_ut']:
                arrival_to_stop = self.s_bus.get(row['to_stop_I'], math.inf)

                if row['arr_time_ut'] < arrival_to_stop:

                    if row['arr_time_ut'] == row['dep_time_ut']:
                        reiterate = True
                    self.s_bus[row['to_stop_I']] = row['arr_time_ut']
                    if self.s.get(row['to_stop_I'], math.inf) > row['arr_time_ut']:
                        self.s[row['to_stop_I']] = row['arr_time_ut']
                    self.routes[row['to_stop_I']] = self.routes[row['from_stop_I']] + [row['route_I']]
                    self.path[row['to_stop_I']] = self.path[row['from_stop_I']] + [row['to_stop_I']]

                    for node, f in self.graph.transfer_graph[row['to_stop_I']].items():
                        walk_arr = row['arr_time_ut'] + f.w(walk_speed=self.walking_speed,
                                                            bicycle_speed=self.bicycle_speed)
                        if walk_arr < self.s.get(node, math.inf):
                            self.s[node] = walk_arr
                            self.routes[node] = self.routes[row['to_stop_I']] + f.route_names
                            self.path[node] = self.path[row['to_stop_I']] + f.nodes[1:]

            zero_len = row['arr_time_ut'] == row['dep_time_ut']
            if (zero_len) & (not zero_len_old):
                zero_len_first_index = index
            zero_len_old = zero_len

            if reiterate:
                index = zero_len_first_index
            else:
                index += 1

            if index < len(self.graph.transport_connections_df):
                row = self.graph.transport_connections_df[index]
            else:
                if self.s.get(self.target):
                    return {
                        'path': self.path[self.target],
                        'routes': self.routes[self.target],
                        'arrival': self.s[self.target],
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
                else:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }


class CustomizedConnectionScanAlgorithm:
    def __init__(self, graph: CustomizedTimetable, start_time: int, start_node: int, end_node: int, walking_speed: int,
                 ):
        """
        Implementation of the connection scan algorithm. Link on the paper: https://arxiv.org/pdf/1703.05997
        Parameters
        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.start_time = start_time
        self.walking_speed = walking_speed
        self.s = {start_node: start_time}
        self.routes = {start_node: []}
        self.path = {start_node: [start_node]}

    def shortest_path(self):
        """
        Finding shortest path with CSA algorithm Figure 4 in paper.
        Returns
        -------

        """
        start_time = time.monotonic()
        for node, f in self.graph.walk_graph[self.source].items():
            self.s[node] = self.start_time + f.w(walking_speed=self.walking_speed)
            self.routes[node] = f.route_names
            self.path[node] = f.nodes

        index = bisect_left(self.graph.departure_times, self.start_time)
        row = self.graph.transport_connections_df[index]
        zero_len_old = False
        zero_len_first_index = 0

        while True:
            reiterate = False

            if self.s.get(self.target, math.inf) < row['dep_time_ut']:
                return {
                    'path': self.path[self.target],
                    'routes': self.routes[self.target],
                    'arrival': self.s[self.target],
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }
            if self.s.get(row['from_stop_I'], math.inf) <= row['dep_time_ut']:
                arrival_to_stop = self.s.get(row['to_stop_I'], math.inf)

                if row['arr_time_ut'] < arrival_to_stop:

                    if row['arr_time_ut'] == row['dep_time_ut']:
                        reiterate = True
                    self.s[row['to_stop_I']] = row['arr_time_ut']
                    self.routes[row['to_stop_I']] = self.routes[row['from_stop_I']] + [row['route_I']]
                    self.path[row['to_stop_I']] = self.path[row['from_stop_I']] + [row['to_stop_I']]

                    for node, f in self.graph.walk_graph[row['to_stop_I']].items():
                        walk_arr = row['arr_time_ut'] + f.w(walking_speed=self.walking_speed)
                        if walk_arr < self.s.get(node, math.inf):
                            self.s[node] = walk_arr
                            self.routes[node] = self.routes[row['to_stop_I']] + f.route_names
                            self.path[node] = self.path[row['to_stop_I']] + f.nodes[1:]

            zero_len = row['arr_time_ut'] == row['dep_time_ut']
            if (zero_len) & (not zero_len_old):
                zero_len_first_index = index
            zero_len_old = zero_len

            if reiterate:
                index = zero_len_first_index
            else:
                index += 1

            if index < len(self.graph.transport_connections_df):
                row = self.graph.transport_connections_df[index]
            else:
                if self.s.get(self.target):
                    return {
                        'path': self.path[self.target],
                        'routes': self.routes[self.target],
                        'arrival': self.s[self.target],
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
                else:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }


class McCustomizedConnectionScanAlgorithm:
    def __init__(self, graph: CustomizedTimetable, start_time: int, start_node: int, end_node: int, walking_speed: int,
                 ):
        """
        Implementation of the connection scan algorithm. Link on the paper: https://arxiv.org/pdf/1703.05997
        Parameters
        ----------
        graph: Timetable datastructure
        start_time: unix_start_time
        start_node: index of start node
        end_node: index of end node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        cost = (start_time, 0)
        self.start_path = Path(sequence_nodes=[self.source],
                               sequence_route_names=['start'],
                               cost=cost)
        self.start_time = start_time
        self.walking_speed = walking_speed
        self.s = {start_node: [self.start_path]}

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
        """
        Finding shortest path with CSA algorithm Figure 4 in paper.
        Returns
        -------

        """
        start_time = time.monotonic()
        for node, f in self.graph.walk_graph[self.source].items():
            walk_duration = f.w(walking_speed=self.walking_speed)
            arrival_time = self.start_time + walk_duration
            self.s[node] = [Path(sequence_nodes=f.nodes + [node],
                                 sequence_route_names=f.route_names + ['walk'],
                                 cost=(arrival_time, walk_duration))]

        index = bisect_left(self.graph.departure_times, self.start_time)
        row = self.graph.transport_connections_df[index]
        zero_len_old = False
        zero_len_first_index = 0

        while True:
            reiterate = False

            for path_from in self.s.get(row['from_stop_I'], []):
                if path_from.cost[0] <= row['dep_time_ut']:
                    new_path = Path(sequence_nodes=path_from.sequence_nodes + [row['to_stop_I']],
                                    sequence_route_names=path_from.sequence_route_names + [row['route_I']],
                                    cost=(row['arr_time_ut'], path_from.cost[1]))
                    self.s[row['to_stop_I']], add_new_element = self.add_to_non_dominated_list(
                        self.s.get(row['to_stop_I'], []), new_path)
                    if add_new_element:
                        if row['arr_time_ut'] == row['dep_time_ut']:
                            reiterate = True
                        for node, f in self.graph.walk_graph[row['to_stop_I']].items():
                            walk_duration = f.w(walking_speed=self.walking_speed)
                            walk_path = Path(sequence_nodes=new_path.sequence_nodes + [node],
                                             sequence_route_names=new_path.sequence_route_names + ['walk'],
                                             cost=(new_path.cost[0] + walk_duration, new_path.cost[1] + walk_duration))
                            self.s[node], add_new_element = self.add_to_non_dominated_list(
                                self.s.get(node, []), walk_path)

            zero_len = row['arr_time_ut'] == row['dep_time_ut']
            if (zero_len) & (not zero_len_old):
                zero_len_first_index = index
            zero_len_old = zero_len

            if reiterate:
                index = zero_len_first_index
            else:
                index += 1

            if index < len(self.graph.transport_connections_df):
                row = self.graph.transport_connections_df[index]
            elif self.s.get(self.target):
                return {
                    'paths': self.s[self.target],
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }
            else:
                message = f"Target {self.target} not reachable from node {self.source}"
                logging.warning(message)
                return {
                    'paths': [],
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }
