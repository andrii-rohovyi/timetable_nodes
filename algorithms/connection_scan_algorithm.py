import math
from binary_search import bisect_left
import logging
import time

from timetable import Timetable
from utils import to_milliseconds


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
