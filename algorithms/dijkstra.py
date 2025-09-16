from typing import Union, Dict, List
import heapdict
import time
import math
import logging
# from bisect import bisect_left
from helpers.binary_search import bisect_left

from data_structures.graph import TransportGraph, CustomizedTransportGraph
from algorithms.algorithms_wrapper import _check_running_time
from helpers.utils import to_milliseconds
from helpers.atf import ATF
from helpers.path import Path


class TimeDependentDijkstra:
    def __init__(self, graph: TransportGraph, start_time: int, start_node: int, end_node: int):
        """
        Realization of Dijkstra algorithm
        https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

        :param graph: TransportGraph Multimodal transport network
        :param start_time: int Start time in unix
        :param start_node: int Start node from, which we build a path
        :param end_node: int Target node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.start_time = start_time

        self.candidate_weights = {self.source: start_time}
        self.candidate_priorities = heapdict.heapdict({self.source: start_time})
        self.candidate_sequences = {self.source: [self.source]}
        self.candidate_route_names = {self.source: []}
        self.candidate_roots = {self.source: [self.source]}

    def shortest_path(self,
                      duration: Union[float, None] = None,
                      time_table_nodes: str = None,
                      ) -> Dict[str, Union[List[Union[int, str]], int]]:
        """
        Adaptation of Dijkstra with TTN

        Parameters
        ----------
        duration: maximum duration of algorithm running
        time_table_nodes: Type of TTN. Possible types: 'cst', 'fc'
            More details about all of these options, you could find here: https://arxiv.org/pdf/2410.15715

        Returns
        -------

        """

        exception = None

        winner_node = self.source
        winner_weight = self.start_time
        if time_table_nodes == 'cst':
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                departure = bisect_left(self.graph.nodes_schedule[winner_node], winner_weight)
                nodes_indexes = self.graph.position_in_edge[winner_node].get(
                    departure)  # {nodeA: start_index, nodeB: start_index}

                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, nodes_indexes)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
        elif time_table_nodes == 'bst':
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                (weight, nodes_indexes) = self.graph.nodes_trees[winner_node].bisect_left(
                    winner_weight)  # {nodeA: start_index, nodeB: start_index}

                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, nodes_indexes)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
        elif time_table_nodes == 'fc':
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                m_arr = self.graph.m_arr_fractional.get(winner_node)
                pointers = self.graph.pointers.get(winner_node)
                reachable_nodes = self.graph.reachable_nodes.get(winner_node)
                out = self.graph.graph.get(winner_node)

                if pointers:
                    start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_weight)]

                    node = reachable_nodes[0]
                    self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                         winner_weight,
                                                                                         out, node, start_index)

                    for i in range(1, len(m_arr)):
                        if winner_weight <= m_arr[i][next_loc - 1]:
                            start_index, next_loc = pointers[i][next_loc - 1]
                        else:
                            start_index, next_loc = pointers[i][next_loc]

                        node = reachable_nodes[i]
                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                             winner_weight,
                                                                                             out, node,
                                                                                             start_index)
                for node in self.graph.walking_nodes.get(winner_node, []):
                    self._update_vertex_with_node_index_fractional_cascading_walk_profile(winner_node,
                                                                                          winner_weight, out,
                                                                                          node)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }

        else:
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex(node, winner_node, winner_weight, f)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()

                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
        if exception:
            return {
                'path': self.candidate_sequences[winner_node],
                'routes': self.candidate_route_names[winner_node],
                'roots': self.candidate_roots[winner_node],
                'arrival': winner_weight,
                'duration': to_milliseconds(time.monotonic() - start_time)
            }

        return {
            'path': self.candidate_sequences[self.target],
            'routes': self.candidate_route_names[winner_node],
            'roots': self.candidate_roots[winner_node],
            'arrival': winner_weight,
            'duration': to_milliseconds(time.monotonic() - start_time)
        }

    def _update_vertex(self, node: int, winner_node: int, winner_weight: int, f: ATF):
        """
        Update vertex iteration in Dijkstra

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param f: ATF Function, which represent movement from winner_node to node
        :return:
        """
        new_weight, sequence_nodes, route_names = f.arrival(winner_weight)
        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_cst(self, node: int, winner_node: int, winner_weight: int,
                                           nodes_indexes: Dict[int, int]):
        """
        Update vertex in TTN mode

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param nodes_indexes: Dict[int, int] Dictionaries, where key values are nodes
                            and values is index in ATF Bus profile
        :return:
        """
        l = walk_time = math.inf
        sequence_nodes = []
        route_names = []
        f = self.graph.graph[winner_node][node]
        if nodes_indexes:
            start_index = nodes_indexes[node]
            if start_index < f.size:
                l = f.buses[start_index].a
                sequence_nodes = f.buses[start_index].nodes
                route_names = f.buses[start_index].route_names
        if f.walk:
            walk_time = winner_weight + f.walk.w
        if walk_time < l:
            new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names
        else:
            new_weight, sequence_nodes, route_names = l, sequence_nodes, route_names

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_bus_profile(self, winner_node: int, winner_weight: int, out,
                                                                        node, start_index):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """
        f = out[node]
        l = walk_time = math.inf
        sequence_nodes = []
        route_names = []
        if start_index is not None:
            if start_index < f.size:
                l = f.buses[start_index].a
                sequence_nodes = f.buses[start_index].nodes
                route_names = f.buses[start_index].route_names
        if f.walk:
            walk_time = winner_weight + f.walk.w
        if walk_time < l:
            new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names
        else:
            new_weight, sequence_nodes, route_names = l, sequence_nodes, route_names

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_walk_profile(self, winner_node: int, winner_weight: int,
                                                                         out, node):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """

        f = out[node]

        if f.walk:
            walk_time = winner_weight + f.walk.w
            new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names
            if node in self.candidate_weights.keys():
                if new_weight < self.candidate_weights[node]:
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
            elif new_weight != math.inf:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names


class Dijkstra:
    def __init__(self, graph: Dict, start_node: int, end_node: int):
        """
        Realization of Dijkstra algorithm
        https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

        :param graph: Dict with weight
        :param start_node: int Start node from, which we build a path
        :param end_node: int Target node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node

        self.candidate_weights = {self.source: 0}
        self.candidate_priorities = heapdict.heapdict({self.source: 0})
        self.candidate_sequences = {self.source: [self.source]}
        self.candidate_roots = {self.source: [self.source]}

    def shortest_path(self,
                      duration: Union[float, None] = None,
                      ) -> Dict[str, Union[List[Union[int, str]], int]]:
        """
        Adaptation of Dijkstra with TTN

        Parameters
        ----------
        duration: maximum duration of algorithm running

        Returns
        -------

        """

        exception = None

        winner_node = self.source
        winner_weight = 0

        start_time = time.monotonic()
        while (winner_node != self.target) and (not exception):

            exception = _check_running_time(start_time, duration, "Dijkstra")

            for node, w in self.graph[winner_node].items():
                self._update_vertex(node, winner_node, winner_weight, w)

            try:
                winner_node, winner_weight = self.candidate_priorities.popitem()
            except IndexError:
                message = f"Target {self.target} not reachable from node {self.source}"
                logging.warning(message)
                return {
                    'path': [],
                    'routes': [],
                    'roots': [],
                    'arrival': math.inf,
                    'duration': to_milliseconds(time.monotonic() - start_time)
                }
        if exception:
            return {
                'path': self.candidate_sequences[winner_node],
                'roots': self.candidate_roots[winner_node],
                'arrival': winner_weight,
                'duration': to_milliseconds(time.monotonic() - start_time)
            }

        return {
            'path': self.candidate_sequences[self.target],
            'roots': self.candidate_roots[winner_node],
            'arrival': winner_weight,
            'duration': to_milliseconds(time.monotonic() - start_time)
        }

    def _update_vertex(self, node: int, winner_node: int, winner_weight: int, w: int):
        """
        Update vertex iteration in Dijkstra

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param w: ATF Function, which represent movement from winner_node to node
        :return:
        """
        new_weight = w + winner_weight
        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + [node]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + [node]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]


class CustomizedTimeDependentDijkstra(TimeDependentDijkstra):
    def __init__(self, graph: CustomizedTransportGraph,
                 start_time: int,
                 walking_speed: int,
                 start_node: int,
                 end_node: int):
        """
        Realization of Dijkstra algorithm
        https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

        :param graph: TransportGraph Multimodal transport network
        :param start_time: int Start time in unix
        :param start_node: int Start node from, which we build a path
        :param end_node: int Target node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.walking_speed = walking_speed
        self.start_time = start_time

        self.candidate_weights = {self.source: start_time}
        self.candidate_priorities = heapdict.heapdict({self.source: start_time})
        self.candidate_sequences = {self.source: [self.source]}
        self.candidate_route_names = {self.source: ['start']}
        self.candidate_roots = {self.source: [self.source]}

        self.candidate_weights_bus = {self.source: start_time}
        self.candidate_sequences_bus = {self.source: [self.source]}
        self.candidate_route_names_bus = {self.source: ['start']}
        self.candidate_roots_bus = {self.source: [self.source]}

    def _update_vertex_with_node_index_cst(self, node: int, winner_node: int, winner_weight: int,
                                           nodes_indexes: Dict[int, int]):
        """
        Update vertex in TTN mode

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param nodes_indexes: Dict[int, int] Dictionaries, where key values are nodes
                            and values is index in ATF Bus profile
        :return:
        """
        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []
        f = self.graph.graph[winner_node][node]
        if nodes_indexes:
            start_index = nodes_indexes[node]
            if start_index < f.size:
                new_weight_bus = new_weight = f.buses[start_index].a
                sequence_nodes = f.buses[start_index].nodes
                route_names = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(walking_speed=self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_walk_profile(self, winner_node: int, winner_weight: int,
                                                                         out, node):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """

        f = out[node]

        if f.walk:
            walk_duration = f.walk.w(walking_speed=self.walking_speed)
            walk_time = winner_weight + walk_duration
            new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names

            if node in self.candidate_weights.keys():
                if new_weight < self.candidate_weights[node]:
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

            elif new_weight != math.inf:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_bus_profile(self, winner_node: int, winner_weight: int, out,
                                                                        node, start_index):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """
        f = out[node]
        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []
        if start_index is not None:
            if start_index < f.size:
                new_weight = new_weight_bus = f.buses[start_index].a
                sequence_nodes = f.buses[start_index].nodes
                route_names = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(walking_speed=self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names


        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex(self, node: int, winner_node: int, winner_weight: int, f: ATF):
        """
        Update vertex iteration in Dijkstra

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param f: ATF Function, which represent movement from winner_node to node
        :return:
        """

        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []

        start_index = bisect_left(f.buses, winner_weight, key=lambda x: x.d, lo=0, hi=f.size)
        if start_index < f.size:
            new_weight = new_weight_bus = f.buses[start_index].a
            sequence_nodes = f.buses[start_index].nodes
            route_names = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names


        elif new_weight != math.inf:
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names


class CustomizedTimeDependentDijkstraRestricted(TimeDependentDijkstra):
    def __init__(self, graph: CustomizedTransportGraph,
                 start_time: int,
                 walking_speed: int,
                 start_node: int,
                 end_node: int):
        """
        Realization of Dijkstra algorithm
        https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

        :param graph: TransportGraph Multimodal transport network
        :param start_time: int Start time in unix
        :param start_node: int Start node from, which we build a path
        :param end_node: int Target node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.walking_speed = walking_speed
        self.start_time = start_time

        self.candidate_weights = {self.source: start_time}
        self.candidate_priorities = heapdict.heapdict({self.source: start_time})
        self.candidate_sequences = {self.source: [self.source]}
        self.candidate_route_names = {self.source: ['start']}
        self.candidate_roots = {self.source: [self.source]}

        self.candidate_weights_bus = {self.source: start_time}
        self.candidate_sequences_bus = {self.source: [self.source]}
        self.candidate_route_names_bus = {self.source: ['start']}
        self.candidate_roots_bus = {self.source: [self.source]}

    def _update_vertex_with_node_index_cst(self, node: int, winner_node: int, winner_weight: int,
                                           nodes_indexes: Dict[int, int], f):
        """
        Update vertex in TTN mode

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param nodes_indexes: Dict[int, int] Dictionaries, where key values are nodes
                            and values is index in ATF Bus profile
        :return:
        """
        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []
        if nodes_indexes:
            start_index = nodes_indexes[node]
            if start_index < f.size:
                new_weight_bus = new_weight = f.buses[start_index].a
                sequence_nodes = sequence_nodes_bus = f.buses[start_index].nodes
                route_names = route_names_bus = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(walking_speed=self.walking_speed, bicycle_speed=self.bicycle_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
                elif winner_node in self.candidate_weights_bus.keys():
                    bus_walk = self.candidate_weights_bus[winner_node] + walk_duration
                    if bus_walk < new_weight_bus:
                        if bus_walk < self.candidate_weights[node]:
                            self.candidate_weights[node] = bus_walk
                            self.candidate_priorities[node] = bus_walk
                            self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                              + sequence_nodes_walk)
                            self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                            self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                                + route_names_walk)
                    elif new_weight_bus < self.candidate_weights[node]:
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
                elif new_weight_bus < self.candidate_weights[node]:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

        elif new_weight != math.inf:
            if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
            elif winner_node in self.candidate_weights_bus.keys():

                bus_walk = self.candidate_weights_bus[winner_node] + walk_duration

                if bus_walk < new_weight_bus:
                    self.candidate_weights[node] = bus_walk
                    self.candidate_priorities[node] = bus_walk
                    self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                      + sequence_nodes_walk)
                    self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                        + route_names_walk)
                else:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)
            elif new_weight_bus != math.inf:
                self.candidate_weights[node] = new_weight_bus
                self.candidate_priorities[node] = new_weight_bus
                self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                  + sequence_nodes_bus)
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                    + route_names_bus)

        if new_weight_bus != math.inf:
            if node in self.candidate_weights_bus.keys():
                if new_weight_bus < self.candidate_weights_bus[node]:
                    self.candidate_weights_bus[node] = new_weight_bus
                    self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                    self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                    if node not in self.candidate_priorities.keys():
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
            else:
                self.candidate_weights_bus[node] = new_weight_bus
                self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                if node not in self.candidate_priorities.keys():
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

    def _update_vertex_with_node_index_fractional_cascading_walk_profile(self, winner_node: int, winner_weight: int,
                                                                         f, node):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """

        if f.walk:

            walk_duration = f.walk.w(walking_speed=self.walking_speed, bicycle_speed=self.bicycle_speed)
            walk_time = winner_weight + walk_duration
            new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names

            if node in self.candidate_weights.keys():
                if new_weight < self.candidate_weights[node]:
                    if self.candidate_route_names[winner_node][-1] != 'walk':
                        self.candidate_weights[node] = new_weight
                        self.candidate_priorities[node] = new_weight
                        self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
                    elif winner_node in self.candidate_weights_bus.keys():
                        bus_walk = self.candidate_weights_bus[winner_node] + walk_duration
                        if bus_walk < self.candidate_weights[node]:
                            self.candidate_weights[node] = bus_walk
                            self.candidate_priorities[node] = bus_walk
                            self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                              + sequence_nodes)
                            self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                            self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                                + route_names)

            elif new_weight != math.inf:

                if ((self.candidate_route_names[winner_node][-1] != 'walk') or (f.walk.mode == 'bicycle')):
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
                elif winner_node in self.candidate_weights_bus.keys():

                    bus_walk = self.candidate_weights_bus[winner_node] + walk_duration

                    self.candidate_weights[node] = bus_walk
                    self.candidate_priorities[node] = bus_walk
                    self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                      + sequence_nodes)
                    self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                        + route_names)

    def _update_vertex_with_node_index_fractional_cascading_bus_profile(self, winner_node: int, winner_weight: int, out,
                                                                        node, start_index):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """
        f = out[node]
        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []
        if start_index is not None:
            if start_index < f.size:
                new_weight = new_weight_bus = f.buses[start_index].a
                sequence_nodes = sequence_nodes_bus = f.buses[start_index].nodes
                route_names = route_names_bus = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(walking_speed=self.walking_speed, bicycle_speed=self.bicycle_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
                elif winner_node in self.candidate_weights_bus.keys():
                    bus_walk = self.candidate_weights_bus[winner_node] + walk_duration
                    if bus_walk < new_weight_bus:
                        if bus_walk < self.candidate_weights[node]:
                            self.candidate_weights[node] = bus_walk
                            self.candidate_priorities[node] = bus_walk
                            self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                              + sequence_nodes_walk)
                            self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                            self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                                + route_names_walk)
                    elif new_weight_bus < self.candidate_weights[node]:
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
                elif new_weight_bus < self.candidate_weights[node]:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

        elif new_weight != math.inf:
            if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
            elif winner_node in self.candidate_weights_bus.keys():

                bus_walk = self.candidate_weights_bus[winner_node] + walk_duration

                if bus_walk < new_weight_bus:
                    self.candidate_weights[node] = bus_walk
                    self.candidate_priorities[node] = bus_walk
                    self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                      + sequence_nodes_walk)
                    self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                        + route_names_walk)
                else:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)
            elif new_weight_bus != math.inf:
                self.candidate_weights[node] = new_weight_bus
                self.candidate_priorities[node] = new_weight_bus
                self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                  + sequence_nodes_bus)
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                    + route_names_bus)

        if new_weight_bus != math.inf:
            if node in self.candidate_weights_bus.keys():
                if new_weight_bus < self.candidate_weights_bus[node]:
                    self.candidate_weights_bus[node] = new_weight_bus
                    self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                    self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                    if node not in self.candidate_priorities.keys():
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
            else:
                self.candidate_weights_bus[node] = new_weight_bus
                self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                if node not in self.candidate_priorities.keys():
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

    def _update_vertex(self, node: int, winner_node: int, winner_weight: int, f: ATF):
        """
        Update vertex iteration in Dijkstra

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param f: ATF Function, which represent movement from winner_node to node
        :return:
        """
        new_weight_bus = new_weight = math.inf
        sequence_nodes = []
        route_names = []

        start_index = bisect_left(f.buses, winner_weight, key=lambda x: x.d, lo=0, hi=f.size)
        if start_index < f.size:
            new_weight = new_weight_bus = f.buses[start_index].a
            sequence_nodes = sequence_nodes_bus = f.buses[start_index].nodes
            route_names = route_names_bus = f.buses[start_index].route_names
        if f.walk:
            walk_duration = f.walk.w(self.walking_speed, self.bicycle_speed)
            new_weight_walk = winner_weight + walk_duration
            sequence_nodes_walk = f.walk.nodes
            route_names_walk = f.walk.route_names
            if new_weight_walk < new_weight_bus:
                new_weight = new_weight_walk
                sequence_nodes = sequence_nodes_walk
                route_names = route_names_walk

        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                    self.candidate_weights[node] = new_weight
                    self.candidate_priorities[node] = new_weight
                    self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
                elif winner_node in self.candidate_weights_bus.keys():
                    bus_walk = self.candidate_weights_bus[winner_node] + walk_duration
                    if bus_walk < new_weight_bus:
                        if bus_walk < self.candidate_weights[node]:
                            self.candidate_weights[node] = bus_walk
                            self.candidate_priorities[node] = bus_walk
                            self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                              + sequence_nodes_walk)
                            self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                            self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                                + route_names_walk)
                    elif new_weight_bus < self.candidate_weights[node]:
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
                elif new_weight_bus < self.candidate_weights[node]:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

        elif new_weight != math.inf:
            if ((route_names[0] != 'walk') | (self.candidate_route_names[winner_node][-1] != 'walk')):
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
            elif winner_node in self.candidate_weights_bus.keys():

                bus_walk = self.candidate_weights_bus[winner_node] + walk_duration

                if bus_walk < new_weight_bus:
                    self.candidate_weights[node] = bus_walk
                    self.candidate_priorities[node] = bus_walk
                    self.candidate_sequences[node] = (self.candidate_sequences_bus[winner_node]
                                                      + sequence_nodes_walk)
                    self.candidate_roots[node] = self.candidate_roots_bus[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names_bus[winner_node]
                                                        + route_names_walk)
                else:
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)
            elif new_weight_bus != math.inf:
                self.candidate_weights[node] = new_weight_bus
                self.candidate_priorities[node] = new_weight_bus
                self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                  + sequence_nodes_bus)
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                    + route_names_bus)

        if new_weight_bus != math.inf:
            if node in self.candidate_weights_bus.keys():
                if new_weight_bus < self.candidate_weights_bus[node]:
                    self.candidate_weights_bus[node] = new_weight_bus
                    self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                    self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                    if node not in self.candidate_priorities.keys():
                        self.candidate_weights[node] = new_weight_bus
                        self.candidate_priorities[node] = new_weight_bus
                        self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                          + sequence_nodes_bus)
                        self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                        self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                            + route_names_bus)
            else:
                self.candidate_weights_bus[node] = new_weight_bus
                self.candidate_sequences_bus[node] = self.candidate_sequences[winner_node] + sequence_nodes_bus[1:]
                self.candidate_roots_bus[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names_bus[node] = self.candidate_route_names[winner_node] + route_names_bus
                if node not in self.candidate_priorities.keys():
                    self.candidate_weights[node] = new_weight_bus
                    self.candidate_priorities[node] = new_weight_bus
                    self.candidate_sequences[node] = (self.candidate_sequences[winner_node]
                                                      + sequence_nodes_bus)
                    self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                    self.candidate_route_names[node] = (self.candidate_route_names[winner_node]
                                                        + route_names_bus)

    def shortest_path(self,
                      duration: Union[float, None] = None,
                      time_table_nodes: str = None,
                      ) -> Dict[str, Union[List[Union[int, str]], int]]:
        """
        Adaptation of Dijkstra with TTN

        Parameters
        ----------
        duration: maximum duration of algorithm running
        time_table_nodes: Type of TTN. Possible types: 'cst', 'fc'
            More details about all of these options, you could find here: https://arxiv.org/pdf/2410.15715

        Returns
        -------

        """

        exception = None

        winner_node = self.source
        winner_weight = self.start_time
        if time_table_nodes == 'cst':
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                departure = bisect_left(self.graph.nodes_schedule[winner_node], winner_weight)
                nodes_indexes = self.graph.position_in_edge[winner_node].get(
                    departure)  # {nodeA: start_index, nodeB: start_index}

                if winner_node > 0:
                    for node, f in self.graph.graph[winner_node].items():
                        f = self.graph.graph[winner_node][node]
                        self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, nodes_indexes, f)
                elif self.candidate_route_names[winner_node][-1] == 'bicycle':
                    for node, f in self.graph.graph_bicycle_to_node[winner_node].items():
                        f = self.graph.graph_bicycle_to_node[winner_node][node]
                        self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, nodes_indexes, f)
                else:
                    for node, f in self.graph.graph_bicycle_to_bicycle[winner_node].items():
                        f = self.graph.graph_bicycle_to_bicycle[winner_node][node]
                        self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, nodes_indexes, f)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
        elif time_table_nodes == 'fc':
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                m_arr = self.graph.m_arr_fractional.get(winner_node)
                pointers = self.graph.pointers.get(winner_node)
                reachable_nodes = self.graph.reachable_nodes.get(winner_node)

                if pointers:
                    start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_weight)]

                    node = reachable_nodes[0]
                    out = self.graph.graph.get(winner_node)
                    self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                         winner_weight,
                                                                                         out, node, start_index)

                    for i in range(1, len(m_arr)):
                        if winner_weight <= m_arr[i][next_loc - 1]:
                            start_index, next_loc = pointers[i][next_loc - 1]
                        else:
                            start_index, next_loc = pointers[i][next_loc]

                        node = reachable_nodes[i]
                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                             winner_weight,
                                                                                             out, node,
                                                                                             start_index)
                if winner_node > 0:
                    out = self.graph.graph.get(winner_node)
                    for node in self.graph.walking_nodes.get(winner_node, []):
                        self._update_vertex_with_node_index_fractional_cascading_walk_profile(winner_node,
                                                                                              winner_weight, out[node],
                                                                                              node)
                elif self.candidate_route_names[winner_node][-1] == 'bicycle':
                    out = self.graph.graph_bicycle_to_node.get(winner_node)
                    for node in self.graph.graph_bicycle_to_node.get(winner_node, []):
                        self._update_vertex_with_node_index_fractional_cascading_walk_profile(winner_node,
                                                                                              winner_weight, out[node],
                                                                                              node)
                else:
                    out = self.graph.graph_bicycle_to_bicycle.get(winner_node)
                    for node in self.graph.graph_bicycle_to_bicycle.get(winner_node, []):
                        self._update_vertex_with_node_index_fractional_cascading_walk_profile(winner_node,
                                                                                              winner_weight, out[node],
                                                                                              node)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }

        else:
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "Dijkstra")

                if winner_node > 0:
                    for node, f in self.graph.graph[winner_node].items():
                        self._update_vertex(node, winner_node, winner_weight, f)
                elif self.candidate_route_names[winner_node][-1] == 'bicycle':
                    for node, f in self.graph.graph_bicycle_to_node[winner_node].items():
                        self._update_vertex(node, winner_node, winner_weight, f)
                else:
                    for node, f in self.graph.graph_bicycle_to_bicycle[winner_node].items():
                        self._update_vertex(node, winner_node, winner_weight, f)

                try:
                    winner_node, winner_weight = self.candidate_priorities.popitem()
                except IndexError:
                    message = f"Target {self.target} not reachable from node {self.source}"
                    logging.warning(message)
                    return {
                        'path': [],
                        'routes': [],
                        'roots': [],
                        'arrival': math.inf,
                        'duration': to_milliseconds(time.monotonic() - start_time)
                    }
        if exception:
            return {
                'path': self.candidate_sequences[winner_node],
                'routes': self.candidate_route_names[winner_node],
                'roots': self.candidate_roots[winner_node],
                'arrival': winner_weight,
                'duration': to_milliseconds(time.monotonic() - start_time)
            }

        return {
            'path': self.candidate_sequences[self.target],
            'routes': self.candidate_route_names[winner_node],
            'roots': self.candidate_roots[winner_node],
            'arrival': winner_weight,
            'duration': to_milliseconds(time.monotonic() - start_time)

        }


class McCustomizedTimeDependentDijkstra:
    def __init__(self, graph: CustomizedTransportGraph,
                 start_time: int,
                 walking_speed: int,
                 start_node: int,
                 end_node: int):
        """
        Realization of Dijkstra algorithm
        https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

        :param graph: TransportGraph Multimodal transport network
        :param start_time: int Start time in unix
        :param start_node: int Start node from, which we build a path
        :param end_node: int Target node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.walking_speed = walking_speed
        self.start_time = start_time
        cost = (start_time, 0)
        self.start_path = Path(sequence_nodes=[self.source],
                               sequence_route_names=['start'],
                               cost=cost)
        self.candidate_paths = {
            self.source: [self.start_path]  # (duration, walking_duration, transport_changes_amount)
        }
        self.candidate_priorities = heapdict.heapdict({(self.source, self.start_path): cost})
        self.candidate_priorities_second = heapdict.heapdict({(self.source, self.start_path): cost[1]})

    def shortest_path(self,
                      time_table_nodes: str = None,
                      ) -> Dict[str, Union[List[Union[int, str]], int]]:
        """
        Adaptation of Dijkstra with TTN

        Parameters
        ----------
        duration: maximum duration of algorithm running
        time_table_nodes: Type of TTN. Possible types: 'cst', 'fc'
            More details about all of these options, you could find here: https://arxiv.org/pdf/2410.15715

        Returns
        -------

        """

        winner_node = self.source
        winner_path = self.start_path
        if time_table_nodes == 'cst':
            start_time = time.monotonic()
            while True:

                departure = bisect_left(self.graph.nodes_schedule[winner_node], winner_path.cost[0])
                nodes_indexes = self.graph.position_in_edge[winner_node].get(
                    departure)  # {nodeA: start_index, nodeB: start_index}
                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex_with_node_index_cst(node,
                                                            winner_path,
                                                            f,
                                                            nodes_indexes)

                try:
                    (winner_node, winner_path), new_cost = self.candidate_priorities.popitem()

                except IndexError:
                    if self.target not in self.candidate_paths.keys():
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'paths': [],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
                    else:
                        return {
                            'paths': self.candidate_paths[self.target],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
        elif time_table_nodes == 'bst':
            start_time = time.monotonic()
            while True:
                (weight, nodes_indexes) = self.graph.nodes_trees[winner_node].bisect_left(
                    winner_path.cost[0])

                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex_with_node_index_cst(node,
                                                            winner_path,
                                                            f,
                                                            nodes_indexes)

                try:
                    (winner_node, winner_path), new_cost = self.candidate_priorities.popitem()

                except IndexError:
                    if self.target not in self.candidate_paths.keys():
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'paths': [],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
                    else:
                        return {
                            'paths': self.candidate_paths[self.target],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }

        elif time_table_nodes == 'fc':
            start_time = time.monotonic()
            while True:

                m_arr = self.graph.m_arr_fractional.get(winner_node)
                pointers = self.graph.pointers.get(winner_node)
                reachable_nodes = self.graph.reachable_nodes.get(winner_node)

                if pointers:
                    start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_path.cost[0])]

                    node = reachable_nodes[0]
                    f = self.graph.graph[winner_node][node]
                    self._update_vertex_with_node_index_fractional_cascading_bus_profile(node,
                                                                                         winner_path,
                                                                                         f,
                                                                                         start_index)

                    for i in range(1, len(m_arr)):
                        if winner_path.cost[0] <= m_arr[i][next_loc - 1]:
                            start_index, next_loc = pointers[i][next_loc - 1]
                        else:
                            start_index, next_loc = pointers[i][next_loc]

                        node = reachable_nodes[i]
                        f = self.graph.graph[winner_node][node]
                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(node,
                                                                                             winner_path,
                                                                                             f,
                                                                                             start_index)
                for node in self.graph.walking_nodes.get(winner_node, []):
                    f = self.graph.graph[winner_node][node]
                    self._update_vertex_with_node_index_fractional_cascading_walk_profile(node,
                                                                                          winner_path,
                                                                                          f)

                try:
                    (winner_node, winner_path), new_cost = self.candidate_priorities.popitem()

                except IndexError:
                    if self.target not in self.candidate_paths.keys():
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'paths': [],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
                    else:
                        return {
                            'paths': self.candidate_paths[self.target],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }

        else:
            start_time = time.monotonic()
            while True:
                for node, f in self.graph.graph[winner_node].items():
                    self._update_vertex(node, winner_path, f)
                try:
                    (winner_node, winner_path), new_costs = self.candidate_priorities.popitem()
                    '''
                    if not optimal_time:
                        (winner_node, winner_path), new_costs = self.candidate_priorities.popitem()
                        del self.candidate_priorities_second[(winner_node, winner_path)]
                    else:
                        (winner_node, winner_path), second_weight = self.candidate_priorities_second.popitem()
                        
                    if (winner_node == self.target) and (not optimal_time):
                        optimal_time = True
                        max_walking = new_costs[1]
                    '''

                except IndexError:
                    if self.target not in self.candidate_paths.keys():
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'paths': [],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
                    else:
                        return {
                            'paths': self.candidate_paths[self.target],
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }

    def _update_vertex_with_node_index_cst(self,
                                           node: int,
                                           path: Path,
                                           f: ATF,
                                           nodes_indexes: Dict[int, int]):
        """
        Update vertex in TTN mode

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param nodes_indexes: Dict[int, int] Dictionaries, where key values are nodes
                            and values is index in ATF Bus profile
        :return:
        """

        new_paths = []
        new_cost = (math.inf, math.inf)
        winner_weight, old_cost_walk = path.cost
        if nodes_indexes:
            start_index = nodes_indexes[node]
            if start_index < f.size:
                new_cost = (f.buses[start_index].a, old_cost_walk)
                path_bus = Path(sequence_nodes=path.sequence_nodes + [node],
                                sequence_route_names=path.sequence_route_names + f.buses[
                                    start_index].route_names,
                                cost=new_cost)
                new_paths.append(path_bus)

        if f.walk:
            walk_duration = f.walk.w(self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            path_walk = Path(sequence_nodes=path.sequence_nodes + [node],
                             sequence_route_names=path.sequence_route_names + ['walk'],
                             cost=(new_weight_walk, old_cost_walk + walk_duration))
            if path_walk.cost[0] < new_cost[0]:
                new_cost = path_walk.cost
                new_paths.append(path_walk)

        if node in self.candidate_paths.keys():
            for new_path in new_paths:
                self.candidate_paths[node], add_new_element = self.add_to_non_dominated_list(
                    self.candidate_paths[node], new_path)
                if add_new_element:
                    self.candidate_priorities[(node, new_path)] = new_path.cost
        elif new_cost[0] != math.inf:
            self.candidate_paths[node] = new_paths
            for new_path in new_paths:
                self.candidate_priorities[(node, new_path)] = new_path.cost

    def _update_vertex_with_node_index_fractional_cascading_walk_profile(self,
                                                                         node: int,
                                                                         path: Path,
                                                                         f: ATF,
                                                                         ):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """

        if f.walk:
            winner_weight, old_cost_walk = path.cost

            walk_duration = f.walk.w(self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            path_walk = Path(sequence_nodes=path.sequence_nodes + [node],
                             sequence_route_names=path.sequence_route_names + ['walk'],
                             cost=(new_weight_walk, old_cost_walk + walk_duration))

            if node in self.candidate_paths.keys():
                self.candidate_paths[node], add_new_element = self.add_to_non_dominated_list(
                    self.candidate_paths[node], path_walk)
                if add_new_element:
                    self.candidate_priorities[(node, path_walk)] = path_walk.cost

            elif path_walk.cost[0] != math.inf:
                self.candidate_paths[node] = [path_walk]
                self.candidate_priorities[(node, path_walk)] = path_walk.cost

    def _update_vertex_with_node_index_fractional_cascading_bus_profile(self,
                                                                        node: int,
                                                                        path: Path,
                                                                        f: ATF,
                                                                        start_index):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """
        new_paths = []
        new_cost = (math.inf, math.inf)
        winner_weight, old_cost_walk = path.cost

        if start_index is not None:
            if start_index < f.size:
                new_cost = (f.buses[start_index].a, old_cost_walk)
                path_bus = Path(sequence_nodes=path.sequence_nodes + [node],
                                sequence_route_names=path.sequence_route_names + f.buses[
                                    start_index].route_names,
                                cost=new_cost)
                new_paths.append(path_bus)

        if f.walk:
            walk_duration = f.walk.w(self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            path_walk = Path(sequence_nodes=path.sequence_nodes + [node],
                             sequence_route_names=path.sequence_route_names + ['walk'],
                             cost=(new_weight_walk, old_cost_walk + walk_duration))
            if path_walk.cost[0] < new_cost[0]:
                new_cost = path_walk.cost
                new_paths.append(path_walk)

        if node in self.candidate_paths.keys():
            for new_path in new_paths:
                self.candidate_paths[node], add_new_element = self.add_to_non_dominated_list(
                    self.candidate_paths[node], new_path)
                if add_new_element:
                    self.candidate_priorities[(node, new_path)] = new_path.cost
        elif new_cost[0] != math.inf:
            self.candidate_paths[node] = new_paths
            for new_path in new_paths:
                self.candidate_priorities[(node, new_path)] = new_path.cost

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

    def _update_vertex(self, node: int, path: Path, f: ATF):
        """
        Update vertex iteration in Dijkstra

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param f: ATF Function, which represent movement from winner_node to node
        :return:
        """

        new_paths = []
        new_cost = (math.inf, math.inf)
        winner_weight, old_cost_walk = path.cost
        start_index = bisect_left(f.buses, winner_weight, key=lambda x: x.d, lo=0, hi=f.size)

        if start_index < f.size:
            new_cost = (f.buses[start_index].a, old_cost_walk)
            path_bus = Path(sequence_nodes=path.sequence_nodes + [node],
                            sequence_route_names=path.sequence_route_names + f.buses[
                                start_index].route_names,
                            cost=new_cost)
            new_paths.append(path_bus)

        if f.walk:
            walk_duration = f.walk.w(self.walking_speed)
            new_weight_walk = winner_weight + walk_duration
            path_walk = Path(sequence_nodes=path.sequence_nodes + [node],
                             sequence_route_names=path.sequence_route_names + ['walk'],
                             cost=(new_weight_walk, old_cost_walk + walk_duration))
            if path_walk.cost[0] < new_cost[0]:
                new_cost = path_walk.cost
                new_paths.append(path_walk)

        if node in self.candidate_paths.keys():
            for new_path in new_paths:
                self.candidate_paths[node], add_new_element = self.add_to_non_dominated_list(
                    self.candidate_paths[node], new_path)
                if add_new_element:
                    self.candidate_priorities[(node, new_path)] = new_path.cost
                    #self.candidate_priorities_second[(node, new_path)] = new_path.cost[1]
        elif new_cost[0] != math.inf:
            self.candidate_paths[node] = new_paths
            for new_path in new_paths:
                self.candidate_priorities[(node, new_path)] = new_path.cost
                #self.candidate_priorities_second[(node, new_path)] = new_path.cost[1]
