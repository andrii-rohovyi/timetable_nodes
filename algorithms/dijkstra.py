from typing import Union, Dict, List
import heapdict
import time
import math
import logging
# from bisect import bisect_left
from helpers.binary_search import bisect_left

from data_structures.graph import TransportGraph
from algorithms.algorithms_wrapper import _check_running_time
from helpers.utils import to_milliseconds
from helpers.atf import ATF


class Dijkstra:
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

