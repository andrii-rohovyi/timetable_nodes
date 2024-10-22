from typing import Union, Dict, List
import heapdict
import time
import math
import logging
from helpers.binary_search import bisect_left

from data_structures.graph import ContactionTransportGraph
from algorithms.algorithms_wrapper import _check_running_time
from helpers.utils import to_milliseconds


class FCH:
    def __init__(self, graph: ContactionTransportGraph, start_time: int, start_node: int, end_node: int):
        """
        Forward Search over Contraction Hieararchy
        More information could bw found by the next link:
        https://ojs.aaai.org/index.php/SOCS/article/download/18454/18245/21970#:~:text=Contraction%20hierarchies%20are%20graph%2Dbased,on%20bi%2Ddirectional%20Dijkstra%20search.

        :param graph: ContactionTransportGraph Contracted graph on which we run Forward Search
        :param start_time: int Start time in unix time
        :param start_node: int Start node
        :param end_node: int End node
        """
        self.graph = graph

        self.source = start_node
        self.target = end_node
        self.start_time = start_time

        self.candidate_weights = {self.source: start_time}
        self.candidate_priorities = heapdict.heapdict({self.source: start_time})
        self.candidate_sequences = {self.source: [self.source]}
        self.candidate_roots = {self.source: [self.source]}
        self.candidate_route_names = {self.source: []}
        self.candidate_down_move = {self.source: False}
        self.lower_upper_index = {self.source: (0, None)}
        self.lower_index = {self.source: 0}
        self.candidate_schedule = {self.source: self.graph.nodes_schedule[start_node]}

    def shortest_path(self,
                      duration: Union[float, None] = None,
                      geometrical_containers=True,
                      time_table_nodes: str = None,
                      ) -> Dict[str, Union[List[Union[int, str]], int]]:
        """
        Adaptation of Forward Search with TTN

        Parameters
        ----------
        duration: maximum duration of algorithm running
        geometrical_containers: If we use GC for Forward Search or not.
            Basic setup is to use it, as otherwise FS will not give an improvement.
        time_table_nodes: Type of TTN. Possible types: 'cst', 'fc', 'fc_chs'.
            More details about all of these options, you could find here: https://arxiv.org/pdf/2410.15715

        Returns
        -------

        """

        exception = None

        winner_node = self.source
        winner_weight = self.start_time

        position_in_edge = self.graph.position_in_edge
        nodes_schedule = self.graph.nodes_schedule

        if geometrical_containers:
            if time_table_nodes == 'cst':
                start_time = time.monotonic()
                while (winner_node != self.target) and (not exception):
                    exception = _check_running_time(start_time, duration, "FCH")
                    departure = bisect_left(nodes_schedule[winner_node], winner_weight)
                    nodes_indexes = position_in_edge[winner_node].get(departure)
                    for node in self.graph.graph[winner_node]:
                        if not self.candidate_down_move[winner_node]:
                            if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, False,
                                                                        nodes_indexes)
                            elif self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, True,
                                                                        nodes_indexes)
                        elif ((self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]) &
                              (self.target in self.graph.geometrical_containers[node])):
                            self._update_vertex_with_node_index_cst(node, winner_node, winner_weight, True,
                                                                    nodes_indexes)

                    try:
                        winner_node, winner_weight = self.candidate_priorities.popitem()
                    except IndexError:
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'path': [],
                            'routes': [],
                            'arrival': math.inf,
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
            elif time_table_nodes == 'fc':
                start_time = time.monotonic()
                while (winner_node != self.target) and (not exception):
                    exception = _check_running_time(start_time, duration, "FCH")

                    m_arr = self.graph.m_arr_fractional.get(winner_node)
                    pointers = self.graph.pointers.get(winner_node)
                    reachable_nodes = self.graph.reachable_nodes.get(winner_node)
                    out = self.graph.graph.get(winner_node)
                    down_move = self.candidate_down_move[winner_node]
                    if pointers:
                        start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_weight)]

                        node = reachable_nodes[0]
                        if not down_move:
                            if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                    winner_node,
                                    winner_weight,
                                    out, node,
                                    start_index,
                                    False)
                            elif self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                    winner_node,
                                    winner_weight,
                                    out, node,
                                    start_index,
                                    True)
                        elif ((self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]) &
                              (self.target in self.graph.geometrical_containers[node])):
                            self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                                 winner_weight,
                                                                                                 out, node,
                                                                                                 start_index,
                                                                                                 True)
                        for i in range(1, len(m_arr)):
                            if winner_weight <= m_arr[i][next_loc - 1]:
                                start_index, next_loc = pointers[i][next_loc - 1]
                            else:
                                start_index, next_loc = pointers[i][next_loc]

                            node = reachable_nodes[i]
                            if not down_move:
                                if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                    self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                        winner_node,
                                        winner_weight,
                                        out, node,
                                        start_index,
                                        False)
                                elif self.target in self.graph.geometrical_containers[node]:
                                    self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                        winner_node,
                                        winner_weight,
                                        out, node,
                                        start_index,
                                        True)
                            elif ((self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]) &
                                  (self.target in self.graph.geometrical_containers[node])):
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                    winner_node,
                                    winner_weight,
                                    out, node,
                                    start_index,
                                    True)
                    for node in self.graph.walking_nodes.get(winner_node, []):
                        if not self.candidate_down_move[winner_node]:
                            if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                self._update_vertex_with_node_index_fractional_cascading_walk_profile(
                                    winner_node,
                                    winner_weight,
                                    out, node,
                                    False)
                            elif self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex_with_node_index_fractional_cascading_walk_profile(
                                    winner_node,
                                    winner_weight,
                                    out, node,
                                    True)
                        elif ((self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]) &
                              (self.target in self.graph.geometrical_containers[node])):
                            self._update_vertex_with_node_index_fractional_cascading_walk_profile(winner_node,
                                                                                                  winner_weight,
                                                                                                  out, node,
                                                                                                  True)

                    try:
                        winner_node, winner_weight = self.candidate_priorities.popitem()
                    except IndexError:
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'path': [],
                            'routes': [],
                            'arrival': math.inf,
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
            elif time_table_nodes == 'fc_chs':
                start_time = time.monotonic()
                while (winner_node != self.target) and (not exception):
                    exception = _check_running_time(start_time, duration, "FCH")

                    m_arr = self.graph.m_arr_fractional.get(winner_node)
                    pointers = self.graph.pointers.get(winner_node)
                    reachable_nodes = self.graph.reachable_nodes.get(winner_node)
                    out = self.graph.graph.get(winner_node)
                    down_move = self.candidate_down_move[winner_node]
                    if pointers:
                        node = reachable_nodes[0]

                        if not down_move:
                            start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_weight)]
                            if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                                     winner_weight,
                                                                                                     out, node,
                                                                                                     start_index,
                                                                                                     False)
                            elif self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                                     winner_weight,
                                                                                                     out, node,
                                                                                                     start_index,
                                                                                                     True)
                        elif self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]:
                            start_index, next_loc = pointers[0][bisect_left(m_arr[0], winner_weight)]
                            if self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex_with_node_index_fractional_cascading_bus_profile(winner_node,
                                                                                                     winner_weight,
                                                                                                     out, node,
                                                                                                     start_index,
                                                                                                     True)
                        else:
                            next_loc = None
                        if next_loc:
                            i = 1
                            while i < len(m_arr):

                                node = reachable_nodes[i]
                                if not down_move:
                                    if winner_weight <= m_arr[i][next_loc - 1]:
                                        start_index, next_loc = pointers[i][next_loc - 1]
                                    else:
                                        start_index, next_loc = pointers[i][next_loc]
                                    if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                            winner_node,
                                            winner_weight,
                                            out, node,
                                            start_index,
                                            False)
                                    elif self.target in self.graph.geometrical_containers[node]:
                                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                            winner_node,
                                            winner_weight,
                                            out, node,
                                            start_index,
                                            True)
                                elif self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]:
                                    if winner_weight <= m_arr[i][next_loc - 1]:
                                        start_index, next_loc = pointers[i][next_loc - 1]
                                    else:
                                        start_index, next_loc = pointers[i][next_loc]
                                    if self.target in self.graph.geometrical_containers[node]:
                                        self._update_vertex_with_node_index_fractional_cascading_bus_profile(
                                            winner_node,
                                            winner_weight,
                                            out, node,
                                            start_index,
                                            True)
                                else:
                                    i = len(m_arr)
                                i += 1
                    try:
                        winner_node, winner_weight = self.candidate_priorities.popitem()
                    except IndexError:
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'path': [],
                            'routes': [],
                            'arrival': math.inf,
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }
            else:
                start_time = time.monotonic()
                while (winner_node != self.target) and (not exception):

                    exception = _check_running_time(start_time, duration, "FCH")
                    for node in self.graph.graph[winner_node]:
                        if not self.candidate_down_move[winner_node]:
                            if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                                self._update_vertex(node, winner_node, winner_weight, False)
                            elif self.target in self.graph.geometrical_containers[node]:
                                self._update_vertex(node, winner_node, winner_weight, True)
                        elif ((self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]) &
                              (self.target in self.graph.geometrical_containers[node])):
                            self._update_vertex(node, winner_node, winner_weight, True)
                    try:
                        winner_node, winner_weight = self.candidate_priorities.popitem()
                    except IndexError:
                        message = f"Target {self.target} not reachable from node {self.source}"
                        logging.warning(message)
                        return {
                            'path': [],
                            'routes': [],
                            'arrival': math.inf,
                            'duration': to_milliseconds(time.monotonic() - start_time)
                        }

        else:
            # Todo: Add TTN to this implementation. We haven't done it yet, since there is no sense to run this option.
            start_time = time.monotonic()
            while (winner_node != self.target) and (not exception):

                exception = _check_running_time(start_time, duration, "FCH")
                for node in self.graph.graph[winner_node]:
                    if not self.candidate_down_move[winner_node]:
                        if self.graph.hierarchy[node] > self.graph.hierarchy[winner_node]:
                            self._update_vertex(node, winner_node, winner_weight, False)
                        else:
                            self._update_vertex(node, winner_node, winner_weight, True)
                    elif self.graph.hierarchy[node] < self.graph.hierarchy[winner_node]:
                        self._update_vertex(node, winner_node, winner_weight, True)

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

    def _update_vertex(self, node: int, winner_node: int, winner_weight: int,
                       down_move: bool):
        """
        Update vertex iteration in Forward Search

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param down_move: bool True in case of movement down
        :return:
        """

        new_weight, sequence_nodes, route_names = self.graph.graph[winner_node][node].arrival(winner_weight)
        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_down_move[node] = down_move
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_down_move[node] = down_move
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_cst(self, node, winner_node, winner_weight, down_move: bool, nodes_indexes):
        """
        Update vertex iteration in Forward Search

        :param node: int Node information about which we update
        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :param down_move: bool True in case of movement down
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
                self.candidate_down_move[node] = down_move
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_down_move[node] = down_move
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_bus_profile(self, winner_node: int, winner_weight: int, out,
                                                                        node, start_index, down_move):
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
                self.candidate_down_move[node] = down_move
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_down_move[node] = down_move
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names

    def _update_vertex_with_node_index_fractional_cascading_walk_profile(self, winner_node: int, winner_weight: int,
                                                                         out, node, down_move):
        """
        Update vertex in TTN mode

        :param winner_node: int. Parent node from each we reach this node
        :param winner_weight: Time in unix at which we have been at winner_node
        :return:
        """

        f = out[node]
        walk_time = winner_weight + f.walk.w
        new_weight, sequence_nodes, route_names = walk_time, f.walk.nodes, f.walk.route_names
        if node in self.candidate_weights.keys():
            if new_weight < self.candidate_weights[node]:
                self.candidate_down_move[node] = down_move
                self.candidate_weights[node] = new_weight
                self.candidate_priorities[node] = new_weight
                self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
                self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
                self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
        elif new_weight != math.inf:
            self.candidate_down_move[node] = down_move
            self.candidate_weights[node] = new_weight
            self.candidate_priorities[node] = new_weight
            self.candidate_sequences[node] = self.candidate_sequences[winner_node] + sequence_nodes[1:]
            self.candidate_roots[node] = self.candidate_roots[winner_node] + [node]
            self.candidate_route_names[node] = self.candidate_route_names[winner_node] + route_names
