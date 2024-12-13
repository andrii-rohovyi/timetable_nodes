import pandas as pd
import numpy as np
from copy import deepcopy
from collections import defaultdict
from tqdm import tqdm
import heapdict
from copy import copy
# from bisect import bisect_left
from helpers.binary_search import bisect_left
from typing import Set, Dict, Union

from helpers.atf import ATF, min_atf
from helpers.trip import Bus, Walk


class TransportGraph:

    def __init__(self,
                 walk_connections: pd.DataFrame,
                 transport_connections: Union[pd.DataFrame, None] = None):
        """
        Class, which represent Multimodal Transport Network.
        It is __init__ by 2 data frames about transport and walk information over the city
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param walk_connections: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """
        if transport_connections:

            transport_connections_df = transport_connections.copy(deep=True)
            transport_connections_df = transport_connections_df.sort_values(by='dep_time_ut')
            transport_connections_df['dep_arr'] = list(zip(transport_connections_df['dep_time_ut'],
                                                           transport_connections_df['arr_time_ut']))
            transport_connections_df['route_I'] = transport_connections_df['route_I'].astype(str)
            transport_connections_dict = transport_connections_df.groupby(by=['from_stop_I', 'to_stop_I']
                                                                          ).agg({'dep_arr': list, 'route_I': list}
                                                                                ).to_dict('index')
        else:
            transport_connections_dict = {}

        walk_connections_dict = walk_connections.set_index(['from_stop_I', 'to_stop_I'])['d_walk'].to_dict()

        self.graph = defaultdict(dict)
        self.in_nodes = defaultdict(dict)
        self.nodes = set()
        self.nodes_schedule = defaultdict(list)
        self.position_in_edge = defaultdict(dict)

        for adjacent_node, node in set(transport_connections_dict.keys()).union(set(walk_connections_dict.keys())):
            nodes_sequence = [adjacent_node, node]
            walk_duration = walk_connections_dict.get((adjacent_node, node))
            walk = None
            if walk_duration:
                walk = Walk(nodes=nodes_sequence, w=walk_duration)
            transport_connections_nodes_dict = transport_connections_dict.get((adjacent_node, node),
                                                                              {'dep_arr': [], 'route_I': []})
            buses = [Bus(nodes=nodes_sequence, c=c,
                         route_names=[transport_connections_nodes_dict['route_I'][i]])
                     for i, c in enumerate(transport_connections_nodes_dict['dep_arr'])]
            g = ATF(walk=walk, buses=buses)
            g.cut()
            if walk or buses:
                self.in_nodes[node][adjacent_node] = self.graph[adjacent_node][node] = g
                self.nodes.add(adjacent_node)
                self.nodes.add(node)

        self.m_arr_fractional = {}
        self.pointers = {}
        self.reachable_nodes = {}
        self.walking_nodes = {}

    @property
    def edges_cnt(self) -> int:
        """
        Calculate count of edges in graph for the statistics
        :return:
        """
        edges_sum = 0
        for i, v in self.graph.items():
            edges_sum += len(v)
        return edges_sum

    @property
    def nodes_cnt(self) -> int:
        """
        Calculate count of nodes for the statistics
        :return:
        """
        return len(self.nodes)

    @property
    def timetable_stats(self) -> Dict[str, float]:
        """
        Calculation of statistics about complexity of edge functions inside the graph.
        Min, Mean, Standard deviation, Max of the sizes of the function over the graph
        :return:
        """
        timetables = []
        for i, v in self.graph.items():
            for i0, v0 in v.items():
                if v0.buses:
                    timetables += [len(v0.buses)]
        timetables = np.array(timetables)
        return {'min_size': timetables.min(),
                'mean_size': timetables.mean(),
                'std_size': timetables.std(),
                'max_size': timetables.max()}

    def edge_difference(self, node: int) -> int:
        """
        Calculate edges difference for node.
        Idea of edge difference could be read here: https://oliviermarty.net/docs/olivier_marty_contraction_hierarchies_rapport.pdf
        :param node: Node for each we want to calculate edge_difference
        :return:
        """
        out_nodes_cnt = len(self.graph[node])
        in_nodes_cnt = len(self.in_nodes[node])

        # TODO: Validate idea with drooping existing shortcuts
        # existing_shortcuts = len({(x, y)
        #                          for (x, y) in product(self.graph[node], self.in_nodes[node])
        #                          if self.graph[y].get(x)})
        # shortcuts_inserted = out_nodes_cnt * in_nodes_cnt - existing_shortcuts

        shortcuts_inserted = out_nodes_cnt * in_nodes_cnt
        edges_removed = out_nodes_cnt + in_nodes_cnt

        return shortcuts_inserted - edges_removed

    def contraction_hierarchy(self):
        """
        Contraction Hierarchy algorithm.
        :return:
        """
        new_graph = ContactionTransportGraph(self.graph, self.in_nodes, self.nodes)
        in_nodes = deepcopy(self.in_nodes)
        graph = deepcopy(self.graph)
        for index in tqdm(range(len(self.nodes))):
            node = new_graph.contraction_priority.popitem()[0]
            new_depth = new_graph.depth[node] + 1
            if in_nodes[node]:

                while in_nodes[node]:
                    previous_node, f = in_nodes[node].popitem()
                    for next_node, g in graph[node].items():
                        if previous_node != next_node:
                            # calculate new connection function

                            new_f = g.composition(f)
                            if new_f:
                                h = graph[previous_node].get(next_node, None)
                                if h:
                                    new_f = min_atf(new_f, h)
                                in_nodes[next_node][previous_node] = graph[previous_node][next_node] = new_f
                                new_graph.in_nodes[next_node][previous_node] = new_graph.graph[previous_node][
                                    next_node] = new_f
                            if not in_nodes[node]:
                                new_graph.depth[next_node] = max(new_graph.depth[next_node], new_depth)
                                new_graph.contraction_priority[next_node] = (new_graph.edge_difference(next_node)
                                                                             + new_graph.depth[next_node])
                        if not in_nodes[node]:
                            del in_nodes[next_node][node]
                    new_graph.depth[previous_node] = max(new_graph.depth[previous_node], new_depth)
                    new_graph.contraction_priority[previous_node] = (new_graph.edge_difference(previous_node)
                                                                     + new_graph.depth[previous_node])
                    del graph[previous_node][node]
            else:
                while graph[node]:
                    next_node, g = graph[node].popitem()
                    new_graph.depth[next_node] = max(new_graph.depth[next_node], new_depth)
                    new_graph.contraction_priority[next_node] = (new_graph.edge_difference(next_node)
                                                                 + new_graph.depth[next_node])
                    del in_nodes[next_node][node]

            del graph[node], in_nodes[node]

            new_graph.hierarchy[node] = index

        return new_graph

    def optimize_binary_search(self):
        """
        TTN algorithm over the standard graph
        :return:
        """
        for node1, out in tqdm(self.graph.items()):
            self.position_in_edge[node1] = {}
            full_list = []
            for node2, f in out.items():
                full_list += [bus.d for bus in f.buses]

            full_list = list(set(full_list))
            full_list.sort()
            self.nodes_schedule[node1] = full_list
            for i, dep in enumerate(full_list):
                self.position_in_edge[node1][i] = {}
                for node2, f in out.items():
                    self.position_in_edge[node1][i][node2] = bisect_left(f.buses, dep, key=lambda x: x.d)

    def fractional_cascading_precomputation(self, sort_strategy='ascending'):
        if sort_strategy == 'ascending':
            sort_function = lambda x: len(x[1].buses)
        else:
            sort_function = lambda x: -len(x[1].buses)

        for node1, out in tqdm(self.graph.items()):
            m_arr = []
            arr = []
            reachable_nodes = []
            walking_nodes = []
            i = 0

            for node2, f in sorted(out.items(), key=sort_function):
                if i == 0:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
                else:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list += [x for k, x in enumerate(m_arr[i - 1]) if k % 2]
                        full_list = list(set(full_list))
                        full_list.sort()
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
            m_arr = m_arr[::-1]
            arr = arr[::-1]
            reachable_nodes = reachable_nodes[::-1]
            self.pointers[node1] = []
            for i in range(len(m_arr)):
                self.pointers[node1].append([])
                for j in range(len(m_arr[i])):
                    self.pointers[node1][i].append([[]] * len(arr[i]))
                    self.pointers[node1][i][j] = [-1] * 2
            for i, l in enumerate(m_arr):
                for j, m in enumerate(m_arr[i]):
                    self.pointers[node1][i][j] = [
                        bisect_left(arr[i], m_arr[i][j]),
                        0 if i == len(m_arr) - 1 else bisect_left(m_arr[i + 1], m_arr[i][j]),
                    ]
            self.m_arr_fractional[node1] = m_arr
            self.reachable_nodes[node1] = reachable_nodes
            self.walking_nodes[node1] = walking_nodes

    def get_positions_fractional_cascading(self, x, node):
        locations = {}
        m_arr = self.m_arr_fractional.get(node)
        pointers = self.pointers.get(node)
        if pointers:
            loc, next_loc = pointers[0][bisect_left(m_arr[0], x)]
            locations[self.reachable_nodes[node][0]] = loc
            for i in range(1, len(m_arr)):
                if x <= m_arr[i][next_loc - 1]:
                    loc, next_loc = pointers[i][next_loc - 1]
                else:
                    loc, next_loc = pointers[i][next_loc]
                locations[self.reachable_nodes[node][i]] = loc
        return locations


class ContactionTransportGraph(TransportGraph):

    def __init__(self, graph: Dict[int, Dict[int, ATF]], in_nodes: Dict[int, Dict[int, ATF]], nodes: Set[int]):
        """
        CH-graph class

        :param graph:  Dict[int, Dict[int, ATF]] Dictionary of out-going edges contains ATF functions between 2 nodes
        :param in_nodes: Dict[int, Dict[int, ATF]] Dictionary of in-going edges contains ATF functions between 2 nodes
        :param nodes: Set of all nodes
        """
        self.graph = deepcopy(graph)
        self.in_nodes = deepcopy(in_nodes)
        self.nodes = deepcopy(nodes)
        self.hierarchy = {}
        self.geometrical_containers = {}
        self.nodes_schedule = defaultdict(list)
        self.position_in_edge = defaultdict(dict)
        self.depth = defaultdict(int)
        self.contraction_priority = heapdict.heapdict()
        self.m_arr_fractional = {}
        self.pointers = {}
        self.reachable_nodes = {}
        self.walking_nodes = {}
        self.m_arr_fractional_old = {}
        self.pointers_old = {}
        self.reachable_nodes_old = {}
        self.walking_nodes_old = {}
        for x in nodes:
            self.contraction_priority[x] = self.edge_difference(x) + self.depth[x]

    def geometrical_container(self):
        """
        Precalculate Geometrical Containers for all nodes in down-mode move. Needed for Forward Search algorithm
        :return:
        """
        for node in tqdm(self.nodes):
            visited = set()
            self._dfs(visited, node)
            self.geometrical_containers[node] = visited

    def _dfs(self, visited: Set[int], node: int):
        """
        Deep first search for building an Geometrical containers for each node in down movement
        :param visited: Set[int] Set of already visited nodes
        :param node: int Node for which we run DFS
        :return:
        """
        if node not in visited:
            visited.add(node)
            for neighbour in self.graph[node]:
                if self.hierarchy[neighbour] < self.hierarchy[node]:
                    self._dfs(visited, neighbour)

    def optimize_binary_search(self):
        """
        TTN algorithm realization
        :return:
        """
        for node1, out in tqdm(self.graph.items()):
            self.position_in_edge[node1] = {}
            full_list = []
            for node2, f in out.items():
                full_list += [bus.d for bus in f.buses]

            full_list = list(set(full_list))
            full_list.sort()
            self.nodes_schedule[node1] = full_list
            for i, dep in enumerate(full_list):
                self.position_in_edge[node1][i] = {}
                for node2, f in out.items():
                    self.position_in_edge[node1][i][node2] = bisect_left(f.buses, dep, key=lambda x: x.d)

    def fractional_cascading_precomputation(self, sort_strategy='ascending'):
        if sort_strategy == 'ascending':
            sort_function = lambda x: len(x[1].buses)
        elif sort_strategy == 'contraction_hierarchy':
            sort_function = lambda x: -self.hierarchy[x[0]]
        else:
            sort_function = lambda x: -len(x[1].buses)

        for node1, out in tqdm(self.graph.items()):
            m_arr = []
            arr = []
            reachable_nodes = []
            walking_nodes = []
            i = 0

            for node2, f in sorted(out.items(), key=sort_function):
                if i == 0:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
                else:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list += [x for k, x in enumerate(m_arr[i - 1]) if k % 2]
                        full_list = list(set(full_list))
                        full_list.sort()
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
            m_arr = m_arr[::-1]
            arr = arr[::-1]
            reachable_nodes = reachable_nodes[::-1]
            self.pointers[node1] = []
            for i in range(len(m_arr)):
                self.pointers[node1].append([])
                for j in range(len(m_arr[i])):
                    self.pointers[node1][i].append([[]] * len(arr[i]))
                    self.pointers[node1][i][j] = [-1] * 2
            for i, l in enumerate(m_arr):
                for j, m in enumerate(m_arr[i]):
                    self.pointers[node1][i][j] = [
                        bisect_left(arr[i], m_arr[i][j]),
                        0 if i == len(m_arr) - 1 else bisect_left(m_arr[i + 1], m_arr[i][j]),
                    ]
            self.m_arr_fractional[node1] = m_arr
            self.reachable_nodes[node1] = reachable_nodes
            self.walking_nodes[node1] = walking_nodes

    def fractional_cascading_precomputation_old(self):
        for node1, out in tqdm(self.graph.items()):
            m_arr = []
            arr = []
            reachable_nodes = []
            walking_nodes = []
            i = 0

            for node2, f in sorted(out.items(), key=lambda x: len(x[1].buses)):

                if i == 0:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
                else:
                    full_list = [bus.d for bus in f.buses]
                    if full_list:
                        arr.append(copy(full_list))
                        full_list += [x for k, x in enumerate(m_arr[i - 1]) if k % 2]
                        full_list = list(set(full_list))
                        full_list.sort()
                        full_list = [-1] + full_list + [100000000000]
                        m_arr.append(copy(full_list))
                        reachable_nodes.append(node2)
                        i += 1
                    else:
                        walking_nodes.append(node2)
            m_arr = m_arr[::-1]
            arr = arr[::-1]
            reachable_nodes = reachable_nodes[::-1]
            self.pointers_old[node1] = []
            for i in range(len(m_arr)):
                self.pointers_old[node1].append([])
                for j in range(len(m_arr[i])):
                    self.pointers_old[node1][i].append([[]] * len(arr[i]))
                    self.pointers_old[node1][i][j] = [-1] * 2
            for i, l in enumerate(m_arr):
                for j, m in enumerate(m_arr[i]):
                    self.pointers_old[node1][i][j] = [
                        bisect_left(arr[i], m_arr[i][j]),
                        0 if i == len(m_arr) - 1 else bisect_left(m_arr[i + 1], m_arr[i][j]),
                    ]
            self.m_arr_fractional_old[node1] = m_arr
            self.reachable_nodes_old[node1] = reachable_nodes
            self.walking_nodes_old[node1] = walking_nodes
