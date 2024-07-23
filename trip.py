from typing import Tuple, List
from functools import total_ordering


@total_ordering
class Bus:

    def __init__(self, nodes: List[int], route_names: List[str], c: Tuple[int, int]):
        """
        Bus profile for transportation between the nodes
        :param nodes: [start_node, end_node]
        :param route_names: Names of public transport, which we use for transferring
        :param c: (departure_time, arrival_time)
        """

        self.nodes = nodes
        self.d, self.a = c
        self.route_names = route_names

    def __lt__(self, other):
        """
        Lower means that one bus has smaller departure_time
        :param other:
        :return:
        """
        return self.d < other.d

    def __eq__(self, other):
        """
        Equality means that 2 buses have the same start_time
        :param other:
        :return:
        """
        return self.d == other.d


class Walk:

    def __init__(self, nodes: List[int], w: int):
        """
        Walk profile between nodes
        :param nodes: [start_node, end_node]
        :param w: duration in seconds for walk between stations
        """

        self.nodes = nodes
        self.w = w
        self.route_names = ['walk'] * (len(nodes) - 1)



