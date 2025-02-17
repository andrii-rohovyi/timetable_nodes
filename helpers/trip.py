from typing import Tuple, List
from functools import total_ordering
import math


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


class Transfer:

    def __init__(self, nodes: List[int], distance: int, mode: str):
        """
        Walk profile between nodes
        :param nodes: [start_node, end_node]
        :param distance: distance in seconds for walk between stations
        """

        self.nodes = nodes
        self.distance = distance
        self.mode = mode

    def w(self, walk_speed, bicycle_speed):
        if self.mode == 'walk':
            return self.distance / walk_speed
        return self.distance / bicycle_speed

    @property
    def route_names(self):
        return [self.mode]


class WalkTransfer:

    def __init__(self, nodes: List[int], distance: int):
        """
        Walk profile between nodes
        :param nodes: [start_node, end_node]
        :param distance: distance in seconds for walk between stations
        """

        self.nodes = nodes
        self.distance = distance
        self.mode = 'walk'
        self.route_names = ['walk']

    def w(self, walking_speed):
        return self.distance / walking_speed


class BicycleTransfer:

    def __init__(self, nodes: List[int], walk_distance_from: int, bycycle_distance: int, walk_distance_to: int):
        """
        Walk profile between nodes
        :param nodes: [start_node, end_node]
        """

        self.nodes = nodes
        self.walk_distance_from = walk_distance_from
        self.bycycle_distance = bycycle_distance
        self.walk_distance_to = walk_distance_to
        self.mode = 'bicycle'

    def w(self, walking_speed: int, bicycle_speed: int):
        return ((self.walk_distance_from + self.walk_distance_to) / walking_speed
                + self.bycycle_distance / bicycle_speed)


class Transfers:

    def __init__(self, transfers: list = None):
        """
        Transfer profile between nodes
        """

        self.nodes = None
        if transfers is None:
            transfers = []
        self.transfers = transfers
        self.route_names = None

    def w(self, walking_speed, bicycle_speed):
        min_duration = math.inf
        for transfer in self.transfers:
            if transfer.mode == 'walk':
                duration = transfer.w(walking_speed)
                if duration < min_duration:
                    min_duration = duration
                    self.route_names = ['walk']
                    self.nodes = transfer.nodes
            elif transfer.mode == 'bicycle':
                duration = transfer.w(walking_speed, bicycle_speed)
                if duration < min_duration:
                    min_duration = duration
                    self.route_names = ['walk', 'bicycle', 'walk']
                    self.nodes = transfer.nodes

        return min_duration







