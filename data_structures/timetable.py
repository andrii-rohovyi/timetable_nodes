import pandas as pd
import numpy as np
from tqdm import tqdm
from collections import defaultdict

from helpers.trip import WalkTransfer, BicycleTransfer, Transfers


class Timetable:
    def __init__(self,
                 transport_connections: pd.DataFrame,
                 walk_connections: pd.DataFrame):
        """
        Class, which represent Multimodal Transport Network as a Timetable for CSA.
        It is __init__ by 2 data frames about transport and walk information over the city
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param walk_connections: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """

        transport_connections_df = transport_connections.copy(deep=True)
        transport_connections_df = transport_connections_df.sort_values(by=['dep_time_ut', 'arr_time_ut'])
        transport_connections_df['route_I'] = transport_connections_df['route_I'].astype(str)
        self.transport_connections_df = transport_connections_df[['from_stop_I', 'to_stop_I', 'dep_time_ut',
                                                                  'arr_time_ut', 'route_I']].to_dict('records')
        self.departure_times = transport_connections_df['dep_time_ut'].tolist()

        walk_connections_dict = walk_connections.set_index(['from_stop_I', 'to_stop_I'])['d_walk'].to_dict()
        self.walk_graph = defaultdict(dict)

        for adjacent_node, node in set(walk_connections_dict.keys()):
            walk_duration = walk_connections_dict.get((adjacent_node, node))
            if walk_duration is not None:
                self.walk_graph[adjacent_node][node] = walk_duration


class CustomizedTimetable:
    def __init__(self,
                 transport_connections: pd.DataFrame,
                 walk_connections: pd.DataFrame):
        """
        Class, which represent Multimodal Transport Network as a Timetable for CSA.
        It is __init__ by 2 data frames about transport and walk information over the city
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param walk_connections: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """

        transport_connections_df = transport_connections.copy(deep=True)
        transport_connections_df = transport_connections_df.sort_values(by=['dep_time_ut', 'arr_time_ut'])
        transport_connections_df['route_I'] = transport_connections_df['route_I'].astype(str)
        self.transport_connections_df = transport_connections_df[['from_stop_I', 'to_stop_I', 'dep_time_ut',
                                                                  'arr_time_ut', 'route_I']].to_dict('records')
        self.departure_times = transport_connections_df['dep_time_ut'].tolist()

        walk_connections_dict = walk_connections.set_index(['from_stop_I', 'to_stop_I'])['d_walk'].to_dict()
        self.walk_graph = defaultdict(dict)

        for adjacent_node, node in set(walk_connections_dict.keys()):
            walk_distance = walk_connections_dict.get((adjacent_node, node))
            if walk_distance is not None:
                self.walk_graph[adjacent_node][node] = WalkTransfer(nodes=[adjacent_node, node], distance=walk_distance)


class CustomizedTimetableBicycle:
    def __init__(self,
                 transport_connections: pd.DataFrame,
                 transfer_connections: pd.DataFrame):
        """
        Class, which represent Multimodal Transport Network as a Timetable for CSA.
        It is __init__ by 2 data frames about transport and walk information over the city
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param transfer_graph: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """

        transport_connections_df = transport_connections.copy(deep=True)
        transport_connections_df = transport_connections_df.sort_values(by=['dep_time_ut', 'arr_time_ut'])
        transport_connections_df['route_I'] = transport_connections_df['route_I'].astype(str)
        self.transport_connections_df = transport_connections_df[['from_stop_I', 'to_stop_I', 'dep_time_ut',
                                                                  'arr_time_ut', 'route_I']].to_dict(
            'records')
        self.departure_times = transport_connections_df['dep_time_ut'].tolist()

        self.transfer_graph = defaultdict(dict)
        transfer_connections_pairs = set(map(tuple,
                                             transfer_connections[['node_from', 'node_to']].drop_duplicates().values))

        for adjacent_node, node in tqdm(transfer_connections_pairs):
            transfers = []
            for index, row in transfer_connections[(transfer_connections['node_from'] == adjacent_node)
                                                   & (transfer_connections['node_to'] == node)].iterrows():
                if np.isnan(row['bicycle_node_to']):
                    transfer = WalkTransfer(nodes=[adjacent_node, node], distance=row['walking_distance'])
                else:
                    transfer = BicycleTransfer(nodes=[adjacent_node, row['bicycle_node_from'],
                                                      row['bicycle_node_to'], node],
                                               walk_distance_from=row['walk_distance_from'],
                                               bycycle_distance=row['bycycle_distance'],
                                               walk_distance_to=row['walk_distance_to'])
                transfers.append(transfer)
            if transfers:
                transfers_class = Transfers(transfers=transfers)
            else:
                transfers_class = None
            if transfers_class is not None:
                self.transfer_graph[adjacent_node][node] = transfers_class
