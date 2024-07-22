import pandas as pd
from collections import defaultdict


class Timetable:
    def __init__(self,
                 transport_connections: pd.DataFrame,
                 walk_connections: pd.DataFrame):
        """
        Class, which represent Multimodal Transport Network.
        It is __init__ by 2 data frames about transport and walk information over the city
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param walk_connections: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """

        transport_connections_df = transport_connections.copy(deep=True)
        transport_connections_df = transport_connections_df.sort_values(by=['dep_time_ut', 'arr_time_ut'])
        transport_connections_df['route_I'] = transport_connections_df['route_I'].astype(str)
        self.transport_connections_df = transport_connections_df.copy()
        self.departure_times = self.transport_connections_df['dep_time_ut'].tolist()

        walk_connections_dict = walk_connections.set_index(['from_stop_I', 'to_stop_I'])['d_walk'].to_dict()

        self.graph = defaultdict(dict)
        self.walk_graph = defaultdict(dict)
        self.f = defaultdict(dict)

        for adjacent_node, node in set(walk_connections_dict.keys()):
            walk_duration = walk_connections_dict.get((adjacent_node, node))
            if walk_duration is not None:
                self.walk_graph[adjacent_node][node] = walk_duration
