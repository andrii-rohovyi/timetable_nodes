import pandas as pd
from collections import defaultdict

from helpers.trip import WalkTransfer


class CustomizedRAPTOR_DS:
    def __init__(self,
                 transport_connections: pd.DataFrame,
                 walk_connections: pd.DataFrame):
        """
        Customized RAPTOR data structure

        Parameters
        ----------
        :param transport_connections: pd.DataFrame. File format related to network_temporal_day.csv by link: https://zenodo.org/records/1136378
        :param walk_connections: File format related to network_walk.csv by link: https://zenodo.org/records/1136378
        """

        transport_connections = transport_connections.sort_values(by=['dep_time_ut', 'arr_time_ut'])
        transport_connections_fixed_trips = transport_connections.copy()
        transport_connections_fixed_trips['seq'] = transport_connections_fixed_trips.groupby("trip_I").cumcount() + 1
        transport_connections_fixed_trips['tmp'] = (transport_connections_fixed_trips['from_stop_I'].astype(str) + '_'
                                                    + transport_connections_fixed_trips['seq'].astype(str) + '_'
                                                    + transport_connections_fixed_trips['to_stop_I'].astype(str) + '_'
                                                    + transport_connections_fixed_trips['route_type'].astype(str)
                                                    )
        trips_merged = transport_connections_fixed_trips.groupby('trip_I').agg({'tmp': set}).reset_index()
        trips_merged['tmp'] = trips_merged['tmp'].astype(str)
        routes = trips_merged.groupby('tmp').agg({'trip_I': set})['trip_I'].tolist()
        self.route_dict = {}
        for i, key in enumerate(routes):
            for trip in routes[i]:
                self.route_dict[trip] = i + 1
        transport_connections_fixed_trips['route_I'] = transport_connections_fixed_trips['trip_I'].map(self.route_dict)

        self.route_match = {}
        for i, x in enumerate(transport_connections_fixed_trips['route_I'].values):
            self.route_match[x] = transport_connections['route_I'].iloc[i]

        stop_to_route = transport_connections_fixed_trips.melt(
            value_vars=['from_stop_I', 'to_stop_I'],
            id_vars=['route_I'],
            value_name='stop'
        )[['stop', 'route_I']]

        # Group by stop and aggregate routes into sets
        self.stop_to_route = stop_to_route.groupby('stop')['route_I'].agg(set).to_dict()

        self.idx_by_route_stop_dict = transport_connections_fixed_trips.set_index(
            ['route_I', 'to_stop_I'])['seq'].to_dict()
        self.stops_dict = transport_connections_fixed_trips.set_index(['route_I', 'seq'])['to_stop_I'].to_dict()
        zero_stops_dict = transport_connections_fixed_trips[(transport_connections_fixed_trips['seq'] == 1)].set_index('route_I')[
            'from_stop_I'].to_dict()
        stops_dict_keys = list(self.stops_dict.keys())
        for i in stops_dict_keys:
            self.stops_dict[(i[0], 0)] = zero_stops_dict[i[0]]
        self.routes_lens = transport_connections_fixed_trips.groupby(by='route_I').agg({'seq': 'max'})['seq'].to_dict()
        self.departure_times = transport_connections_fixed_trips.set_index(['trip_I', 'from_stop_I'])['dep_time_ut'].to_dict()
        self.trip_to_time = transport_connections_fixed_trips.set_index(['trip_I', 'seq'])['arr_time_ut'].to_dict()
        self.trip_finder = transport_connections_fixed_trips.groupby(['route_I', 'from_stop_I']).agg(
            {'dep_time_ut': list, 'trip_I': list}).to_dict()

        walk_connections_dict = walk_connections.set_index(['from_stop_I', 'to_stop_I'])['d_walk'].to_dict()

        self.walk_graph = defaultdict(dict)

        for adjacent_node, node in set(walk_connections_dict.keys()):
            walk_distance = walk_connections_dict.get((adjacent_node, node))
            if walk_distance is not None:
                self.walk_graph[adjacent_node][node] = WalkTransfer(nodes=[adjacent_node, node], distance=walk_distance)
