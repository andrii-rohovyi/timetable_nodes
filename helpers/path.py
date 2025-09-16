

class Path:

    def __init__(self, sequence_nodes, sequence_route_names, cost, transfers_numbers=None):
        """
        Path to node. Path is characterized by a sequence of nodes.

        Parameters
        ----------
        sequence_nodes: list of nodes
        sequence_route_names: list of route names
        cost: cost characterized the cost to specific node
        transfers_numbers: amount of public transports involved in trip
        """

        self.sequence_nodes = sequence_nodes
        self.sequence_route_names = sequence_route_names
        self.cost = cost
        self.transfers_numbers = transfers_numbers
