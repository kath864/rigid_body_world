from abc import ABC, abstractmethod
import networkx as nx

class World(ABC):

    @property
    @abstractmethod
    def graph(self):
        """
        Returns
        -------
        nx.Graph
            The undirected graph describing the world
        """
        pass

    @property
    def objects(self):
        """
        Returns
        -------
        dict
            Object nodes
        """
        d = self.graph.nodes(data = True)
        return OrderedDict(sorted(d, key=lambda t: t[0]))

    @abstractmethod
    def add_object(self):
        """Adds objects to the world"""
        pass

    def add_field(self, name, field):
        self.graph.add_node(name, **field.serialize())
        edges = field.apply_field(self.graph)
        for (target, data) in edges:
            self.graph.add_edge(name, target, **data)


    def serialize(self, indent = None):
        """
        Returns
        -------
        str
            a json-serialized JIT parsing of the world graph
        """
        return nx.jit_data(self.graph, indent = indent)

    def load(self, data):
        """
        Parameters
        ----------
        data : str
            JIT data to load
        """
        self.graph = nx.jit_graph(data)


    def write(self, path):
        with open(path, 'w') as f:
            f.write(self.serialize())


def parse_world_graph(data):
    """ Parses a JIT-serialized string to a graph
    Parameters
    ----------
    data : (str)
        A JIT serialized message contain nodes and edge info

    Returns
    -------
    nx.DiGraph
        A directed graph representing the scene
    """
    return nx.jit_graph(data, nx.DiGraph())


# def safe_jit_data(G, **kwargs):
#     """Returns data in JIT JSON format.

#     Parameters
#     ----------
#     G : NetworkX Graph

#     **kwargs: Addition parameters for :func:json.dumps

#     Returns
#     -------
#     data: JIT JSON string
#     """
#     json_graph = []
#     for node in G.nodes():
#         json_node = {
#             "id": node,
#             "name": node
#         }
#         # node data
#         json_node["data"] = G.nodes[node]
#         # adjacencies
#         if G[node]:
#             json_node["adjacencies"] = []
#             for neighbour in G[node]:
#                 adjacency = {
#                     "nodeTo": neighbour,
#                 }
#                 # adjacency data
#                 adjacency["data"] = G.edges[node, neighbour]
#                 json_node["adjacencies"].append(adjacency)
#         json_graph.append(json_node)
#     return json.dumps(json_graph, **kwargs)
