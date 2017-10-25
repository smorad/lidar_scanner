import networkx as nx
import itertools
import math

def euclidean_distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 + 
        (a.z - b.z) ** 2
    )

def find_local_maximum(graph, node):
    heights = [neighbor.z for neighbor in graph.neighbors(node)]
    is_maximum = all([node.z >= height for height in heights])
    if is_maximum:
        return node
    return None

class HandHoldGraph:
    def __init__(self, g: nx.Graph):
        # Dont modify original graph
        self.g = g.copy()

    def get_graph(self):
        '''
        Return a graph of handholds
        '''
        raise NotImplementedError()

class Maxima(HandHoldGraph):

    def get_graph(self):
        '''
        Return a graph of handholds
        '''
        # Filter non maxima, use lambda instead of bool 
        # because 0 could potentially be a max
        maxima = filter(
            lambda x: x != None, 
            [find_local_maximum(self.g, node) for node in self.g.nodes]
        )

        # Form a perfectly connected geometric graph, then remove 
        # extra edges using euclidian MST, TODO make this more efficient

        # For some reason if we leave maxima as a filter object weird things 
        # happen, like the list being empty after iterating thru it
        maxima = list(maxima)
        new_g = nx.Graph()
        new_g.add_nodes_from(maxima)
        # itertools does something weird here where it empties maxima, so deepcopy
        all_possible_edges = list(itertools.permutations(maxima, 2))
        print('edges', list(all_possible_edges)[:3])
        print('nodes', list(new_g.nodes)[:3])
        [new_g.add_edge(*edge, weight=euclidean_distance(*edge)) 
            for edge in all_possible_edges]
        for a, b in all_possible_edges:
            print ('{}->{}'.format(a.ID, b.ID))
        #new_g.add_weighted_edges_from(all_possible_edges)
        print('all edges and nodes', len(list(new_g.edges)), len(list(new_g.nodes)))
        return nx.minimum_spanning_tree(new_g)


