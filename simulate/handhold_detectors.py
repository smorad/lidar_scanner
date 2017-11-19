import networkx as nx
import itertools
import math
import operator
import numpy
from sklearn import linear_model, model_selection
import multiprocessing
import sys

from compute_path import bounded_leg_astar
import phase_timer
from graph_utils import euclidean_distance, compute_cost


def find_local_maximum(graph, node):
    heights = [neighbor.z for neighbor in graph.neighbors(node)]
    is_maximum = all([node.z >= height for height in heights])
    if is_maximum:
        return node
    return None

def euclidean_neighbors(graph, node, distance):
    '''
    Return all nodes that form an ego graph of a node. In other words
    return the all nodes within given distance of the given node
    '''
    g = nx.ego_graph(graph, node, distance, center=False, distance='weight')
    return g.nodes


def compute_flatness(graph, node, distance=10):
    '''
    Given a node, return a score denoting how flat the immediate
    area around the node is. The lower the score, the flatter it is, with
    a score of 0 meaning the surface is a plane.
    '''
    neighbors = euclidean_neighbors(graph, node, distance)
    X = [(n.x, n.y) for n in neighbors]
    t = [n.z for n in neighbors]
    
    regression = linear_model.LinearRegression()
    # Get the best fit plane to the surface
    model = regression.fit(X, t)
    
    # find normal to plane
    # since coef is just the int, ie 4 for 4x, it is already the deriv for
    # a planar surface (linear surface)
    # z deriv is just d/dx(f(x)) + d/dy(f(y))
    model_gradient = (regression.coef_[0], regression.coef_[1], regression.coef_[0] + regression.coef_[1])

    node_surface = [(n.x, n.y, n.z) for n in graph.neighbors(node)]
    # Get the point gradient at each point in the surface
    point_gradients = numpy.gradient(node_surface)
    # We don't need to ensure that the normals are pointing towards the positive z direction
    # because the xproduct of vectors in opposite directions is also 0.
    # However, the cross product between the plane normal and all the surface normals
    # may be pointing in the -z direction.
    # This is handled by taking the norm, which also serves to gives us a nice loss function
    return sum(
        [numpy.linalg.norm(numpy.cross(p_grad, model_gradient)) 
        for p_grad in point_gradients]
    )


class HandHoldGraph:
    def __init__(self, g: nx.Graph):
        # Dont modify original graph
        self._g = g.copy()

    def get_graph(self):
        '''
        Return a graph of potential climbing holds
        '''
        raise NotImplementedError()

    def _build_emst(self, nodes, loss_weight=0):
        '''
        Given a list of nodes, builds as complete euclidean graph.
        Returns the euclidean minimum spanning tree of the graph.
        '''
        g = nx.DiGraph()
        g.add_nodes_from(nodes)
        # itertools does something weird here where it empties nodes, so deepcopy
        # edges must be bidirectional, as weight from a good node to bad node
        # should be different than a bad node to good node
        all_possible_edges = list(itertools.permutations(nodes, 2))
        for edge in all_possible_edges:
            g.add_edge(*edge, weight=compute_cost(*edge))
            
        self.g = nx.minimum_spanning_tree(g)

    def guess_edges(self, nodes, neighbor_len=20):
        '''
        Given a bunch of nodes in 3d space, guess some edges for each node.
        The optimal way would be to generate a complete graph (where every node
        is connected to every other node), but the minimum spanning aborescence
        causes tons of page faults and thrashes my disk. 
        '''
        edges = []
        for node in nodes:
            closest_nodes = sorted(nodes, key=lambda x: euclidean_distance(node, x))[:neighbor_len]
            edges += [(node, neighbor) for neighbor in closest_nodes]
        return edges

    def build_edges(self, nodes, risk_weight=1):
        '''
        Given a list of nodes, builds as complete euclidean graph.
        Returns the aborescence (directed analog of euclidean minimum 
        spanning tree) of the graph.
        '''
        print('Building edges...')
        with phase_timer.Timer():
            g = nx.DiGraph()
            g.add_nodes_from(nodes)
            edges = self.guess_edges(nodes)
            # three-ple of node, node, weight
            ebunch = [
                (edge[0], edge[1], compute_cost(*edge))
                for edge in edges
            ]
            g.add_weighted_edges_from(ebunch)
            self.g = g
            

    def get_path(self, start_node=None, goal_node=None, risk_weight=0.1):
        '''
        Return a path of nodes
        '''
        if not (start_node or goal_node):
            for node in self.g.nodes:
                if not goal_node:
                    goal_node = node
                    start_node = node
                if node.y > goal_node.y:
                    goal_node = node
                if node.y < goal_node.y:
                    start_node = node
        # Can't pickle lambdas
        return bounded_leg_astar(
            self.g, start_node, goal_node, 
            # Weight is a tuple of (distance, risk)
            # Bounded leg will take a bound for max leg length
            # lambda x, y: euclidean_distance(x, y) + y.loss * risk_weight
            compute_cost
        )

class Planar(HandHoldGraph):
    '''For microspine grippers. Search for a flat, planar surface.'''

    def get_graph(self, percentile=15):
        '''Get emst with the flattest surfaces'''
        assert(percentile > 0 and percentile < 100)

        print('Computing flatness for {} verticies...'.format(self._g.number_of_nodes()))
        with phase_timer.Timer():
            # sklearn is super duper slow to generate linear models
            # sidestep python GIL, and raise the temp of my room 5 degrees
            p = multiprocessing.Pool(16)
            flatness_losses = p.starmap(
                    compute_flatness, [(self._g, node) for node in self._g.nodes])
            # I hope the node iterator is ordered...
            # For decimated example, flatness losses vary between 25 and 3000
            for idx, node in enumerate(self._g.nodes):
                node.loss = flatness_losses[idx]
            loss_sorted_nodes = sorted(self._g.nodes, key=lambda x: x.loss)
            divider = int(percentile / 100 * len(loss_sorted_nodes))
            best_nodes = loss_sorted_nodes[:divider + 1]
        print('Found {} candidate handholds'.format(len(best_nodes)))
        self.build_edges(best_nodes, 0.1)
        return self.g


class Maxima(HandHoldGraph):

    def get_graph(self):
        '''
        Return a graph of handholds
        '''
        # Filter non maxima, use lambda instead of bool 
        # because 0 could potentially be a max
        maxima = filter(
            lambda x: x != None, 
            [find_local_maximum(self._g, node) for node in self._g.nodes]
        )

        # For some reason if we leave maxima as a filter object weird things 
        # happen, like the list being empty after iterating thru it
        maxima = list(maxima)
        self._build_emst(maxima)
        return self.g


