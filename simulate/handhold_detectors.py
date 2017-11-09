import networkx as nx
import itertools
import math
import numpy
from sklearn import linear_model, model_selection

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

def euclidean_neighbors(graph, node, distance):
    '''
    Return all nodes that form an ego graph of a node. In other words
    return the all nodes within given distance of the given node
    '''
    g = nx.ego_graph(graph, node, distance, center=False, distance='weight')
    #print('Neighbors of size {}'.format(g.number_of_nodes()))
    return g.nodes

def assign_flatness(graph, node, distance=3):
    '''
    Given a node, assigns it a score denoting how flat the immediate
    area around the node is. The lower the score, the flatter it is, with
    a score of 0 meaning the surface is a plane.
    '''
    #TODO search for all possible planes instead of x+y
    # Generate normal of surface, align normal with Z axis
    #https://stackoverflow.com/questions/1023948/rotate-normal-vector-onto-axis-plane
    #https://math.stackexchange.com/questions/17246/is-there-a-way-to-rotate-the-graph-of-a-function
    neighbors = euclidean_neighbors(graph, node, distance)
    X = [(n.x, n.y) for n in neighbors]
    t = [n.z for n in neighbors]
    
    #X = [(n.x, n.y) for n in graph.neighbors(node)]
    #t = [n.z for n in graph.neighbors(node)]
    regression = linear_model.LinearRegression()
    model = regression.fit(X, t)
    #print('coef', regression.coef_)
    
    # find normal to plane
    # since coef is just the int, ie 4 for 4x, it is already the deriv
    # for a planar surface
    model_gradient = (regression.coef_[0], regression.coef_[1], regression.coef_[0] + regression.coef_[1])
    #print('model grad', model_gradient)

    # What if instead of machine learning, we take every grad at every point and
    # minimize xproduct, maybe compare it to ml'ed surface normal?
    vectors = [(n.x, n.y, n.z) for n in graph.neighbors(node)]
    point_gradients = numpy.gradient(vectors)
    # fitness
    # square for positive
    #print(point_gradients[0])
    # there are far too many things called "norm"
    node.loss = sum([abs(numpy.linalg.norm(numpy.cross(p_grad, model_gradient))) for p_grad in point_gradients])
    #print('loss', node.loss)


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
        g = nx.Graph()
        g.add_nodes_from(nodes)
        # itertools does something weird here where it empties nodes, so deepcopy
        all_possible_edges = list(itertools.permutations(nodes, 2))
        [g.add_edge(*edge, weight=euclidean_distance(*edge) + loss_weight * edge[1].loss ) 
            for edge in all_possible_edges]
        self.g = nx.minimum_spanning_tree(g)


    def get_path(self, start_node=None, goal_node=None):
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
        return nx.shortest_path(self.g, start_node, goal_node)

class Planar(HandHoldGraph):
    '''For microspine grippers. Search for a flat, planar surface.'''

    def get_graph(self, percentile=10):
        '''Get emst with the flattest surfaces'''
        assert(percentile > 0 and percentile < 100)

        print('Computing flatness for {} nodes...'.format(self._g.number_of_nodes()))
        count = 0
        for node in self._g.nodes:
            assign_flatness(self._g, node)
            count += 1
            if not count % 100:
                print(
                    'Computed flatness for {} / {} nodes'
                    .format(count, self._g.number_of_nodes()))

        [assign_flatness(self._g, node) for node in self._g.nodes]
        node_losses = sorted(self._g.nodes, key=lambda x: x.loss)
        divider = int(percentile / 100 * len(node_losses))
        nodes = node_losses[:divider + 1]
        self._build_emst(nodes, 0.1)
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


