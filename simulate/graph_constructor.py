#!/usr/bin/env python3

import plyfile
import networkx as nx
import itertools

# TODO fix disgusting code



class Node:
    registry = []
    def __init__(self, ID, args):
        self.ID = ID
        self.x = args[0]
        self.y = args[1]
        self.z = args[2]
        self.loss = 0
    
        Node.registry.append(self)

    def __repr__(self):
        return 'Node {}:({:.01f}, {:.01f}, {:.01f})'.format(self.ID, self.x, self.y, self.z)

def generate_graph(path: str):
    '''
    Generate a graph data structure from a mesh polygon file
    '''
    p = plyfile.PlyData.read(path)
    # format is x, y, z, nx, ny, nz
    nodes = [Node(index, vertex) for index, vertex in enumerate(p['vertex'])]
    # convert faces to edges
    faces = [face[0] for face in p['face']]

    edges = []
    for face in faces:
        edges += face_to_edge(face)

    G = nx.Graph()
    G.add_nodes_from(nodes)
    edges = associate_edges(edges)
    G.add_edges_from(edges)

    return G

# Associate edge with Node object instead of just index
def associate_edges(edges):
    edge_objs = []
    for a, b in edges:
        #print('Associating {} {} with {} {}'.format(a, b, Node.registry[a], Node.registry[b]))
        edge_objs.append((Node.registry[a], Node.registry[b]))

    return edge_objs

# collapse a face (3 tuple of edges) into a proper edge (2 tuple)
def face_to_edge(face):
    a, b, c = face
    # undirected graph, so use replacement
    return list(itertools.permutations([a, b, c], 2))

if __name__ == '__main__':
    generate_graph('../meshes/mesh1.ply')
