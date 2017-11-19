#!/usr/bin/env python3

import networkx as nx
import graph_constructor
import handhold_detectors
import os
import itertools
import pickle
import math

from compute_path import bounded_leg_astar
from graph_utils import compute_cost

TEST = os.getenv('TEST')

if TEST:
    mesh = '../data/mesh1-superdecimated.ply'
else:
    mesh = '../data/mesh1.ply'

def main():
    print('Generating graph...')
    g = graph_constructor.generate_graph(mesh)
    print('Searching for handholds...')
    h = handhold_detectors.Planar(g)

    ## Generate XYZ file for pics
    with open('../data/handholds.xyz', 'w') as f:
        for n in h.get_graph().nodes:
            f.write('{} {} {}\n'.format(int(n.x), int(n.y), int(n.z)))

    print([n for n in g.nodes if n.ID == 310])
        
    # These are good nodes
    if TEST:
        start = [n for n in h.g.nodes 
                if abs(n.x - 553.117615) < 0.01 and
                abs(n.y + -846.342651 < 0.01)]
        end = [n for n in h.g.nodes 
            if abs(n.x + 111.093) < 0.01 and
                abs(n.y - 739.826) < 0.01]
    else:
        start = [n for n in h.g.nodes 
            if abs(n.x - 258.258789) < 0.01 and
            abs(n.y + 835.248047) < 0.01]

        end = [n for n in h.g.nodes
            if abs(n.x + 52.572998) < 0.01 and
            abs(n.y - 1269.827271) < 0.01]
    print('start', *start, 'end', *end)
    with open('../data/shortest_path.xyz', 'w') as f:
        for p in h.get_path(*start, *end):
            f.write('{} {} {}\n'.format(int(p.x), int(p.y), int(p.z)))

def get_node(h, pos):
    '''Given node coords in h, return corresponding node in h.
    This is due to network not allowing the overloading of
    the __eq__ method for nodes
    '''

    for n in h:
        if math.isclose(n.x, pos[0], rel_tol=0.01) and \
                math.isclose(n.y, pos[1], rel_tol=0.01) and \
                math.isclose(n.z, pos[2], rel_tol = 0.01):
            return n

    print('Warning, node ', id, ' not in h')


class Bot:
    node = None

    def __init__(self, id, start, tether_distance=500):
        self.id = id
        self.node = start
        self.tether_distance = tether_distance


def write_graph():
    print('Generating graph...')
    g = graph_constructor.generate_graph(mesh)
    nx.write_gpickle(g, '../data/graph.gpickle')

def write_handholds(g=None):
    if not g:
        g = nx.read_gpickle('../data/graph.gpickle')
    print('Searching for handholds...')
    h = handhold_detectors.Planar(g)
    h1 = h.get_graph()
    with open('../data/handhold.pickle', 'wb+') as f:
        pickle.dump(h, f)

def multi_bot(bots=4, cache=True):
    print('Loading graph...')
    g = nx.read_gpickle('../data/graph.gpickle')
    print('Loading handholds...')
    with open('../data/handhold.pickle', 'rb') as f:
        h = pickle.load(f)

    ## Generate XYZ file for pics
    with open('../data/handholds.xyz', 'w') as f:
        for n in h.g.nodes:
            f.write('{} {} {}\n'.format(n.x, n.y, n.z))

    START_NODE_POS = [
        (417.507, -744.396, -33.9633),
        (433.345, -738.355, -33.6464),
        (423.88, -759.175, -33.5664),
        (440.459, -752.136, -35.0659),
    ]


    END_NODE_POS = (-35.7883, 1210.73, 328.319)

    start_nodes = [get_node(h.g, pos) for pos in START_NODE_POS]
    end_node = get_node(h.g, END_NODE_POS)
    assert(any(start_nodes))

    bots = [Bot(i, node, 500) for i, node in enumerate(start_nodes)] 
    while True:
        for bot in bots:
            #path_nodes = h.get_path(bot.node, end_node)
            path_nodes = bounded_leg_astar(
                h.g, 
                bot.node, 
                end_node, 
                heuristic=compute_cost
            )
            print('{}: {} -> {}'.format(bot.id, bot.node, path_nodes[1]))
            path_nodes[1].occupied = bot
            bot.node.occupied = False
            bot.node = path_nodes[1]
            if bot.node == end_node:
                print('{} reached end node'.format(bot))
                print('final pos: ', [bot.node for bot in bots])
                break
            

if __name__ == '__main__':
    #main()
    #g = write_graph()
    #write_handholds(g)
    #write_handholds()
    multi_bot()
