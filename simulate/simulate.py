#!/usr/bin/env python3

# This is a dumping ground for stuff that I'm too lazy to put into a class
# beware, thar be strange things ahead
# Run with $ TEST=true ./simulate
# to run used reduced data, so it doesn't take an hour
# and OOM your machine

import networkx as nx
import graph_constructor
import handhold_detectors
import os
import itertools
import pickle
import math
import matplotlib

from compute_path import bounded_leg_astar
from graph_utils import compute_cost, euclidean_distance

class Found(Exception): pass

DATA_DIR = os.path.dirname(os.path.realpath(__file__)) + '/../data/'
TEST = os.getenv('TEST')
REGEN_GRAPH = os.getenv('GRAPH')
REGEN_HOLDS = os.getenv('HOLDS')
ITOKAWA = os.getenv('ITOKAWA')

if TEST:
    mesh = DATA_DIR + 'mesh1-superdecimated.ply'
elif ITOKAWA:
    mesh = DATA_DIR + 'itokawa_f0196608.ply'
else:
    mesh = DATA_DIR + 'mesh1.ply'

print('Loading mesh {}...'.format(mesh))

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

    print('Warning, node ', pos, ' not in h')


class Bot:
    def __init__(self, id, start, tether_distance=250):
        self.id = id
        self.node = start
        self.tether_distance = tether_distance
        self.path = []


def write_graph():
    '''Only call me if you generated a new mesh'''
    print('Generating graph...')
    g = graph_constructor.generate_graph(mesh)
    nx.write_gpickle(g, '../data/graph.gpickle')
    return g

def write_handholds(g=None):
    '''
    Only call me if you changed 
    the handhold detector code
    '''
    if not g:
        g = nx.read_gpickle(DATA_DIR + 'graph.gpickle')
    print('Searching for handholds...')
    h = handhold_detectors.Maxima(g)
    h.get_graph()
    with open(DATA_DIR + 'handhold.pickle', 'wb+') as f:
        pickle.dump(h, f)

def multi_bot(bots=4, cache=True):
    '''
    Have multiple tethered robots attempt to get to
    the goal node. Boths can't occupy the same space
    at the same time, or stray more than
    tether_distance from the other bots.help
    '''
    print('Loading graph...')
    g = nx.read_gpickle(DATA_DIR + 'graph.gpickle')
    print('Loading handholds...')
    with open(DATA_DIR + 'handhold.pickle', 'rb') as f:
        h = pickle.load(f)

    ## Generate XYZ file for pics
    with open(DATA_DIR + 'handholds.xyz', 'w') as f:
        for n in h.g.nodes:
            f.write('{} {} {}\n'.format(n.x, n.y, n.z))

    # These look like good node positions
    START_NODE_POS = [
        (-249.14, 27.9, -34.4),
        (543.18, -696.065, -28.9262),
        (518.1, -687.459, -23.527),
        (524.652, -667.315, -20.868),
        (550.481, -675.239, -23.726)
    ]
    END_NODE_POS = (-26.453, 1181.19, 337.438)

    if ITOKAWA:
        START_NODE_POS = [
            (-253.16, -12.39, -14.45),
            (-252.87, -6.67, -16.96),
            (-253.90, -10.44, -9.77),
            (-253.79, -3.99, -10.65)
        ]

        #END_NODE_POS = (127.71, 67.21, 68.44)
        END_NODE_POS = (33.33, 114.62, 16.09)


    #sorted_nodes = sorted(h.g.nodes(), key=lambda n: n.x ** 2 + n.y ** 2 + n.z ** 2)
    print(h.g.number_of_nodes())
    print(h.g.number_of_edges())
    start_nodes = [get_node(h.g, pos) for pos in START_NODE_POS]
    #start_nodes = sorted_nodes[:4]
    end_node = get_node(h.g, END_NODE_POS)
    #end_node = sorted_nodes[250]
    # Make sure that we actually find the nodes...
    assert(any(start_nodes))

    # Run a simulation with bots tethered to each other
    # they can't occupy the same space, and cannot go further
    # than Bot.tether_length from the other bots
    bots = [Bot(i, node, 350) for i, node in enumerate(start_nodes)] 
    print('bots', [b.node for b in bots])
    turn = 0
    try:
        while True:
            for bot in bots:
                turn += 1
                bot.path.append((turn, bot.node.x, bot.node.y, bot.node.z))
                path_nodes = bounded_leg_astar(
                    h.g, 
                    bot.node, 
                    end_node, 
                    heuristic=compute_cost,
                    bots=[b for b in bots if b.id != bot.id]
                )
                print('{}: {}: {} -> {}'.format(len(bot.path), bot.id, bot.node, path_nodes[1]))
                bot.node.occupied = False
                bot.node = path_nodes[1]
                path_nodes[1].occupied = bot
                if end_node.occupied: 
                    raise Found
    except Found:
        print('Bot {} reached end node'.format(end_node.occupied.id))
        print('final pos: ', [bot.node for bot in bots])
        for bot1 in bots:
            for bot2 in bots:
                if bot1 == bot2:
                    continue
                print('bot {} is {} units away from {}'.format(
                    bot1.id, euclidean_distance(bot1.node, bot2.node), bot2.id))
        print('Saving paths...')
        for bot in bots:
            # Save paths for analysis
            with open(DATA_DIR + 'bot{}-path'.format(bot.id), 'w+') as f:
                for move in bot.path:
                    f.write('{} {} {} {}\n'.format(bot.id, move[1], move[2], move[3]))
            with open(DATA_DIR + 'bot{}-path.xyz'.format(bot.id), 'w+') as f:
                for move in bot.path: 
                    f.write('{} {} {}\n'.format(move[1], move[2], move[3]))

if __name__ == '__main__':
    if REGEN_GRAPH:
        g = write_graph()
        write_handholds(g)
    if not REGEN_GRAPH and REGEN_HOLDS:
        write_handholds()
    multi_bot()
