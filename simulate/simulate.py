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
from graph_utils import compute_cost, euclidean_distance, euclidean_distance_c

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
    def __init__(self, id, start, max_hop_distance=100, tether_distance=250):
        self.id = id
        self.node = start
        self.hop_distance = max_hop_distance
        self.tether_distance = tether_distance
        self.path = []
        self.total_dist = 0
        self.moved = True


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
    #h = handhold_detectors.Maxima(g)
    h = handhold_detectors.Planar(g)
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
        # planar
        (362.98, -747.74, -33.73),
        (365.12, -747.02, -34.31),
        (355.47, -732.96, -31.34),
        (370.75, -724.73, -32.03)
        # Old
#        (-249.14, 27.9, -34.4),
#        (543.18, -696.065, -28.9262),
#        (518.1, -687.459, -23.527),
#        (524.652, -667.315, -20.868),
#        (550.481, -675.239, -23.726)
    ]
    END_NODE_POS = (-139.15, 877.49, 212.1)
    # old
    #END_NODE_POS = (-63.16, -427.12, 69.33)

    if ITOKAWA:
        START_NODE_POS = [
            # MAXIMA
            #(-87.43, 50.58, 111.15),
            #(-90.38, 41, 108.55),
            #(-81.549, 39.78, 110.72),
            #(-70.25, 62.9, 106.7),
            # PLANAR
            (-159.38, -119.26, 15.17),
            (-162.14, -118.72, 14.44),
            (-162.10, -118.41, 15.70),
            (-159.33, -118.92, 16.42)

        ]

        # MAXIMA
        #END_NODE_POS = (140.88, -22.55, 113.61)
        # PLANAR
        #END_NODE_POS = (94.06, 89.07, 74.53)
        #END_NODE_POS = (-69.78, -7.15, -105.94)
        # Works
        #END_NODE_POS = (-161.405, -115.7, 25.83)
        # Works
        #END_NODE_POS = (-135.54, -56.34, 77.56)
        END_NODE_POS = (-61.57, 21.41, 116.34)


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
    bots = [Bot(i, node) for i, node in enumerate(start_nodes)] 
    # init occupied nodes
    for b in bots:
        b.node.occupied = b
    print('bots', [b.node for b in bots])
    turn = 0
    try:
        while True:
            if not any([b.moved for b in bots]):
                raise Exception('No bots moved last turn')
            for bot in bots:
                turn += 1
                bot.path.append((turn, bot.node.x, bot.node.y, bot.node.z))
                try:
                    path_nodes = bounded_leg_astar(
                        h.g, 
                        bot.node, 
                        end_node, 
                        heuristic=compute_cost,
                        bots=[b for b in bots if b.id != bot.id]
                    )
                    bot.moved = True
                except:
                    print(bot.id, 'did not move') 
                    bot.moved = False
                    bot.node.occupied = bot
                    continue
                print('{}: {}: {} -> {}'.format(len(bot.path), bot.id, bot.node, path_nodes[1]))
                bot.node.occupied = False
                bot.total_dist += euclidean_distance(bot.node, path_nodes[1])
                bot.node = path_nodes[1]
                path_nodes[1].occupied = bot
                if end_node.occupied: 
                    raise Found
    except Found:
        print('Bot {} reached end node'.format(end_node.occupied.id))
        print('final pos: ', [bot.node for bot in bots])
        for n in h.g.nodes:
            if n.occupied:
                print(n, 'is occupied')
        for bot in bots:
            import compute_path
            x, y, z = compute_path.compute_hub_pos(bots)
            print('bot {} is {} units away from hub'.format(
                bot.id, euclidean_distance_c(bot.node, (x, y, z))))
        print('Saving paths...')
        for bot in bots:
            # Print stats
            print('{} moved {} total distance units'.format(bot.id, bot.total_dist))
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
