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
from graph_utils import compute_cost

class Found(Exception): pass

TEST = os.getenv('TEST')
REGEN_GRAPH = os.getenv('GRAPH')
REGEN_HOLDS = os.getenv('HOLDS')

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
        g = nx.read_gpickle('../data/graph.gpickle')
    print('Searching for handholds...')
    h = handhold_detectors.Planar(g)
    h.get_graph()
    with open('../data/handhold.pickle', 'wb+') as f:
        pickle.dump(h, f)

def multi_bot(bots=4, cache=True):
    '''
    Have multiple tethered robots attempt to get to
    the goal node. Boths can't occupy the same space
    at the same time, or stray more than
    tether_distance from the other bots.help
    '''
    print('Loading graph...')
    g = nx.read_gpickle('../data/graph.gpickle')
    print('Loading handholds...')
    with open('../data/handhold.pickle', 'rb') as f:
        h = pickle.load(f)

    ## Generate XYZ file for pics
    with open('../data/handholds.xyz', 'w') as f:
        for n in h.g.nodes:
            f.write('{} {} {}\n'.format(n.x, n.y, n.z))

    # These look like good node positions
    START_NODE_POS = [
        #(417.507, -744.396, -33.9633),
        #(433.345, -738.355, -33.6464),
        #(423.88, -759.175, -33.5664),
        #(440.459, -752.136, -35.0659),
        (543.18, -696.065, -28.9262),
        (518.1, -687.459, -23.527),
        (524.652, -667.315, -20.868),
        (550.481, -675.239, -23.726)
    ]


    END_NODE_POS = (-26.453, 1181.19, 337.438)#(-35.7883, 1210.73, 328.319)

    start_nodes = [get_node(h.g, pos) for pos in START_NODE_POS]
    end_node = get_node(h.g, END_NODE_POS)
    # Make sure that we actually find the nodes...
    assert(any(start_nodes))

    # Run a simulation with bots tethered to each other
    # they can't occupy the same space, and cannot go further
    # than Bot.tether_length from the other bots
    bots = [Bot(i, node, 500) for i, node in enumerate(start_nodes)] 
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
                    heuristic=compute_cost
                )
                print('{}: {}: {} -> {}'.format(len(bot.path), bot.id, bot.node, path_nodes[1]))
                bot.node.occupied = False
                bot.node = path_nodes[1]
                path_nodes[1].occupied = bot
                if path_nodes[1].ID == end_node.ID:
                    print('Reached goal')
                    # Make sure we add the goal node before terminating
                    bot.path.append((turn, bot.node.x, bot.node.y, bot.node.z))
                    raise Found
    except Found:
        print('{} reached end node'.format(end_node.occupied.id))
        print('final pos: ', [bot.node for bot in bots])
        print('Saving paths...')
        for bot in bots:
            with open('../data/bot{}-path'.format(bot.id), 'w+') as f:
                for move in bot.path:
                    f.write('{} {} {} {}\n'.format(bot.id, move[1], move[2], move[3]))
            with open('../data/bot{}-path.xyz'.format(bot.id), 'w+') as f:
                for move in bot.path: 
                    f.write('{} {} {}\n'.format(move[1], move[2], move[3]))

if __name__ == '__main__':
    if REGEN_GRAPH:
        g = write_graph()
        write_handholds(g)
    if not REGEN_GRAPH and REGEN_HOLDS:
        write_handholds()
    multi_bot()
