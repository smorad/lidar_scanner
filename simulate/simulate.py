#!/usr/bin/env python3

import networkx as nx
import graph_constructor
import handhold_detectors
import os
import itertools
import pickle

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

def get_node(g, id):
    for node in g:
        if node.ID == id:
            print('found node ', node)
            return node


class Bot:
    node = None

    def __init__(self, id, start, tether_distance=500):
        self.id = id
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

    ## Generate XYZ file for pics
    with open('../data/handholds.xyz', 'w') as f:
        for n in h1.nodes:
            f.write('{} {} {}\n'.format(int(n.x), int(n.y), int(n.z)))


def multi_bot(bots=4, cache=True):
    print('Loading graph...')
    g = nx.read_gpickle('../data/graph.gpickle')
    print('Loading handholds...')
    h = nx.read_gpickle('../data/handhold.pickle')

    START_NODE_IDS = [
        2164,
        2708,
        653,
        649
    ]

    END_NODE_ID = 111

    start_nodes = [get_node(g, id) for id in START_NODE_IDS]
    end_node = get_node(g, END_NODE_ID)
    assert(len(start_nodes) == 4)

    bots = [Bot(i, node, 500) for i, node in enumerate(start_nodes)] 
    for bot in bots:
        path_nodes = h.get_path(bot.node, end)
        print('{}: {} -> {}'.format(bot.id, bot.node, path_nodes[0]))
        path_nodes[0].occupied = bot
        bot.node.occupied = False
        bot.node = path_nodes[0]
        if bot.node == end_node:
            print('{} reached end node')
            print('final pos: ', [bot.node for bot in bots])
            

if __name__ == '__main__':
    #main()
    #g = write_graph()
    #write_handholds(g)
    #write_handholds()
    multi_bot()
