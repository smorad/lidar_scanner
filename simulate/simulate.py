#!/usr/bin/env python3

import networkx as nx
import graph_constructor
import handhold_detectors
import os

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

if __name__ == '__main__':
    main()
