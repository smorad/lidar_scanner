#!/usr/bin/env python3

import networkx as nx
import graph_constructor
import handhold_detectors

def main():
    print('Generating graph...')
    g = graph_constructor.generate_graph('../data/mesh1-decimated.ply')
    print('Searching for handholds...')
#    h = handhold_detectors.Maxima(g)
    h = handhold_detectors.Planar(g)

    ## Generate XYZ file for pics
    with open('../data/handholds.xyz', 'w') as f:
        for n in h.get_graph().nodes:
            f.write('{} {} {}\n'.format(int(n.x), int(n.y), int(n.z)))
        
    # These are good nodes
    #start = [n for n in g.nodes if n.x == 11]
    start = [n for n in g.nodes 
            if abs(n.x + 141.446533) < 0.01 and
            abs(n.y - 1690.372192 < 0.01)]
    #end = [n for n in g.nodes if n.ID == 18498]
    end = [n for n in g.nodes 
        if abs(n.x - 200.396072) < 0.01 and
        abs(n.y + 841.415039) < 0.01]
    print('start', start, 'end', end)
    with open('../data/shortest_path.xyz', 'w') as f:
        for p in h.get_path(*start, *end):
            f.write('{} {} {}\n'.format(int(p.x), int(p.y), int(p.z)))

if __name__ == '__main__':
    main()
