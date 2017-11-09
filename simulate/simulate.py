import networkx as nx
import graph_constructor
import handhold_detectors

g = graph_constructor.generate_graph('../data/mesh1.ply')
#h = handhold_detectors.Maxima(g)
h = handhold_detectors.Planar(g)

# Generate XYZ file for pics
with open('../data/handholds.xyz', 'w') as f:
    for n in h.get_graph().nodes:
        f.write('{} {} {}\n'.format(int(n.x), int(n.y), int(n.z)))
    
with open('../data/shortest_path.xyz', 'w') as f:
    for p in h.get_path():
        f.write('{} {} {}\n'.format(int(p.x), int(p.y), int(p.z)))

