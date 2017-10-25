import networkx as nx
import graph_constructor
import handhold_detectors

g = graph_constructor.generate_graph('../meshes/mesh1.ply')
h = handhold_detectors.Maxima(g).get_graph()
print(len(h.edges))
