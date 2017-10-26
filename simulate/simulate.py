import networkx as nx
import graph_constructor
import handhold_detectors

g = graph_constructor.generate_graph('../meshes/mesh1.ply')
h = handhold_detectors.Maxima(g).get_graph()
#print(h.edges)

# Generate XYZ file for pics
goal_node = None
start_node = None
for node in h.nodes:
    if not goal_node:
        goal_node = node
        start_node = node
    if node.x > goal_node.x and node.y > goal_node.y:
        goal_node = node
    if node.x < start_node.x and node.y < start_node.y:
        start_node = node

print('start and goal', start_node, goal_node)
path = nx.shortest_path(h, start_node, goal_node)
f = open('shortest_path.xyz', 'w')
for p in path:
    f.write('{} {} {}\n'.format(int(p.x), int(p.y), int(p.z)))

