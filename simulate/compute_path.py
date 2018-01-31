from heapq import heappush, heappop
from itertools import count
from networkx import NetworkXError
import networkx as nx

from graph_utils import euclidean_distance, euclidean_distance_c


def compute_hub_pos(bots):
    #points = [(b0.node, b1.node) for (b0, b1) in itertools.combinations(bots, 2)]
    #midpoints = [((n0.x + n1.x)/2, (n0.y + n1.y)/2, (n0.z + n1.z)/2)for n0, n1 in points]
    #hub_pos = 
    points = [(b.node.x, b.node.y, b.node.z) for b in bots]
    # transpose list
    x, y, z = map(list, zip(*points))

    return (sum(x) / len(x), sum(y) / len(y), sum(z) / len(z)) 


def bounded_leg_astar(G,
                      source,
                      target,
                      heuristic=None,
                      weight='weight',
                      bots=[]):
    # stolen and modified from networkx library
    '''
    Given a graph, source, and sink compute a path from the source to sink. 
    The leg distance (not edge weight) is bounded as our hopping/climbing robot does not
    have unlimited hop/reach distance. This function will also skip nodes occupied
    by other robots, and will respect the tether length of each robot.
    '''
    if G.is_multigraph():
        raise NetworkXError("astar_path() not implemented for Multi(Di)Graphs")

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    if not bots:
        print('Warning: no bots passed to a*, this should only happen if you are using one bot')

    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    first_run = True
    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent = pop(queue)

        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            if neighbor in explored:
                continue
            # we want bound to be distance only (no risk/weight)
            # ensure only one edge from a to b
            if w['dist'] > source.occupied.hop_distance:
                continue
            # node is already occupied by another bot
            if first_run and neighbor.occupied:
                continue

            # Ensure we don't leave tether range
            if first_run and euclidean_distance_c(
                neighbor, compute_hub_pos(bots)) > source.occupied.tether_distance:
                continue

            # cost should also include risk
            ncost = dist + w.get(weight, 1)
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost < ncost, a longer path to neighbor remains
                # enqueued. Removing it would need to filter the whole
                # queue, it's better just to leave it there and ignore
                # it when we visit the node a second time.
                if qcost <= ncost:
                    continue

            else:
                h = heuristic(neighbor, target)
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))
            # We've run through the first hop, the constraints such as
            # tether distance are dynamic, so let's not compute
            # them for future hops
            first_run = False

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))
