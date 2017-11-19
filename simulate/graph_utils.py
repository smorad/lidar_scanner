import math

def euclidean_distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 + 
        (a.z - b.z) ** 2
    )
