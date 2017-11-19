import math

def euclidean_distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 + 
        (a.z - b.z) ** 2
    )

def compute_cost(a, b, risk_weight=1):
    '''Compute cost (distance and risk)'''
    return euclidean_distance(a, b) + risk_weight * b.loss

