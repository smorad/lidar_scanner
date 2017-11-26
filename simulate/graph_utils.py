import math

# Just some utils that I want to use without create
# circular deps

RISK_WEIGHT = 0.25

def euclidean_distance(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 + 
        (a.z - b.z) ** 2
    )

def compute_cost(a, b):
    '''Compute cost (distance and risk)'''
    return euclidean_distance(a, b) + RISK_WEIGHT * b.loss

