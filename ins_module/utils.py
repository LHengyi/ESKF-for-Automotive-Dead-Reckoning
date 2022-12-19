import numpy as np

"""
return skew matrix of a vector
"""
def skewMatrix(vector:np.ndarray):
    return np.array([
        [0,         -vector[2], vector[1]],
        [vector[2], 0,          -vector[0]],
        [-vector[1],vector[0],  0]
    ])