import numpy as np
from numpy import linalg as LA
def egiens(matrix_):
    '''docstring for eigens''' 
    matrix = np.zeros((len(matrix_),len(matrix_)), int)
    for i, line in enumerate(matrix_):
        for j, item in enumerate(line):
           matrix[i][j] = item; 
    w, v = LA.eig(matrix)
    print "egiens:" 
    print w
    return w

