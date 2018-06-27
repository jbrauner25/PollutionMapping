class Map:
    '''Creates a 2D array of size (cols, rows) containing Nodes'''
    def __init__(self, cols, rows, size):
        self.cellSize = size
        self.numCols = cols
        self.numRows = rows
        self.map = [[0 for i in range(cols)] for j in range(rows)]
        for i in range(rows):
            for j in range(cols):
                self.map[i][j] = Node()

    def get_ij_from_xy(self, x, y):
        '''Returns the array indices for point (x, y)'''
        if type(self.cellSize) == 'int' or type(self.cellSize) == 'float':
            if (x < 0 or y < 0):
                raise IndexError("x and y must be in range")
            if (x > self.cellSize * self.numCols or y > self.cellSize * self.numRows):
                raise IndexError("x and y must be in range")
            return x // self.cellSize, y // self.cellSize
        else:
            if (x < 0 or y < 0):
                raise IndexError("x and y must be in range")
            if (x > self.cellSize[0] * self.numCols or y > self.cellSize[1] * self.numRows):
                raise IndexError("x and y must be in range")
            return x // self.cellSize[0], y // self.cellSize[1]

    def get_xy_from_ij(self, i, j):
        '''Returns the center point of (i, j)'''
        if (i < 0 or j < 0):
            raise IndexError("i and j must be in range")
        if (i >= self.numCols or j >= self.numRows):
            raise IndexError("i and j must be in range")
        if type(self.cellSize) == 'int' or type(self.cellSize) == 'float':
            return i * self.cellSize + self.cellSize / 2, j * self.cellSize + self.cellSize / 2
        else:
            return i * self.cellSize[0] + self.cellSize[0] / 2, j * self.cellSize[1] + self.cellSize[1] / 2

    def get_node(self, i, j):
        return self.map[j][i]

class Node:
    '''Creates a node which stores current state estimation and variance for a point'''
    def __init__(self, state=1, variance=999e20):
        self.state_est = state
        self.variance_est = variance

    def set_state_est(self, value):
        self.state_est = value

    def set_variance_est(self, value):
        self.variance_est = value

    def get_state_est(self):
        return self.state_est

    def get_variance_est(self):
        return self.variance_est
