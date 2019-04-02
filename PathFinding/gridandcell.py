import numpy as np, networkx as nx, matplotlib.pyplot as plt
import math
import cellKalman
import random
import scipy





class Cell(object):

    def __init__(self, col, row, x, y, width, polEst=None, polEstVar=None):
        # Lat= x, lon = y, for cartesian.
        self.col = col
        self.row = row
        self.x = x
        self.y = y
        self.width = width
        self.polEst = polEst
        self.polEstVar = polEstVar
        self.cost = 0

    def __eq__(self, other):
        if self.col == other.col and self.row == other.row and self.x == other.x and self.y == other.y and self.width == other.width:
            return True
        return False

    def __repr__(self):
        return 'Cell(row/y = %s, col/x = %s, x = %s, y = %s)' % (self.row, self.col, self.x, self.y)

    def get_cell_ID(self):
        '''Returns the column number and row number the column resides in
		from its parent grid'''
        return self.col, self.row




    def update_cell_state(self, measVal, xPos, yPos):
        '''Updates the parameters for the Kalman Filter'''
        distSquared = (xPos - self.x) ** 2 + (yPos - self.y) ** 2
        dist = np.sqrt(distSquared)
        measVar = cellKalman.meas_var_dist(dist)
        if self.polEst is None:
            self.polEst = measVal
            self.polEstVar = measVar
        else:
            posteriEst, posteriEstVar = cellKalman.kalman_filter(self.polEst, self.polEstVar, measVal, measVar)
            self.polEst = posteriEst
            self.polEstVar = posteriEstVar
        #print("x location: " + str(xPos) + "y loc " + str(yPos) + "measured value: " + str(measVal) + "Pol estimate: " + str(self.polEst) + "Var estimate: " + str(self.polEstVar) + "My cell's x: " + str(self.x) + " my y: " + str(self.y))
        return self.polEst, self.polEstVar

    def cell_objective_function(self, alpha):
        '''Cost function of cell for optimizing the path taken in planning a route'''
        cost = alpha * self.polEstVar + (1 - alpha) * self.polEst
        self.cost = cost


class Grid2DCartesian(object):

    def __init__(self, width, height, meter_box, truth_function=False):
        '''Initializes a 2D Grid given the length (x) and width (y) of the grid. The data parameter should be given as a 2D list.
		The length of the main list should be size X and the lenght of each list within the list should be size Y. The grid
		origin is the bottom left most point of the grid.'''
        self.graph = nx.Graph()

        r_earth = 6378137
        coords = []
        col = 0
        row = 0
        row_hold = []
        self.grid = []
        self.width = width
        self.height = height
        x = meter_box/2
        y = meter_box/2  # Start in midpoint of first cell, bottom left
        # Start in bottom left corner, work way up
        # Imagine [0][0] bottom left, increases up and right.
        self.coords = []
        self.list = linkedList()
        self.arraylist = []

        # Builds 2D cell array and networkX graph

        while x <= width:
            while y <= height:
                coords.append((x, y))
                new_cell = Cell(col, row, x, y, meter_box)
                self.coords.append((x,y))
                self.list.add_to_front((x, y, col, row))
                self.graph.add_node((x, y), x_cart=x, y_cart=y, cell=new_cell, col=col, row=row)
                row_hold.append(new_cell)
                y += meter_box
                row += 1
            self.grid.append(row_hold)
            x += meter_box
            y = meter_box/2  # Reset for new iteration
            row = 0  # Reset for new iteration
            row_hold = []
            col += 1
        #for node in self.graph.nodes():
        #self.coordinates = coords
        self.numCol = len(self.grid)
        self.numRow = len(self.grid[0])
        print("(row, col) (" + str(self.numRow) + ", " + str(self.numCol) + ")")

        for col in range(self.numCol - 1):  # If 5 col, iterate 0-4
            for row in range(self.numRow - 2):  # Not trying to connect the last row to anything.
                curPoint = self.get_cell_center(self.get_cell(col, row))
                northPoint = self.get_cell_center(self.get_cell(col, row + 1))
                self.graph.add_edge(curPoint, northPoint)

        for row in range(self.numRow-1):
            for col in range(self.numCol - 2):
                curPoint = self.get_cell_center(self.get_cell(col, row))
                eastPoint = self.get_cell_center(self.get_cell(col + 1, row))  # This just finds the name of the cell
                self.graph.add_edge(curPoint, eastPoint)  # This connects the nodes by looking up names.
        self.linkedList = CompiledLinkedList(self.grid, meter_box)
        self.gridsize = len(coords)

        if truth_function:
            for x in range(len(self.grid)):
                for y in range(len(self.grid[0])):
                    new_x = self.grid[x][y].x
                    new_y = self.grid[x][y].y
                    pollution = self.pollutionfunction(new_x, new_y)
                    self.grid[x][y].polEst = pollution
                    self.grid[x][y].polEstVar = 0


        #self.plot_coords()

        # Build Linked list

    @staticmethod
    def pollutionfunction(x, y):
        return x*y
        #return math.sin(y/100) + math.cos(x/100)
        #if x < 300 and y < 550 and x > 100 and y > 400:
        #     return 5000
        #return 5

    def get_cell(self, col, row):
        return self.grid[col][row]

    # def kalman(self, pol, point):
    #
    #

    def get_cell_center(self, cell):
        '''Gets a cell given a specific row and column'''
        return (cell.x, cell.y)

    def get_cell_from_index(self, x_or_col, y_or_row):
        '''Input: cell index
            Output: mutable cell class
         '''
        return self.grid[x_or_col][y_or_row]

    def whichCellAmIIn(self, x, y):
        '''Input: cartesian coords
                Output: immutable cell class
                '''
        if x > self.width or y > self.height:
            raise Exception('Out of range')
        return self.linkedList.whichcellamIin(x, y)

    def whichCellAmIIn_loc(self, x, y):
        '''Input: cartesian coords
        Output: lat,long of cell it is in
        '''
        cell = self.linkedList.whichcellamIin(x, y)
        return cell.x, cell.y

    def whichCellAmIIn_index(self, x, y):
        '''Input: cartesian coords
        Output: col, row of cell it is in
        '''
        cell = self.linkedList.whichcellamIin(x, y)
        return cell.col, cell.row

    def random_kalman(self, pol_count, pol_min, pol_max):
        # random.seed(100)
        pointpollutionList = []
        for z in range(pol_count):
            point = (random.uniform(0, self.width), random.uniform(0, self.height))
            # self.points.append(point)
            # self.updated_nodes.append(ox.get_nearest_node(self.env.G, point))
            # self.kalman.update(random.randint(pol_min, pol_max), point, 100)  # r
            pollution = random.randint(pol_min, pol_max)
            pointpollutionList.append((point, pollution))
            for x in range(len(self.grid)):
                for y in range(len(self.grid[0])):
                    pol, var = self.grid[x][y].update_cell_state(pollution, point[0], point[1])
        return pointpollutionList


    def add_pollution(self, pol, cart_loc):
        '''Loops through all cels to update pollution values'''
        for x in range(len(self.grid)):
            for y in range(len(self.grid[0])):
                self.grid[x][y].update_cell_state(pol, cart_loc[0], cart_loc[1])

    def compare(self, other):
        residuals = []
        max = 0
        min = 999999999
        for x in range(len(self.grid)):
            for y in range(len(self.grid[0])):
                selfPollution = self.grid[x][y].polEst
                otherPollution = other.grid[x][y].polEst
                value = np.abs(selfPollution - otherPollution)
                if value > max:
                    max = value
                if value < min:
                    min = value
                residuals.append(value)
        return (residuals, max, min)

    def objective(self, lambda_l, locations):
        objective = 0
        for x in range(len(self.grid)):
            for y in range(len(self.grid[0])):
                if (x, y) not in locations:
                    objective += lambda_l * self.grid[x][y].polEst + (1 - lambda_l) * self.grid[x][y].polEstVar
        return objective

    def save_grid(self, fileName):
        new_grid = []
        for x in range(len(self.grid)):
            vector = []
            for y in range(len(self.grid[0])):
                pollution = self.grid[x][y].polEst
                var = self.grid[x][y].polEstVar
                locationx = self.grid[x][y].x
                locationy = self.grid[x][y].y
                vector.append((locationx, locationy, pollution, var))
            new_grid.append(vector)
        numpyarray = np.array(new_grid)

        scipy.io.savemat(fileName, mdict={'dataarray': numpyarray})






class linkedList(object):

    def __init__(self, element=None):
        if element:
            self.myHead = self.listnode(element)  #Listnode
            self.mySize = 1
        else:
            self.myHead = None
            self.mySize = 0 #int

    def isEmpty(self):
        return self.length() == 0

    def length(self):
        return self.mySize

    def getIndex(self, index):
        if index < 0:
            return None
        elif index >= self.mySize:
            return None
        currentIndex = 0
        currentNode = self.myHead
        while currentIndex != index:
            currentIndex += 1
            currentNode = currentNode.myNext
        return currentNode.myData

    def findLocation(self, data):
        if self.mySize == 0:
            return None
        currentNode = self.myHead
        while currentNode.myNext:
            if currentNode.myNext.myData.col >= data:
                return currentNode
            currentNode = currentNode.myNext
        return currentNode

# add(data) adds data to end of list
    def add(self, data):
        if self.mySize == 0:
            self.myHead = ListNode(data)
        elif self.myHead.myNext == None:
            self.myHead.myNext = ListNode(data)
        else:
            currentnode = self.myHead
            while not currentnode.myNext == None:
                currentnode = currentnode.myNext
            currentnode.myNext = ListNode(data)
        self.mySize += 1

    def add_to_front(self, data):
        if self.mySize == 0:
            self.myHead = ListNode(data)
        temp = self.myHead
        self.myHead = ListNode(data)
        self.myHead.myNext = temp
        self.mySize += 1


class CompiledLinkedList(object):

    def __init__(self, inputarray, cellwidth):
        # 2D array access follows [col][row]
        #It will hold object cell. We won't care about var/pol. just col, row, x, and y
        # We want to build each column first, then connect the rows. The following is a ~fun~ diagram
         #   []    []    []
        #    ^     ^     ^
        #   []    []    []
       #    ^     ^     ^
        #   [] -> [] -> []
        self.cellwidth = cellwidth
        array_of_linkedLists = []
        seconddimentionlinkedlist = linkedList()
        for x in range(len(inputarray)):
            list = linkedList()
            for y in range(len(inputarray[0])):
                list.add(inputarray[x][y])
            array_of_linkedLists.append(list)
        for x in array_of_linkedLists:
            seconddimentionlinkedlist.add(x)
        self.data = seconddimentionlinkedlist

    def whichcellamIin(self, x, y):
        #x -> y ^   remember bottom left is the origin
        if self.data.mySize == 0:
            return None
        currentNode = self.data.myHead
        x_location = None
        while currentNode.myNext:
            if currentNode.myNext.myData.myHead.myData.x - (self.cellwidth/2) >= x:
                break
            currentNode = currentNode.myNext
        LinkedListColumn = currentNode.myData
        y_location = None
        currentNode = LinkedListColumn.myHead
        while currentNode.myNext:
            if currentNode.myNext.myData.y - (self.cellwidth/2) >= y:
                break
            currentNode = currentNode.myNext

        return currentNode.myData


class ListNode(object):
    def __init__(self, element=None, next=None):
            self.myData = element
            self.myNext = next








