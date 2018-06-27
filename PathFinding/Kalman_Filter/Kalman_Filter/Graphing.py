from Coordinates import *
from Map import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm

def graph_state_est(map):
    '''Graphs the filtered state estimates with matplotlib'''
    xx = []; yy = []; zz = []
    for j in range(map.numRows): #Create 2D arrays for x, y, z axis
        xRow = []; yRow = []; zRow = []
        for i in range(map.numCols):
            x, y = map.get_xy_from_ij(i, j)
            xRow.append(x)
            yRow.append(y)
            zRow.append(map.get_node(i, j).get_state_est())
        xx.append(xRow); yy.append(yRow); zz.append(zRow)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(xx, yy, zz, cmap=cm.coolwarm)
    #ax.set_zlim(1, 5)
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.set_zlabel('CH4 Concentration (ppm)')
    fig.colorbar(surf)
    plt.show()

def graph_variance(map):
    '''Graphs variance of map with matplotlib'''
    xx = []; yy = []; zz = []
    for j in range(map.numRows): #Create 2D arrays for x, y, z axis
        xRow = []; yRow = []; zRow = []
        for i in range(map.numCols):
            x, y = map.get_xy_from_ij(i, j)
            xRow.append(x)
            yRow.append(y)
            zRow.append(map.get_node(i, j).get_variance_est())
        xx.append(xRow); yy.append(yRow); zz.append(zRow)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(xx, yy, zz, cmap=cm.coolwarm)
    #ax.set_zlim(1, 5)
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.set_zlabel('Variance (ppm^2)')
    fig.colorbar(surf)
    plt.show()

def graph_variance_plot(x, y):
    '''Plot variance vs distance'''
    plt.plot(x, y)
    plt.xlabel('Distance between points (m)')
    plt.ylabel('Variance (#/cm^3)')
    plt.show()
