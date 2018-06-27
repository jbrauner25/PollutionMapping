from Coordinates import *
from Graphing import *
from Variance import *
from Map import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from scipy import stats
import scipy.io
import time

def Kalman_Filter(map, data, filter_radius):
    '''Performs the Kalman Filter algorithm for every data point in "data"
    Only nodes on the map that are within the filter_radius of the data point
    are put through the filter.'''
    m = calc_variance_slope(data, 5, 20)
    for point in data:
        for i in range(map.numCols):
            for j in range(map.numRows):
                cart_point = map.get_xy_from_ij(i, j)
                dist = distance([point[0], point[1]], cart_point)
                if (filter_radius == None or dist <= filter_radius): #Only nodes close to the data point are filtered
                    node = map.get_node(i, j)
                    meas_var = get_variance(m, dist)
                    K_t = node.get_variance_est() / (node.get_variance_est() + meas_var) #calculate Kalman gain
                    node.set_state_est(node.get_state_est() + K_t * (point[2] - node.get_state_est()))
                    node.set_variance_est(node.get_variance_est() - K_t * node.get_variance_est())
    #matlab_matrices(map)
    graph_state_est(map) #graph state estimates of the filtered map in matplotlib
    #graph_variance(map) #graph variance of the filtered map in matplotlib
    '''
    xs = []
    ys = []
    va = []
    for j in range(map.numCols):
        node = map.get_node(j, 0)
        xs.append(map.get_xy_from_ij(j, 0)[0])
        ys.append(node.get_state_est())
        va.append(node.get_variance_est())
    plt.plot(xs, ys)
    plt.xlabel('Position (m)')
    plt.ylabel('Concentration (#/cm^3)')
    plt.title('predicted aveconc vs position for 1D Indian Hill test (t=2)')
    plt.show()
    plt.plot(xs, va)
    plt.xlabel('Position (m)')
    plt.ylabel('Variance ((#/cm^3)^2)')
    plt.title('predicted variance vs position for 1D Indian Hill test (t=2)')
    plt.show()
    '''
    return map

def Kalman_Filter_UCR(map, data, filter_radius):
    '''Performs the Kalman Filter algorithm for every data point in "data"
    Only nodes on the map that are within the filter_radius of the data point
    are put through the filter.'''
    m = calc_variance_slope_UCR(data, 5, 100)
    #var_tracker = []
    for point in data:
        for i in range(map.numCols):
            for j in range(map.numRows):
                cart_point = map.get_xy_from_ij(i, j)
                dist = distance([point[0], point[1]], cart_point)
                if (filter_radius == None or dist <= filter_radius): #Only nodes close to the data point are filtered
                    node = map.get_node(i, j)
                    meas_var = get_variance(m, dist)
                    K_t = node.get_variance_est() / (node.get_variance_est() + meas_var) #calculate Kalman gain
                    node.set_state_est(node.get_state_est() + K_t * (point[2] - node.get_state_est()))
                    node.set_variance_est(node.get_variance_est() - K_t * node.get_variance_est())
                    #if i == 60 and j == 60:
                    #    var_tracker.append(node.get_variance_est())
    #matlab_matrices(map)
    #map.get_node(60, 60).set_state_est(3)
    #plt.plot([i for i in range(len(var_tracker))], var_tracker)
    #plt.xlabel("# of measurements added")
    #plt.ylabel("Variance (ppm^2)")
    #plt.show()
    #graph_state_est(map) #graph state estimates of the filtered map in matplotlib
    #graph_variance(map) #graph variance of the filtered map in matplotlib
    return map

def matlab_matrices(map):
    '''Formats the data from map to save as .mat'''
    xx = []; yy = []; conc = []; var = []
    for j in range(map.numRows): #Create 2D arrays for x, y, z axis
        xRow = []; yRow = []; concRow = []; varRow = []
        for i in range(map.numCols):
            x, y = map.get_xy_from_ij(i, j)
            xRow.append(x)
            yRow.append(y)
            concRow.append(map.get_node(i, j).get_state_est())
            varRow.append(map.get_node(i, j).get_variance_est())
        xx.append(xRow); yy.append(yRow); conc.append(concRow); var.append(varRow)
    scipy.io.savemat('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/Kalman_Filter/MATLAB_data/130622-b/data7.mat',
        mdict={'x': xx, 'y': yy, 'conc': conc, 'var': var})
