from Coordinates import *
from Graphing import *
from Map import *
import numpy as np
from scipy import stats
import math

def get_variance(m, distance):
    '''returns the measument variance for a data point wrt a node'''
    return m * distance ** 2 #n calculated from calc_variance_slope

def calc_variance_slope(data, numBins, binSize):
    '''Calculate the slope of the line between measurement
    variance and distance between measurements.'''
    def get_bin(d):
        return int(d // binSize)
    bins = [[] for a in range(numBins)]
    for i in range(len(data)):
        for j in range(i + 1, len(data)):
            dist = distance(data[i][:2], data[j][:2])
            if (dist < numBins * binSize):
                b = get_bin(dist)
                bins[b].append(abs(data[i][2] - data[j][2]))
    distances = []
    bin_variance = []
    for i in range(len(bins)):
        bin_variance.append(np.var(bins[i]))
        distances.append(i * binSize + binSize / 2)
    #Uncomment line below to plot variance vs distance
    graph_variance_plot(distances, bin_variance)
    xs = np.array(distances)
    ys = np.array(bin_variance)
    #Calculate slope of line of best fit for points
    m = (((np.mean(xs) * np.mean(ys)) - np.mean(xs * ys)) /
         ((np.mean(xs) * np.mean(xs)) - np.mean(xs * xs)))
    #print(m)
    return m

def calc_variance_slope_UCR(data, numBins, binSize):
    '''Calculate the slope of the line between measurement
    variance and distance between measurements.'''
    peak = [25000, 44000]
    def get_bin(d):
        return int(d // binSize)
    bins = [[] for a in range(numBins)]
    for i in range(len(data)):
        if distance(data[i][:2], peak) <= 1000:
            for j in range(i + 1, len(data)):
                if distance(data[j][:2], peak) <= 1000:
                    dist = distance(data[i][:2], data[j][:2])
                    if (dist < numBins * binSize):
                        b = get_bin(dist)
                        bins[b].append(abs(data[i][2] - data[j][2]))
    distances = []
    bin_variance = []
    for i in range(len(bins)):
        bin_variance.append(np.var(bins[i]))
        distances.append(i * binSize + binSize / 2)
    #Uncomment line below to plot variance vs distance
    #graph_variance_plot(distances, bin_variance)
    xs = np.array(distances)
    ys = np.array(bin_variance)
    #Calculate slope of line of best fit for points
    m = (((np.mean(xs) * np.mean(ys)) - np.mean(xs * ys)) /
         ((np.mean(xs) * np.mean(xs)) - np.mean(xs * xs)))
    return m
