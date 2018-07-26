import numpy as np

earthRadius = 6.371e6

def coord_to_cart(lat, lon, origin_lat, origin_lon):
    '''Converts geographic coordinates to cartesian
    Let the origin be the southwestern point on the map and let
    positive x be west, positive y be north. Units are meters'''
    yval = (lat - origin_lat) * (np.pi / 180) * np.cos((lon - origin_lon) / 2 * np.pi / 180) * earthRadius
    xval = (lon - origin_lon) * np.pi / 180 * earthRadius
    return xval, yval

def get_origin(data):
    '''Returns an origin for the dataset, which is the southernmost and easternmost
    point on the map. data is a 2D array where each index is a seperate
    data point in the form [latitude, longitude, ch4_ppm]'''
    min_lat = 999
    min_lon = 999
    for point in data:
        if point[0] < min_lat:
            min_lat = point[0]
        if point[1] < min_lon:
            min_lon = point[1]
    return min_lat, min_lon

def convert_data_cart(data, origin=False):
    '''Takes a 2D array of data, and converts the lat and lon to cartesian.
    Units are meters.'''
    if not origin:
        origin = get_origin(data)
    new_data = []
    for point in data:
        x, y = coord_to_cart(point[0], point[1], origin[0], origin[1])
        new_data.append([x, y])
    return new_data

def distance(point1, point2):
    return np.sqrt(((point2[0]-point1[0]) ** 2) + ((point2[1]-point1[1]) ** 2))
