# General utility functions for Snow Plowing
# Copyright, Zenith Robotics Inc.
# Python imports
import random
#from images2gif import writeGif
import glob
#from PIL import Image as PIL_Image
#from images2gif import writeGif
import math
import os


RANDOM_DISTINCT_COLORS = [
    '#3cb44b',
    '#0082c8',
    '#f58231',
    '#911eb4',
    '#46f0f0',
    '#f032e6',
    '#ffe119',
    '#d2f53c',
    '#fabebe',
    '#008080',
    '#e6beff',
    '#aa6e28',
    '#800000',
    '#aaffc3',
    '#808000',
    '#000080']

# General Utilities


def random_distinct_colors(n):
    """Return one of the 17 random distinct colors."""
    return RANDOM_DISTINCT_COLORS[n % len(RANDOM_DISTINCT_COLORS)]


def random_color():
    """Return hex code in str of a random color."""
    return "#%06x" % random.randint(0, 0xFFFFFF)


def distance(p1, p2):
    try:
        return math.sqrt(((p1[0] - p2[0])**2) + ((p1[1] - p2[1])**2))
    except BaseException:
        return False


def saveImagesToGif(gif_filename):
    """Save a series of image to gif."""
    print("Saving GIF")
    images = [
        PIL_Image.open(image) for image in glob.glob(
            'images/' +
            gif_filename +
            '*.png')]
    file_path_name = 'images/' + gif_filename + '.gif'
    writeGif(file_path_name, images, duration=0.15)
