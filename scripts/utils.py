#!/usr/bin/env python3

from math import sqrt, asin, atan2
import numpy as np

dist = lambda a,b: sqrt((a['x']-b['x'])**2+(a['y']-b['y'])**2)

def calculate_distance_matrix(data):
    n = len(data)
    dist_matrix = np.zeros((n,n))
    for i in range(n):
        for j in range(i, n):
            dist_matrix[i,j] = dist(data[i],data[j])
            dist_matrix[j,i] = dist_matrix[i,j]

    return dist_matrix

def ball_player_min_distance(data, ball_position):
    n = len(data)
    distances = []
    for i in range(n):
        distances.append(dist(data[i],ball_position))
    np_distances = np.array(distances)

    return [np_distances.argmin(), np_distances.min()]


def get_angle_player_object(player_position, object_position, initial_angle):

    angle = atan2(object_position['y']-player_position['y'], object_position['x']-player_position['x'])

    return angle - initial_angle

def get_player_rivalGoal_distance(player_position, rivalGoal_position):
    return dist(player_position, rivalGoal_position)

