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

def ball_player_min_distance(our_players, ball_position):

    n = len(our_players)
    distances = []
    for i in range(n):
        distances.append(dist(our_players[i],ball_position))
    np_distances = np.array(distances)

    return [np_distances.argmin(), np_distances.min()]

def they_have_the_ball(all_players, ball_position, gap_player_ball):

    n = len(all_players)
    distances = []
    for i in range(n):
        distances.append(dist(all_players[i].get_position(),ball_position))
    np_distances = np.array(distances)

    distance = np_distances.min()

    closer_player = all_players[np_distances.argmin()]

    if closer_player.get_team() != 'blue':
        if distance <= gap_player_ball:
            return True

    return False



def get_angle_player_object(player_position, object_position, initial_angle):

    angle = atan2(object_position['y']-player_position['y'], object_position['x']-player_position['x'])

    return angle - initial_angle

def get_distance_player_object(player_position, object_position):
    return dist(player_position, object_position)

def get_active_player(players, ball_position):

    our_players_positions = [item.get_position() for item in players]
    index_player, distance_player_ball = ball_player_min_distance(our_players_positions, ball_position)

    closer_player = players[index_player]

    if closer_player.ball_is_in_area(ball_position):
        # si la pelota esta en Area del player mas cercano, se activa
        return [closer_player, distance_player_ball]
    else:
        # sino, sigo con el siguiente jugador mas cercano
        del players[index_player]
        return get_active_player(players, ball_position)
