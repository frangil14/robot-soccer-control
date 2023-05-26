#!/usr/bin/env python3

from math import sqrt, asin, atan2
import numpy as np

dist = lambda a,b: sqrt((a['x']-b['x'])**2+(a['y']-b['y'])**2)

def get_closer_player(all_players, our_player):
    # retorna cual es el jugador mas cercano a nuestro player

    team = our_player.get_team()

    players = [item for item in all_players if not (item.get_team() == team and item.get_role() == our_player.get_role())]
    n = len(players)
    distances = []
    for i in range(n):
        distances.append(dist(players[i].get_position(),our_player.get_position()))
    np_distances = np.array(distances)

    index = np_distances.argmin()
    closer_player = players[index]
    distance = np_distances.min()

    return [closer_player, distance]

def get_closer_partner(our_players, our_player):

    players = [item for item in our_players if item.get_role() != our_player.get_role()]

    x_coordenates = [item.get_position()['x'] for item in our_players if item.get_role() != our_player.get_role()]
    max_x = max(x_coordenates)
    min_x = min(x_coordenates)

    n = len(players)
    distances = []
    for i in range(n):
        distances.append(dist(players[i].get_position(),our_player.get_position()))
    np_distances = np.array(distances)

    index = np_distances.argmin()
    closer_player = players[index]
    distance = np_distances.min()

    if our_player.get_team() == 'blue':

        if max_x > our_player.get_position()['x']:
            # tenemos un jugador mas adelante que nosotros y lo buscamos a el

            if closer_player.get_position()['x'] > our_player.get_position()['x']:
                # si el jugador mas cercano esta mas adelante, se la damos a el
                return [closer_player, distance]
            else:
                # sino, sigo con el siguiente jugador mas cercano
                del players[index]
                return get_closer_partner(players, our_player)

        else:
            return [closer_player, distance]
    
    else:
        if min_x < our_player.get_position()['x']:
            # tenemos un jugador mas adelante que nosotros y lo buscamos a el

            if closer_player.get_position()['x'] < our_player.get_position()['x']:
                # si el jugador mas cercano esta mas adelante, se la damos a el
                return [closer_player, distance]
            else:
                # sino, sigo con el siguiente jugador mas cercano
                del players[index]
                return get_closer_partner(players, our_player)

        else:
            return [closer_player, distance]


def ball_player_min_distance(our_players, ball_position):

    n = len(our_players)
    distances = []
    for i in range(n):
        distances.append(dist(our_players[i],ball_position))
    np_distances = np.array(distances)

    return [np_distances.argmin(), np_distances.min()]


def they_have_the_ball(all_players, ball_position, gap_player_ball, our_team):

    n = len(all_players)
    distances = []
    for i in range(n):
        distances.append(dist(all_players[i].get_position(),ball_position))
    np_distances = np.array(distances)

    distance = np_distances.min()
    closer_player = all_players[np_distances.argmin()]

    if closer_player.get_team() != our_team:
        if distance <= gap_player_ball:
            return True

    return False



def get_angle_player_object(player, object_position, initial_angle):
    angle = atan2(object_position['y']-player.get_position()['y'], object_position['x']-player.get_position()['x'])
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
