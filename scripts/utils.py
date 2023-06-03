#!/usr/bin/env python3

from math import sqrt, atan2, atan, sin, cos
import numpy as np
import random
from config import YELLOW_GOAL_Y_MIN, YELLOW_GOAL_Y_MAX

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
                return closer_player
            else:
                # sino, sigo con el siguiente jugador mas cercano
                del players[index]
                return get_closer_partner(players, our_player)

        else:
            return closer_player

    # yellow team
    
    else:
        if min_x < our_player.get_position()['x']:
            # tenemos un jugador mas adelante que nosotros y lo buscamos a el

            if closer_player.get_position()['x'] < our_player.get_position()['x']:
                # si el jugador mas cercano esta mas adelante, se la damos a el
                return closer_player
            else:
                # sino, sigo con el siguiente jugador mas cercano
                del players[index]
                return get_closer_partner(players, our_player)

        else:
            return closer_player


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

def get_rectangle_vertices(x1, y1, x2, y2, gap):
    # pendiente de la recta

    if x2 == x1:
        dx = (gap/2)
        dy = 0
    else:
        m = (y2-y1) / (x2-x1)
        theta = atan(m)
        dx = (gap/2) * sin(theta)
        dy = (gap/2) * cos(theta)

    point_1 = (x1 + dx, y1 - dy)
    point_2 = (x1 - dx, y1 + dy)
    point_3 = (x2 - dx, y2 + dy)
    point_4 = (x2 + dx, y2 - dy)

    return [point_1, point_2, point_3, point_4]

def point_in_rectangle(rectangle, point):
    x1, y1 = rectangle[0]
    x2, y2 = rectangle[1]
    x3, y3 = rectangle[2]
    x4, y4 = rectangle[3]

    line1 = (y2 - y1) * point[0] - (x2 - x1) * point[1] - x1 * y2 + x2 * y1
    line2 = (y3 - y2) * point[0] - (x3 - x2) * point[1] - x2 * y3 + x3 * y2
    line3 = (y4 - y3) * point[0] - (x4 - x3) * point[1] - x3 * y4 + x4 * y3
    line4 = (y1 - y4) * point[0] - (x1 - x4) * point[1] - x4 * y1 + x1 * y4

    if line1 >= 0 and line2 >= 0 and line3 >= 0 and line4 >= 0:
        return True
    elif line1 <= 0 and line2 <= 0 and line3 <= 0 and line4 <= 0:
        return True
    else:
        return False
            

def is_someone_in_between(active_player, target, all_players, for_pass = False, gap = 200):

    if for_pass:
        x2 = target.get_position()['x']
        y2 = target.get_position()['y']
    else:
        x2 = target['x']
        y2 = target['y']

    x1 = active_player.get_position()['x']
    y1 = active_player.get_position()['y']


    vertices = get_rectangle_vertices(x1, y1, x2, y2, gap)

    team = active_player.get_team()

    # no comparo con el mismo, ni con el companero
    if for_pass:
        players = [item for item in all_players if not (item.get_team() == team and (item.get_role() == active_player.get_role() or item.get_role() == target.get_role()))]
    else:
        players = [item for item in all_players if not (item.get_team() == team and item.get_role() == active_player.get_role())]

    for player in players:

        x = player.get_position()['x']
        y = player.get_position()['y']
        if point_in_rectangle(vertices, (x, y)):
            return True

    return False

def get_new_point_in_rival_goal(current_point):
    x = current_point['x']
    y = current_point['y']
    # x coordenate never changes
    if y > 0:
        y = random.randint(YELLOW_GOAL_Y_MIN, 0)
    else:
        y = random.randint(0, YELLOW_GOAL_Y_MAX)

    return {'x':x, 'y':y}