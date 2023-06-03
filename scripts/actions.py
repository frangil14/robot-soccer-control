#!/usr/bin/env python3

from utils import get_angle_player_object, get_closer_player, is_someone_in_between, get_new_point_in_rival_goal, get_distance_player_object
from config import *


def locate_target(rospy, player_to_action, all_players, target, ball_position=None, target_description = 'goal'):

    output = None

    gap = 0.03
    if target == 'partner':
        gap = 0.15

    angle_player_target = get_angle_player_object(player_to_action, target, player_to_action.get_angle())
    distance_player_target = get_distance_player_object(player_to_action.get_position(), target)

    if angle_player_target < gap and angle_player_target > -gap:
        # estoy mirando al arco
        player_to_action.stop_rotate()

        if target_description == 'goal':
            output = go_rival_goal(rospy, player_to_action, all_players, ball_position, target)
        elif target_description == 'partner':
            pass_to_partner(player_to_action, distance_player_target)
        elif target_description == 'ball':
            go_search_ball(rospy,player_to_action, all_players, distance_player_target)
        elif target_description == 'point':
            go_defined_point(player_to_action, all_players, distance_player_target)
            


    elif (angle_player_target>0 and angle_player_target < 4) or angle_player_target<-4:
        player_to_action.rotate_right(0.30)
    else:
        player_to_action.rotate_left(0.30)

    return output


def avoid_crushing_player(player_to_action, all_players, velocity = 0.2):
    closer_player, distance_closer_player = get_closer_player(all_players, player_to_action)
    angle_player_closerPlayer = get_angle_player_object(player_to_action, closer_player.get_position(), player_to_action.get_angle())

    if distance_closer_player < safe_distance_rival_player and (angle_player_closerPlayer < 0.3 and angle_player_closerPlayer > -0.3):
        # estoy por chocar a otro jugador

        player_to_action.stop_go()
        player_to_action.go_sideways(velocity_sideways) # trato de esquivarlo

    else:
        player_to_action.stop_go_sideways()
        player_to_action.go_forward(velocity)

def go_rival_goal(rospy, player_to_action, all_players, ball_position, target):

    avoid_crushing_player(player_to_action, all_players)

    if player_to_action.get_team() == 'yellow':

        if (player_to_action.get_position()['x'] >= BLUE_SMALL_AREA_X_MIN and player_to_action.get_position()['x'] <= BLUE_SMALL_AREA_X_MAX and
        player_to_action.get_position()['y'] >= BLUE_SMALL_AREA_Y_MIN and player_to_action.get_position()['y'] <= BLUE_SMALL_AREA_Y_MAX):

            # estamos dentro del area rival

            if is_someone_in_between(player_to_action, target, all_players):
                # hay alguien en el medio, debo cambiar de target dentro del arco
                player_to_action.stop_go()
                new_rival_goal = get_new_point_in_rival_goal(target)
                rospy.loginfo('aca cambio el punto del arco')
                return new_rival_goal
            else:
                player_to_action.stop_go()
                player_to_action.stop_dribbling()
                player_to_action.kick_ball(3.0)    # pateo
                player_to_action.lost_the_ball()
    
    else:
        if (player_to_action.get_position()['x'] >= YELLOW_SMALL_AREA_X_MIN and player_to_action.get_position()['x'] <= YELLOW_SMALL_AREA_X_MAX and
        player_to_action.get_position()['y'] >= YELLOW_SMALL_AREA_Y_MIN and player_to_action.get_position()['y'] <= YELLOW_SMALL_AREA_Y_MAX):

            # estamos dentro del area rival

            if is_someone_in_between(player_to_action, target, all_players):
                # hay alguien en el medio, debo cambiar de target dentro del arco
                player_to_action.stop_go()
                new_rival_goal = get_new_point_in_rival_goal(target)
                rospy.loginfo('aca cambio el punto del arco')
                return new_rival_goal
            else:
                player_to_action.stop_go()
                player_to_action.stop_dribbling()
                player_to_action.kick_ball(3.0)    # pateo
                player_to_action.lost_the_ball()

    if not player_to_action.ball_is_in_area(ball_position, reaching_limit_area):
        # estoy al limite de mi area, le doy la pelota a un companero
        player_to_action.stop_go()
        player_to_action.pass_to_partner()

def pass_to_partner(player_to_action, distance_partner_player):
    # estoy mirando a mi companero 
    player_to_action.stop_rotate()
    player_to_action.stop_dribbling()
    player_to_action.kick_ball(3.0 * (distance_partner_player/1800))    # doy el pase
    player_to_action.go_to_goal()
    player_to_action.lost_the_ball()

def go_search_ball(rospy, player_to_action, all_players, distance_player_ball):
    if distance_player_ball < 110:
        rospy.loginfo('estoy al lado de la pelota')
        # si estoy al lado de la pelota
        player_to_action.stop_go()
        rospy.loginfo('AGARRE LA PELOTA')
        player_to_action.got_the_ball()
        player_to_action.start_dribbling()
    else:
        # si estoy lejos de la pelota, voy hacia adelante porque ya estoy posicionado
        # aca tenemos que chequear que no nos choquemos con otro jugador
        avoid_crushing_player(player_to_action, all_players)

def go_defined_point(player_to_action, all_players, distance_player_point):
    if distance_player_point < 105:
        # estoy en posicion
        player_to_action.stop_go()
    else:
        avoid_crushing_player(player_to_action, all_players, 0.4)