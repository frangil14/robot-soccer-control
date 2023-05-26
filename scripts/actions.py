#!/usr/bin/env python3

from utils import get_angle_player_object, get_closer_player
from config import *
import rospy


def locate_target(player_to_action, all_players, anglePlayerTarget, ball_position=None, distance_partner_player = None, distance_player_ball = None, target = 'goal'):
    gap = 0.03
    if target == 'partner':
        gap = 0.15

    if anglePlayerTarget < gap and anglePlayerTarget > -gap:
        # estoy mirando al arco
        player_to_action.stop_rotate()

        if target == 'goal':
            go_rival_goal(player_to_action, all_players, ball_position)
        elif target == 'partner':
            pass_to_partner(player_to_action, distance_partner_player)
        elif target == 'ball':
            go_search_ball(player_to_action, all_players, distance_player_ball)


    elif (anglePlayerTarget>0 and anglePlayerTarget < 4) or anglePlayerTarget<-4:
        player_to_action.rotate_right(0.30)
    else:
        player_to_action.rotate_left(0.30)


def avoid_crushing_player(player_to_action, all_players):
    closer_player, distance_closer_player = get_closer_player(all_players, player_to_action)
    angle_player_closerPlayer = get_angle_player_object(player_to_action, closer_player.get_position(), player_to_action.get_angle())

    if distance_closer_player < safe_distance_rival_player and (angle_player_closerPlayer < 0.03 and angle_player_closerPlayer > -0.03):
        # estoy por chocar a otro jugador

        player_to_action.stop_go()
        player_to_action.go_sideways(velocity_sideways) # trato de esquivarlo

    else:
        player_to_action.stop_go_sideways()
        player_to_action.go_forward(0.2)

def go_rival_goal(player_to_action, all_players, ball_position):

    avoid_crushing_player(player_to_action, all_players)

    if (player_to_action.get_position()['x'] >= BLUE_SMALL_AREA_X_MIN and player_to_action.get_position()['x'] <= BLUE_SMALL_AREA_X_MAX and
    player_to_action.get_position()['y'] >= BLUE_SMALL_AREA_Y_MIN and player_to_action.get_position()['y'] <= BLUE_SMALL_AREA_Y_MAX):

        # estamos dentro del area rival
        rospy.loginfo('estamos dentro del area rival')
        player_to_action.stop_dribbling()
        player_to_action.kick_ball(3.0)    # pateo
        player_to_action.lost_the_ball()

    if not player_to_action.ball_is_in_area(ball_position, reaching_limit_area):
        # estoy al limite de mi area, le doy la pelota a un companero
        rospy.loginfo('estoy al limite de mi area')
        player_to_action.stop_go()
        player_to_action.pass_to_partner()

def pass_to_partner(player_to_action, distance_partner_player):
    # estoy mirando a mi companero 
    rospy.loginfo('YELLOW estoy mirando a mi companero')
    player_to_action.stop_rotate()
    player_to_action.stop_dribbling()
    player_to_action.kick_ball(3.0 * (distance_partner_player/1800))    # doy el pase
    player_to_action.go_to_goal()
    player_to_action.lost_the_ball()

def go_search_ball(player_to_action, all_players, distance_player_ball):
    if distance_player_ball < 105:
        rospy.loginfo('YELLOW estoy al lado de la pelota')
        # si estoy al lado de la pelota
        player_to_action.stop_go()
        rospy.loginfo('AGARRE LA PELOTA')
        player_to_action.got_the_ball()
        player_to_action.start_dribbling()
    else:
        # si estoy lejos de la pelota, voy hacia adelante porque ya estoy posicionado
        # aca tenemos que chequear que no nos choquemos con otro jugador
        avoid_crushing_player(player_to_action, all_players)