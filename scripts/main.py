#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from krssg_ssl_msgs.msg import SSL_DetectionFrame
from grsim_ros_bridge_msgs.msg import SSL
from utils import get_angle_player_object, they_have_the_ball, get_active_player, ball_player_min_distance, get_closer_player
from player import Player
from config import RIVAL_GOAL, RIVAL_ACTION_AREA_X_MAX, RIVAL_ACTION_AREA_X_MIN, RIVAL_ACTION_AREA_Y_MAX, RIVAL_ACTION_AREA_Y_MIN

# we are the blue team
our_goalkeeper = Player('blue', 'goalkeeper')
our_central_defender = Player('blue', 'central_defender')
our_lateral_right = Player('blue', 'lateral_right')
our_lateral_left = Player('blue', 'lateral_left')
our_stricker = Player('blue', 'stricker')

players = [our_lateral_left, our_central_defender, our_lateral_right, our_stricker, our_goalkeeper]

# rivals
rival_goalkeeper = Player('yellow', 'goalkeeper')
rival_central_defender = Player('yellow', 'central_defender')
rival_lateral_right = Player('yellow', 'lateral_right')
rival_lateral_left = Player('yellow', 'lateral_left')
rival_stricker = Player('yellow', 'stricker')

rival_players = [rival_lateral_left, rival_central_defender, rival_lateral_right, rival_stricker, rival_goalkeeper]

# ball 
ball_position = {'x':0, 'y':0}

all_players = players + rival_players


def callback(data):
    global ball_position

    global our_goalkeeper
    global our_central_defender
    global our_lateral_right
    global our_lateral_left
    global our_stricker

    global rival_goalkeeper
    global rival_central_defender
    global rival_lateral_right
    global rival_lateral_left
    global rival_stricker


    if len(data.balls)>0:
        # we have info about ball
        ball_position['x'] = data.balls[0].pixel_x
        ball_position['y'] = data.balls[0].pixel_y

    if len(data.robots_blue)>0:
        # we have info about our players
        for item in data.robots_blue:
            if item.robot_id == 0:
                our_lateral_left.set_position(item.pixel_x, item.pixel_y)
                our_lateral_left.set_angle(item.orientation)
            elif item.robot_id == 1:
                our_central_defender.set_position(item.pixel_x, item.pixel_y)
                our_central_defender.set_angle(item.orientation)
            elif item.robot_id == 2:
                our_lateral_right.set_position(item.pixel_x, item.pixel_y)
                our_lateral_right.set_angle(item.orientation)
            elif item.robot_id == 3:
                our_stricker.set_position(item.pixel_x, item.pixel_y)
                our_stricker.set_angle(item.orientation)
            elif item.robot_id == 4:
                our_goalkeeper.set_position(item.pixel_x, item.pixel_y)
                our_goalkeeper.set_angle(item.orientation)


    if len(data.robots_yellow)>0:
        # we have info about rival players
        for item in data.robots_yellow:
            if item.robot_id == 0:
                rival_lateral_right.set_position(item.pixel_x, item.pixel_y)
                rival_lateral_right.set_angle(item.orientation)
            elif item.robot_id == 1:
                rival_central_defender.set_position(item.pixel_x, item.pixel_y)
                rival_central_defender.set_angle(item.orientation)
            elif item.robot_id == 2:
                rival_lateral_left.set_position(item.pixel_x, item.pixel_y)
                rival_lateral_left.set_angle(item.orientation)
            elif item.robot_id == 3:
                rival_stricker.set_position(item.pixel_x, item.pixel_y)
                rival_stricker.set_angle(item.orientation)
            elif item.robot_id == 4:
                rival_goalkeeper.set_position(item.pixel_x, item.pixel_y)
                rival_goalkeeper.set_angle(item.orientation)

    

if __name__ == '__main__':

    rospy.init_node('listener_vision_information', anonymous=True)

    rospy.Subscriber("vision", SSL_DetectionFrame, callback)

    our_lateral_left_publisher = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)
    our_lateral_left.set_publisher(our_lateral_left_publisher)

    our_central_defender_publisher = rospy.Publisher("/robot_blue_1/cmd", SSL, queue_size=10)
    our_central_defender.set_publisher(our_central_defender_publisher)

    our_lateral_right_publisher = rospy.Publisher("/robot_blue_2/cmd", SSL, queue_size=10)
    our_lateral_right.set_publisher(our_lateral_right_publisher)

    our_stricker_publisher = rospy.Publisher("/robot_blue_3/cmd", SSL, queue_size=10)
    our_stricker.set_publisher(our_stricker_publisher)

    our_goalkeeper_publisher = rospy.Publisher("/robot_blue_4/cmd", SSL, queue_size=10)
    our_goalkeeper.set_publisher(our_goalkeeper_publisher)

    # no debemos publicar mensajes a los rivales

    # rival_lateral_right_publisher = rospy.Publisher("/robot_yellow_0/cmd", SSL, queue_size=10)
    # rival_lateral_right.set_publisher(rival_lateral_right_publisher)

    # rival_central_defender_publisher = rospy.Publisher("/robot_yellow_1/cmd", SSL, queue_size=10)
    # rival_central_defender.set_publisher(rival_central_defender_publisher)

    # rival_lateral_left_publisher = rospy.Publisher("/robot_yellow_2/cmd", SSL, queue_size=10)
    # rival_lateral_left.set_publisher(rival_lateral_left_publisher)

    # rival_stricker_publisher = rospy.Publisher("/robot_yellow_3/cmd", SSL, queue_size=10)
    # rival_stricker.set_publisher(rival_stricker_publisher)

    # rival_goalkeeper_publisher = rospy.Publisher("/robot_yellow_4/cmd", SSL, queue_size=10)
    # rival_goalkeeper.set_publisher(rival_goalkeeper_publisher)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        for item in players:
            if not item.is_active():
                # esto lo podemos mejorar, y podemos hacer que los jugadores No activos vayan a una posicion base, dependiendo si estamos atacando o defendiendo
                item.stop_go()
                item.stop_go_sideways()
                item.stop_rotate()

        players_temp = players.copy()

        # solo para ver: el jugador mas cercano a la pelota
        our_players_positions = [item.get_position() for item in players]
        closer_player = players[ball_player_min_distance(our_players_positions, ball_position)[0]]

        # jugador activo: tiene en cuenta ademas las areas definidas para cada player
        player_to_action, distance_player_ball = get_active_player(players_temp, ball_position)

        # rival mas cercano a nuestro player
        rival_players_positions = [item.get_position() for item in rival_players]
        rival_player_closer, distance_player_rival = get_closer_player(all_players, player_to_action, True)

        # companero mas cercano a nuestro player
        partner_player, distance_partner_player = get_closer_player(all_players, player_to_action)
        angle_torotate_partner_player = get_angle_player_object(player_to_action.get_position(), partner_player.get_position(), player_to_action.get_angle())

        player_to_action.activate_player()
        angle_torotate_toget_ball = get_angle_player_object(player_to_action.get_position(), ball_position, player_to_action.get_angle())
        angle_torotate_rival_goal = get_angle_player_object(player_to_action.get_position(), RIVAL_GOAL, player_to_action.get_angle())

        rospy.loginfo('angle_torotate_toget_ball ' + str(angle_torotate_toget_ball))
        rospy.loginfo('distance with the ball ' + str(distance_player_ball))
        rospy.loginfo('angle_torotate_rival_goal ' + str(angle_torotate_rival_goal))
        rospy.loginfo('OUR closer player ' + str(closer_player.get_role()))
        rospy.loginfo('OUR active player ' + str(player_to_action.get_role()))
        rospy.loginfo('OUR closer partner ' + str(partner_player.get_role()))
        rospy.loginfo('distancia con nuestro companero ' + str(distance_partner_player))
        rospy.loginfo('has the ball ' + str(player_to_action.has_the_ball()))
        rospy.loginfo('ball position ' + str(ball_position))
        rospy.loginfo('they have the ball ' + str(they_have_the_ball(all_players, ball_position, 130)))
        rospy.loginfo('RIVAL closer player ' + str(rival_player_closer.get_role()))
        rospy.loginfo('distance player rival ' + str(distance_player_rival))
        rospy.loginfo('angle_torotate_partner_player ' + str(angle_torotate_partner_player))

        # la primera data que tira la camara esta equivocada
        if angle_torotate_toget_ball == 0 and distance_player_ball == 0 and ball_position['x'] == 0 and ball_position['y'] == 0:
            pass
        else:

            # aca empieza la logica para activar los jugadores

            if they_have_the_ball(all_players, ball_position, 130):
                # ver que hacemos aca, tenemos que defender
                # por lo pronto hagamos que vayan a una posicion base definida
                pass

            else:
                # nosotros tenemos la pelota, o esta libre

                if not player_to_action.has_the_ball():
                    # no tiene la pelota
                    # se tiene que acomodar para buscar la pelota

                    if angle_torotate_toget_ball < 0.03 and angle_torotate_toget_ball > -0.03:
                            # se puso de frente a la pelota
                            player_to_action.stop_rotate()  # dejo de rotar

                            if distance_player_ball < 105:
                                # si estoy al lado de la pelota
                                player_to_action.stop_go()
                                player_to_action.got_the_ball()
                                player_to_action.start_dribbling()
                            else:
                                # si estoy lejos de la pelota, voy hacia adelante porque ya estoy posicionado
                                # aca tenemos que chequear que no nos choquemos con un rival

                                rival_player_closer, distance_player_rival = get_closer_player(all_players, player_to_action, True)

                                if distance_player_rival < 300:
                                    # estoy por chocar al rival
                                    player_to_action.stop_go()
                                    player_to_action.go_sideways(0.1) # trato de esquivarlo

                                else:
                                    player_to_action.stop_go_sideways()
                                    player_to_action.go_forward(0.2)

                    elif angle_torotate_toget_ball>0:
                            player_to_action.rotate_right(0.30)
                    else:
                            player_to_action.rotate_left(0.30)

                else:
                    # tengo la pelota
                    # tengo que acomodarme para mirar al arco

                    # podemos chequear si se acerca al limite de su area, y dar un pase

                    if player_to_action.going_goal():
                        # estoy yendo al arco

                        if angle_torotate_rival_goal < 0.03 and angle_torotate_rival_goal > -0.03:
                            # estoy mirando al arco
                            player_to_action.stop_rotate()
                            # aca tenemos que chequear que no nos choquemos con un rival

                            rival_player_closer, distance_player_rival = get_closer_player(all_players, player_to_action, True)

                            if distance_player_rival < 300:
                                # estoy por chocar al rival
                                player_to_action.stop_go()
                                player_to_action.go_sideways(0.1) # trato de esquivarlo

                            else:
                                player_to_action.stop_go_sideways()
                                player_to_action.go_forward(0.2)

                                if (player_to_action.get_position()['x'] >= RIVAL_ACTION_AREA_X_MIN and player_to_action.get_position()['x'] <= RIVAL_ACTION_AREA_X_MAX and
                                player_to_action.get_position()['y'] >= RIVAL_ACTION_AREA_Y_MIN and player_to_action.get_position()['y'] <= RIVAL_ACTION_AREA_X_MAX):

                                    # estamos dentro del area rival
                                    player_to_action.stop_dribbling()
                                    player_to_action.kick_ball(3.0)    # pateo
                                    player_to_action.lost_the_ball()

                                if not player_to_action.ball_is_in_area(ball_position, -200):
                                    # estoy al limite de mi area, le doy la pelota a un companero
                                    player_to_action.stop_go()
                                    player_to_action.pass_to_partner()
                    
                        elif angle_torotate_rival_goal>0:
                            player_to_action.rotate_right(0.30)
                        else:
                            player_to_action.rotate_left(0.30)
                    
                    else:
                        # no estoy yendo al arco, estoy buscando a mi companero
                        partner_player, distance_partner_player = get_closer_player(all_players, player_to_action)

                        angle_torotate_partner_player = get_angle_player_object(player_to_action.get_position(), partner_player.get_position(), player_to_action.get_angle())

                        if angle_torotate_partner_player < 0.15 and angle_torotate_partner_player > -0.15:
                            # estoy mirando a mi companero 
                            player_to_action.stop_rotate()
                            player_to_action.stop_dribbling()
                            player_to_action.kick_ball(3.0 * (distance_partner_player/2000))    # doy el pase
                            player_to_action.go_to_goal()
                            player_to_action.lost_the_ball()

                        elif angle_torotate_partner_player>0:
                            player_to_action.rotate_right(0.30)
                        else:
                            player_to_action.rotate_left(0.30)


        player_to_action.deactivate_player()
        rate.sleep()