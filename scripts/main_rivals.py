#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from krssg_ssl_msgs.msg import SSL_DetectionFrame
from grsim_ros_bridge_msgs.msg import SSL
from utils import get_angle_player_object, they_have_the_ball, get_active_player, ball_player_min_distance, get_closer_player, get_closer_partner
from player import Player
from config import BLUE_GOAL, BLUE_SMALL_AREA_X_MAX, BLUE_SMALL_AREA_X_MIN, BLUE_SMALL_AREA_Y_MIN, BLUE_SMALL_AREA_Y_MAX
from actions import locate_target

# we are the yellow team
our_goalkeeper = Player('yellow', 'goalkeeper')
our_central_defender = Player('yellow', 'central_defender')
our_lateral_right = Player('yellow', 'lateral_right')
our_lateral_left = Player('yellow', 'lateral_left')
our_stricker = Player('yellow', 'stricker')

players = [our_lateral_right, our_central_defender, our_lateral_left, our_stricker, our_goalkeeper]

# rivals
rival_goalkeeper = Player('blue', 'goalkeeper')
rival_central_defender = Player('blue', 'central_defender')
rival_lateral_right = Player('blue', 'lateral_right')
rival_lateral_left = Player('blue', 'lateral_left')
rival_stricker = Player('blue', 'stricker')

rival_players = [rival_lateral_left, rival_central_defender, rival_lateral_right, rival_stricker, rival_goalkeeper]

# ball 
ball_position = {'x':0, 'y':0}
all_players = players + rival_players
RIVAL_GOAL = BLUE_GOAL

def we_have_the_ball(players, distance_player_ball):
    if distance_player_ball > 120:
        return False
    else:
        for player in players:
            if player.has_the_ball():
                return True
        return False

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
        # we have info about rival players
        for item in data.robots_blue:
            if item.robot_id == 0:
                rival_lateral_left.set_position(item.pixel_x, item.pixel_y)
                rival_lateral_left.set_angle(item.orientation)
            elif item.robot_id == 1:
                rival_central_defender.set_position(item.pixel_x, item.pixel_y)
                rival_central_defender.set_angle(item.orientation)
            elif item.robot_id == 2:
                rival_lateral_right.set_position(item.pixel_x, item.pixel_y)
                rival_lateral_right.set_angle(item.orientation)
            elif item.robot_id == 3:
                rival_stricker.set_position(item.pixel_x, item.pixel_y)
                rival_stricker.set_angle(item.orientation)
            elif item.robot_id == 4:
                rival_goalkeeper.set_position(item.pixel_x, item.pixel_y)
                rival_goalkeeper.set_angle(item.orientation)


    if len(data.robots_yellow)>0:
        # we have info about rival players
        for item in data.robots_yellow:
            if item.robot_id == 0:
                our_lateral_right.set_position(item.pixel_x, item.pixel_y)
                our_lateral_right.set_angle(item.orientation)
            elif item.robot_id == 1:
                our_central_defender.set_position(item.pixel_x, item.pixel_y)
                our_central_defender.set_angle(item.orientation)
            elif item.robot_id == 2:
                our_lateral_left.set_position(item.pixel_x, item.pixel_y)
                our_lateral_left.set_angle(item.orientation)
            elif item.robot_id == 3:
                our_stricker.set_position(item.pixel_x, item.pixel_y)
                our_stricker.set_angle(item.orientation)
            elif item.robot_id == 4:
                our_goalkeeper.set_position(item.pixel_x, item.pixel_y)
                our_goalkeeper.set_angle(item.orientation)

    

if __name__ == '__main__':

    rospy.init_node('listener_vision_information', anonymous=True)

    rospy.Subscriber("vision", SSL_DetectionFrame, callback)

    our_lateral_right_publisher = rospy.Publisher("/robot_yellow_0/cmd", SSL, queue_size=10)
    our_lateral_right.set_publisher(our_lateral_right_publisher)

    our_central_defender_publisher = rospy.Publisher("/robot_yellow_1/cmd", SSL, queue_size=10)
    our_central_defender.set_publisher(our_central_defender_publisher)

    our_lateral_left_publisher = rospy.Publisher("/robot_yellow_2/cmd", SSL, queue_size=10)
    our_lateral_left.set_publisher(our_lateral_left_publisher)

    our_stricker_publisher = rospy.Publisher("/robot_yellow_3/cmd", SSL, queue_size=10)
    our_stricker.set_publisher(our_stricker_publisher)

    our_goalkeeper_publisher = rospy.Publisher("/robot_yellow_4/cmd", SSL, queue_size=10)
    our_goalkeeper.set_publisher(our_goalkeeper_publisher)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        for item in players:
            if not item.is_active():
                item.stop_go()
                item.stop_go_sideways()
                item.stop_rotate()

        players_temp = players.copy()

        # solo para ver: el jugador mas cercano a la pelota
        our_players_positions = [item.get_position() for item in players]
        closer_player = players[ball_player_min_distance(our_players_positions, ball_position)[0]]

        # jugador activo: tiene en cuenta ademas las areas definidas para cada player
        player_to_action, distance_player_ball = get_active_player(players_temp, ball_position)


        player_to_action.activate_player()
        angle_torotate_toget_ball = get_angle_player_object(player_to_action, ball_position, player_to_action.get_angle())
        angle_torotate_rival_goal = get_angle_player_object(player_to_action, RIVAL_GOAL, player_to_action.get_angle())

        # rospy.loginfo('angle_torotate_toget_ball ' + str(angle_torotate_toget_ball))
        # rospy.loginfo('distance with the ball ' + str(distance_player_ball))
        # rospy.loginfo('angle_torotate_rival_goal ' + str(angle_torotate_rival_goal))
        # rospy.loginfo('OUR closer player ' + str(closer_player.get_role()))
        rospy.loginfo('YELLOW active player ' + str(player_to_action.get_role()))
        # rospy.loginfo('OUR closer partner ' + str(partner_player.get_role()))
        # rospy.loginfo('distancia con nuestro companero ' + str(distance_partner_player))
        # rospy.loginfo('has the ball ' + str(player_to_action.has_the_ball()))
        # rospy.loginfo('ball position ' + str(ball_position))
        # rospy.loginfo('they have the ball ' + str(they_have_the_ball(all_players, ball_position, 130, player_to_action.get_team())))
        # rospy.loginfo('RIVAL closer player ' + str(rival_player_closer.get_role()))
        # rospy.loginfo('distance player rival ' + str(distance_player_rival))
        # rospy.loginfo('angle_torotate_partner_player ' + str(angle_torotate_partner_player))

        # la primera data que tira la camara esta equivocada
        if angle_torotate_toget_ball == 0 and distance_player_ball == 0 and ball_position['x'] == 0 and ball_position['y'] == 0:
            pass
        else:

            # aca empieza la logica para activar los jugadores

            if we_have_the_ball(players, distance_player_ball):
                rospy.loginfo('YELLOW has the ball')
                for player in players:
                    if not item.is_active():
                        player.go_atack(all_players)
                # nosotros tenemos la pelota
                # tengo que acomodarme para mirar al arco

                # podemos chequear si se acerca al limite de su area, y dar un pase

                if player_to_action.going_goal():
                    # estoy yendo al arco
                    rospy.loginfo('YELLOW esta yendo al arco')

                    locate_target(player_to_action, all_players, angle_torotate_rival_goal, ball_position)
                
                else:
                    # no estoy yendo al arco, estoy buscando a mi companero
                    rospy.loginfo('YELLOW busco mi companero')
                    partner_player, distance_partner_player = get_closer_partner(players, player_to_action)

                    angle_torotate_partner_player = get_angle_player_object(player_to_action, partner_player.get_position(), player_to_action.get_angle())
                    locate_target(player_to_action, all_players, angle_torotate_partner_player, None, distance_partner_player, None, 'partner')
            elif they_have_the_ball(all_players, ball_position, 130, player_to_action.get_team()):
                # rospy.loginfo('BLUE has the ball')
                for player in players:
                    player.go_defend(all_players)

            else:
                rospy.loginfo('la pelota esta libre')
                # la pelota esta libre
                # se tiene que acomodar para buscar la pelota

                locate_target(player_to_action, all_players, angle_torotate_toget_ball, None, None,distance_player_ball, 'ball')



        player_to_action.deactivate_player()
        rate.sleep()