#!/usr/bin/env python3
import rospy
from krssg_ssl_msgs.msg import SSL_DetectionFrame
from grsim_ros_bridge_msgs.msg import SSL
from utils import get_angle_player_object, they_have_the_ball, get_active_player, get_closer_partner, is_someone_in_between
from player import Player
from config import FIELD_X_MAX
from actions import locate_target


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

rival_goal = {'x':FIELD_X_MAX, 'y':0}


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

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        for player in players:
            if not player.is_active():
                player.stop_go()
                player.stop_go_sideways()
                player.stop_rotate()

        players_temp = players.copy()

        # jugador activo: tiene en cuenta ademas las areas definidas para cada player
        player_to_action, distance_player_ball = get_active_player(players_temp, ball_position)


        player_to_action.activate_player()

        partner_player = get_closer_partner(players, player_to_action)

        rospy.loginfo('ACTIVE player ' + str(player_to_action.get_role()))
        rospy.loginfo('closer player ' + str(partner_player.get_role()))
        rospy.loginfo('is_someone_in_between ' + str(is_someone_in_between(player_to_action, rival_goal, all_players)))
        rospy.loginfo('rival goal ' + str(rival_goal))
        rospy.loginfo('ball location ' + str(ball_position))


        # la primera data que tira la camara esta equivocada
        angle_torotate_toget_ball = get_angle_player_object(player_to_action, ball_position, player_to_action.get_angle())
        if angle_torotate_toget_ball == 0 and distance_player_ball == 0 and ball_position['x'] == 0 and ball_position['y'] == 0:
            pass

        else:
            # aca empieza la logica para activar los jugadores

            if we_have_the_ball(players, distance_player_ball):
                rospy.loginfo('BLUE has the ball')

                for player in players:
                    if not player.is_active():
                        player.go_attack(all_players,rospy)

                # nosotros tenemos la pelota
                # tengo que acomodarme para mirar al arco, o para darle la pelota a mi companero

                if player_to_action.going_goal():
                    # estoy yendo al arco
                    rospy.loginfo('BLUE esta yendo al arco')

                    output = locate_target(rospy,player_to_action, all_players, rival_goal, ball_position)

                    if output is not None:
                        rival_goal = output
                
                else:
                    # no estoy yendo al arco, estoy buscando a mi companero
                    rospy.loginfo('BLUE busco mi companero')
                    partner_player = get_closer_partner(players, player_to_action)

                    locate_target(rospy,player_to_action, all_players, partner_player.get_position(), None, 'partner')

            elif they_have_the_ball(all_players, ball_position, 130, player_to_action.get_team()):
                for player in players:
                    player.go_defend(all_players,rospy)

            else:
                rospy.loginfo('la pelota esta libre')
                locate_target(rospy,player_to_action, all_players, ball_position, None, 'ball')



        player_to_action.deactivate_player()
        rate.sleep()