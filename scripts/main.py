#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from krssg_ssl_msgs.msg import SSL_DetectionFrame
from grsim_ros_bridge_msgs.msg import SSL
from utils import ball_player_min_distance, get_angle_player_object, get_player_rivalGoal_distance
from player import Player

# we are the blue team
our_goalkeeper = Player('blue')
our_central_defender = Player('blue')
our_lateral_right = Player('blue')
our_lateral_left = Player('blue')
our_stricker = Player('blue')

players = [our_lateral_left, our_central_defender, our_lateral_right, our_stricker, our_goalkeeper]

# rivals
rival_goalkeeper = Player('yellow')
rival_central_defender = Player('yellow')
rival_lateral_right = Player('yellow')
rival_lateral_left = Player('yellow')
rival_stricker = Player('yellow')

# ball 
ball_position = {'x':0, 'y':0}

# rival goal
RIVAL_GOAL = {'x':2, 'y':0}


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

    rival_lateral_right_publisher = rospy.Publisher("/robot_yellow_0/cmd", SSL, queue_size=10)
    rival_lateral_right.set_publisher(rival_lateral_right_publisher)

    rival_central_defender_publisher = rospy.Publisher("/robot_yellow_1/cmd", SSL, queue_size=10)
    rival_central_defender.set_publisher(rival_central_defender_publisher)

    rival_lateral_left_publisher = rospy.Publisher("/robot_yellow_2/cmd", SSL, queue_size=10)
    rival_lateral_left.set_publisher(rival_lateral_left_publisher)

    rival_stricker_publisher = rospy.Publisher("/robot_yellow_3/cmd", SSL, queue_size=10)
    rival_stricker.set_publisher(rival_stricker_publisher)

    rival_goalkeeper_publisher = rospy.Publisher("/robot_yellow_4/cmd", SSL, queue_size=10)
    rival_goalkeeper.set_publisher(rival_goalkeeper_publisher)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        for item in players:
            if not item.is_active():
                item.stop_go()
                item.stop_rotate()

        data = [item.get_position() for item in players]
        index_player, distance_player_ball = ball_player_min_distance(data, ball_position)

        player_to_action = players[ball_player_min_distance(data, ball_position)[0]]
        player_to_action.activate_player()
        angle_torotate_toget_ball = get_angle_player_object(player_to_action.get_position(), ball_position, distance_player_ball, player_to_action.get_angle())

        distance_player_rivalGoal = get_player_rivalGoal_distance(player_to_action.get_position(), RIVAL_GOAL)
        angle_torotate_rival_goal = get_angle_player_object(player_to_action.get_position(), RIVAL_GOAL, distance_player_rivalGoal, player_to_action.get_angle())
        rospy.loginfo('angle_torotate_toget_ball ' + str(angle_torotate_toget_ball))
        rospy.loginfo('distance with the ball ' + str(distance_player_ball))
        rospy.loginfo('angle_torotate_rival_goal ' + str(angle_torotate_rival_goal))
        rospy.loginfo('active player ' + str(index_player))
        rospy.loginfo('has the ball ' + str(player_to_action.has_the_ball()))
        

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
                    player_to_action.go_forward(0.1)

            elif angle_torotate_toget_ball>0:
                player_to_action.rotate_right(0.25)
            else:
                player_to_action.rotate_left(0.25)

        else:
            # tengo la pelota
            # tengo que acomodarme para mirar al arco

            if angle_torotate_rival_goal < 0.03 and angle_torotate_rival_goal > -0.03:
                # estoy mirando al arco
                player_to_action.stop_rotate()
                player_to_action.stop_dribbling()
                player_to_action.kick_ball()    # pateo
                player_to_action.lost_the_ball()
                
            elif angle_torotate_rival_goal>0:
                player_to_action.rotate_right(0.25)
            else:
                player_to_action.rotate_left(0.25)


        player_to_action.deactivate_player()
        rate.sleep()
