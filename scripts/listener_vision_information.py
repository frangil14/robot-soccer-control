#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from krssg_ssl_msgs.msg import SSL_DetectionFrame
from grsim_ros_bridge_msgs.msg import SSL
from utils import ball_player_min_distance, get_angle_player_ball

# we are the blue team

our_goalkeeper_position = {'x':0, 'y':0}
our_central_defender_position = {'x':0, 'y':0}
our_lateral_right_position = {'x':0, 'y':0}
our_lateral_left_position = {'x':0, 'y':0}
our_stricker_position = {'x':0, 'y':0}

rival_goalkeeper_position = {'x':0, 'y':0}
rival_central_defender_position = {'x':0, 'y':0}
rival_lateral_right_position = {'x':0, 'y':0}
rival_lateral_left_position = {'x':0, 'y':0}
rival_stricker_position = {'x':0, 'y':0}

ball_position = {'x':0, 'y':0}

our_lateral_left_angle = 0


def callback(data):
    global ball_position
    global our_goalkeeper_position
    global our_central_defender_position 
    global our_lateral_right_position 
    global our_lateral_left_position 
    global our_stricker_position 

    global rival_goalkeeper_position 
    global rival_central_defender_position 
    global rival_lateral_right_position 
    global rival_lateral_left_position 
    global rival_stricker_position

    global our_lateral_left_angle


    if len(data.balls)>0:
        # we have info about ball
        ball_position['x'] = data.balls[0].pixel_x
        ball_position['y'] = data.balls[0].pixel_y

    if len(data.robots_blue)>0:
        # we have info about our players
        for item in data.robots_blue:
            if item.robot_id == 0:
                our_lateral_left_position['x'] = item.pixel_x
                our_lateral_left_position['y'] = item.pixel_y
                our_lateral_left_angle = item.orientation
            elif item.robot_id == 1:
                our_central_defender_position['x'] = item.pixel_x
                our_central_defender_position['y'] = item.pixel_y
            elif item.robot_id == 2:
                our_lateral_right_position['x'] = item.pixel_x
                our_lateral_right_position['y'] = item.pixel_y
            elif item.robot_id == 3:
                our_stricker_position['x'] = item.pixel_x
                our_stricker_position['y'] = item.pixel_y
            elif item.robot_id == 4:
                our_goalkeeper_position['x'] = item.pixel_x
                our_goalkeeper_position['y'] = item.pixel_y



    if len(data.robots_yellow)>0:
        # we have info about rival players
        for item in data.robots_yellow:
            if item.robot_id == 0:
                rival_lateral_right_position['x'] = item.pixel_x
                rival_lateral_right_position['y'] = item.pixel_y
            elif item.robot_id == 1:
                rival_central_defender_position['x'] = item.pixel_x
                rival_central_defender_position['y'] = item.pixel_y
            elif item.robot_id == 2:
                rival_lateral_left_position['x'] = item.pixel_x
                rival_lateral_left_position['y'] = item.pixel_y
            elif item.robot_id == 3:
                rival_stricker_position['x'] = item.pixel_x
                rival_stricker_position['y'] = item.pixel_y
            elif item.robot_id == 4:
                rival_goalkeeper_position['x'] = item.pixel_x
                rival_goalkeeper_position['y'] = item.pixel_y

    # rospy.loginfo('ball' + str(ball_position))

    

if __name__ == '__main__':

    rospy.init_node('listener_vision_information', anonymous=True)

    rospy.Subscriber("vision", SSL_DetectionFrame, callback)

    publisher = rospy.Publisher("/robot_blue_0/cmd", SSL, queue_size=10)
    rate = rospy.Rate(10)
    msg = SSL()


    while not rospy.is_shutdown():

        data = [our_lateral_left_position, our_central_defender_position, our_lateral_right_position, our_stricker_position, our_goalkeeper_position]
        player_to_action = ball_player_min_distance(data, ball_position)[0]
        min_distance = ball_player_min_distance(data, ball_position)[1]

        angle_torotate_toget_ball = get_angle_player_ball(our_lateral_left_position, ball_position, min_distance, our_lateral_left_angle)
        rospy.loginfo('angle_torotate_toget_ball ' + str(angle_torotate_toget_ball))

        if angle_torotate_toget_ball < 0.03 and angle_torotate_toget_ball > -0.03:

            msg.cmd_vel.angular.z = 0.00

        elif angle_torotate_toget_ball>0:
            msg.cmd_vel.angular.z = 0.25
        else:
            msg.cmd_vel.angular.z = -0.25

        publisher.publish(msg)
        rate.sleep()