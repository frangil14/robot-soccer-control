from grsim_ros_bridge_msgs.msg import SSL

class Player:
    def __init__(self, team):
        self.team = team
        self.position = {'x':0, 'y':0}
        self.angle = 0
        self.publisher = None
        self.msg = SSL()
        self.ball_possession = False
        self.active = False

    def rotate_right(self, velocity):
        if self.publisher is not None:
            self.msg.cmd_vel.angular.z = velocity
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def rotate_left(self, velocity):
        if self.publisher is not None:
            self.msg.cmd_vel.angular.z = -velocity
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def stop_rotate(self):
        if self.publisher is not None:
            self.msg.cmd_vel.angular.z = 0.00
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def go_forward(self, velocity):
        if self.publisher is not None:
            self.msg.cmd_vel.linear.x = velocity
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def go_backwards(self, velocity):
        if self.publisher is not None:
            self.msg.cmd_vel.linear.x = -velocity
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def stop_go(self):
        if self.publisher is not None:
            self.msg.cmd_vel.linear.x = 0
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def get_position(self):
        return self.position

    def set_position(self, x, y):
        self.position['x'] = x
        self.position['y'] = y

    def get_angle(self):
        return self.angle

    def set_angle(self, theta):
        self.angle = theta

    def set_publisher(self, publisher):
        self.publisher = publisher

    def got_the_ball(self):
        self.ball_possession = True

    def lost_the_ball(self):
        self.ball_possession = False

    def has_the_ball(self):
        return self.ball_possession

    def kick_ball(self):
        if self.publisher is not None:
            self.msg.kicker = True
            self.publisher.publish(self.msg)
            self.msg.kicker = False
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def start_dribbling(self):
        if self.publisher is not None:
            self.msg.dribbler = True
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def stop_dribbling(self):
        if self.publisher is not None:
            self.msg.dribbler = False
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def activate_player(self):
        self.active = True

    def deactivate_player(self):
        self.active = False

    def is_active(self):
        return self.active