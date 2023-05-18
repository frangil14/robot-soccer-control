from grsim_ros_bridge_msgs.msg import SSL

# constants

FIELD_X_MIN = -2000
FIELD_X_MAX = 2000
FIELD_Y_MIN = -2000
FIELD_Y_MAX = 2000
GOAL_Y_MIN = -500
GOAL_Y_MAX = 500
CENTRAL_CIRCLE_RADIUS = 500
SMALL_AREA_X_MAX = -1000
SMALL_AREA_Y_MIN = -1000
SMALL_AREA_Y_MAX = 1000



LATERAL_RIGHT_ACTION_AREA_X_MIN = FIELD_X_MIN
LATERAL_RIGHT_ACTION_AREA_X_MAX = FIELD_X_MAX
LATERAL_RIGHT_ACTION_AREA_Y_MIN = FIELD_Y_MIN
LATERAL_RIGHT_ACTION_AREA_Y_MAX = GOAL_Y_MIN

LATERAL_LEFT_ACTION_AREA_X_MIN = FIELD_X_MIN
LATERAL_LEFT_ACTION_AREA_X_MAX = FIELD_X_MAX
LATERAL_LEFT_ACTION_AREA_Y_MIN = GOAL_Y_MAX
LATERAL_LEFT_ACTION_AREA_Y_MAX = FIELD_Y_MAX

CENTRAL_DEFENDER_ACTION_AREA_X_MIN = FIELD_X_MIN
CENTRAL_DEFENDER_ACTION_AREA_X_MAX = CENTRAL_CIRCLE_RADIUS
CENTRAL_DEFENDER_ACTION_AREA_Y_MIN = SMALL_AREA_Y_MIN + (FIELD_Y_MIN - SMALL_AREA_Y_MIN)/2
CENTRAL_DEFENDER_ACTION_AREA_Y_MAX = SMALL_AREA_Y_MAX + (FIELD_Y_MAX - SMALL_AREA_Y_MAX)/2

GOALKEEPER_ACTION_AREA_X_MIN = FIELD_X_MIN
GOALKEEPER_ACTION_AREA_X_MAX = SMALL_AREA_X_MAX
GOALKEEPER_ACTION_AREA_Y_MIN = SMALL_AREA_Y_MIN
GOALKEEPER_ACTION_AREA_Y_MAX = SMALL_AREA_Y_MAX


class Player:
    def __init__(self, team, role):
        self.role = role
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

    def go_sideways(self, velocity):
        if self.publisher is not None:
            self.msg.cmd_vel.linear.y = velocity
            self.publisher.publish(self.msg)
        else:
            print('Publisher not configured')

    def stop_go_sideways(self):
        if self.publisher is not None:
            self.msg.cmd_vel.linear.y = 0
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

    def kick_ball(self, power):
        if self.publisher is not None:
            self.msg.kicker = power
            self.publisher.publish(self.msg)
            self.msg.kicker = 0.0
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

    def get_role(self):
        return self.role

    def get_team(self):
        return self.team

    def ball_is_in_area(self, ball_coordenates):
        if (self.role == 'lateral_right'):
            if (ball_coordenates['x'] >= LATERAL_RIGHT_ACTION_AREA_X_MIN and ball_coordenates['x'] <= LATERAL_RIGHT_ACTION_AREA_X_MAX and
            ball_coordenates['y'] >= LATERAL_RIGHT_ACTION_AREA_Y_MIN and ball_coordenates['y'] <= LATERAL_RIGHT_ACTION_AREA_Y_MAX):
                return True
            else:
                return False
        elif (self.role == 'lateral_left'):
            if (ball_coordenates['x'] >= LATERAL_LEFT_ACTION_AREA_X_MIN and ball_coordenates['x'] <= LATERAL_LEFT_ACTION_AREA_X_MAX and
            ball_coordenates['y'] >= LATERAL_LEFT_ACTION_AREA_Y_MIN and ball_coordenates['y'] <= LATERAL_LEFT_ACTION_AREA_Y_MAX):
                return True
            else:
                return False
        elif (self.role == 'central_defender'):
            if (ball_coordenates['x'] >= CENTRAL_DEFENDER_ACTION_AREA_X_MIN and ball_coordenates['x'] <= CENTRAL_DEFENDER_ACTION_AREA_X_MAX and
            ball_coordenates['y'] >= CENTRAL_DEFENDER_ACTION_AREA_Y_MIN and ball_coordenates['y'] <= CENTRAL_DEFENDER_ACTION_AREA_Y_MAX):
                return True
        elif (self.role == 'goalkeeper'):
            if (ball_coordenates['x'] >= GOALKEEPER_ACTION_AREA_X_MIN and ball_coordenates['x'] <= GOALKEEPER_ACTION_AREA_X_MAX and
            ball_coordenates['y'] >= GOALKEEPER_ACTION_AREA_Y_MIN and ball_coordenates['y'] <= GOALKEEPER_ACTION_AREA_Y_MAX):
                return True
            else:
                return False
                
        else:
            # delantero no tiene limites de accion
            return True
