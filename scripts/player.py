from grsim_ros_bridge_msgs.msg import SSL
from config import LATERAL_LEFT_ACTION_AREA_X_MAX, LATERAL_LEFT_ACTION_AREA_X_MIN, LATERAL_LEFT_ACTION_AREA_Y_MAX, LATERAL_LEFT_ACTION_AREA_Y_MIN, LATERAL_RIGHT_ACTION_AREA_X_MAX, LATERAL_RIGHT_ACTION_AREA_X_MIN, LATERAL_RIGHT_ACTION_AREA_Y_MAX, LATERAL_RIGHT_ACTION_AREA_Y_MIN, CENTRAL_DEFENDER_ACTION_AREA_X_MAX, CENTRAL_DEFENDER_ACTION_AREA_X_MIN, CENTRAL_DEFENDER_ACTION_AREA_Y_MAX, CENTRAL_DEFENDER_ACTION_AREA_Y_MIN, GOALKEEPER_ACTION_AREA_X_MAX, GOALKEEPER_ACTION_AREA_X_MIN, GOALKEEPER_ACTION_AREA_Y_MAX, GOALKEEPER_ACTION_AREA_Y_MIN, LATERAL_RIGHT_DEFENDING_POSITION_X, LATERAL_RIGHT_DEFENDING_POSITION_Y, LATERAL_LEFT_DEFENDING_POSITION_X, LATERAL_LEFT_DEFENDING_POSITION_Y, CENTRAL_DEFENDER_DEFENDING_POSITION_X, CENTRAL_DEFENDER_DEFENDING_POSITION_Y, STRICKER_DEFENDING_POSITION_X, STRICKER_DEFENDING_POSITION_Y, GOALKEEPER_DEFENDING_POSITION_X, GOALKEEPER_DEFENDING_POSITION_Y, LATERAL_RIGHT_ATACKING_POSITION_X, LATERAL_RIGHT_ATACKING_POSITION_Y, LATERAL_LEFT_ATACKING_POSITION_X, LATERAL_LEFT_ATACKING_POSITION_Y, CENTRAL_DEFENDER_ATACKING_POSITION_X, CENTRAL_DEFENDER_ATACKING_POSITION_Y, STRICKER_ATACKING_POSITION_X, STRICKER_ATACKING_POSITION_Y, GOALKEEPER_ATACKING_POSITION_X, GOALKEEPER_ATACKING_POSITION_Y
from utils import get_angle_player_object, get_closer_player, get_distance_player_object

safe_distance_rival_player = 225
velocity_sideways = 0.1

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
        self.go_goal = True

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

    def pass_to_partner(self):
        self.go_goal = False

    def go_to_goal(self):
        self.go_goal = True

    def going_goal(self):
        return self.go_goal

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

    def ball_is_in_area(self, ball_coordenates, gap = 0):
        if (self.role == 'lateral_right'):
            if (ball_coordenates['x'] >= LATERAL_RIGHT_ACTION_AREA_X_MIN - gap and ball_coordenates['x'] <= LATERAL_RIGHT_ACTION_AREA_X_MAX + gap and
            ball_coordenates['y'] >= LATERAL_RIGHT_ACTION_AREA_Y_MIN - gap and ball_coordenates['y'] <= LATERAL_RIGHT_ACTION_AREA_Y_MAX + gap):
                return True
            else:
                return False
        elif (self.role == 'lateral_left'):
            if (ball_coordenates['x'] >= LATERAL_LEFT_ACTION_AREA_X_MIN - gap and ball_coordenates['x'] <= LATERAL_LEFT_ACTION_AREA_X_MAX + gap and
            ball_coordenates['y'] >= LATERAL_LEFT_ACTION_AREA_Y_MIN - gap and ball_coordenates['y'] <= LATERAL_LEFT_ACTION_AREA_Y_MAX + gap):
                return True
            else:
                return False
        elif (self.role == 'central_defender'):
            if (ball_coordenates['x'] >= CENTRAL_DEFENDER_ACTION_AREA_X_MIN - gap and ball_coordenates['x'] <= CENTRAL_DEFENDER_ACTION_AREA_X_MAX + gap and
            ball_coordenates['y'] >= CENTRAL_DEFENDER_ACTION_AREA_Y_MIN - gap and ball_coordenates['y'] <= CENTRAL_DEFENDER_ACTION_AREA_Y_MAX + gap):
                return True
        elif (self.role == 'goalkeeper'):
            if (ball_coordenates['x'] >= GOALKEEPER_ACTION_AREA_X_MIN - gap and ball_coordenates['x'] <= GOALKEEPER_ACTION_AREA_X_MAX + gap and
            ball_coordenates['y'] >= GOALKEEPER_ACTION_AREA_Y_MIN - gap and ball_coordenates['y'] <= GOALKEEPER_ACTION_AREA_Y_MAX + gap):
                return True
            else:
                return False
        else:
            # delantero no tiene limites de accion
            return True

    def go_defend(self, all_players):

        if (self.role == 'lateral_right'):
            goal = {'x':LATERAL_RIGHT_DEFENDING_POSITION_X, 'y':LATERAL_RIGHT_DEFENDING_POSITION_Y}
        elif (self.role == 'lateral_left'):
            goal = {'x':LATERAL_LEFT_DEFENDING_POSITION_X, 'y':LATERAL_LEFT_DEFENDING_POSITION_Y}
        elif (self.role == 'central_defender'):
            goal = {'x':CENTRAL_DEFENDER_DEFENDING_POSITION_X, 'y':CENTRAL_DEFENDER_DEFENDING_POSITION_Y}
        elif (self.role == 'stricker'):
            goal = {'x':STRICKER_DEFENDING_POSITION_X, 'y':STRICKER_DEFENDING_POSITION_Y}
        else:
            goal = {'x':GOALKEEPER_DEFENDING_POSITION_X, 'y':GOALKEEPER_DEFENDING_POSITION_Y}

        angle_to_rotate = get_angle_player_object(self.get_position(), goal, self.get_angle())
        if angle_to_rotate < 0.03 and angle_to_rotate > -0.03:
            # estoy mirando 
            self.stop_rotate()
            distance = get_distance_player_object(self.get_position(), goal)
            if distance < 105:
                # estoy en posicion
                self.stop_go()
            else:
                closer_player, distance_closer_player = get_closer_player(all_players, self)

                if distance_closer_player < safe_distance_rival_player:
                    # estoy por chocar a otro jugador
                    self.stop_go()
                    self.go_sideways(velocity_sideways) # trato de esquivarlo

                else:
                    self.stop_go_sideways()
                    self.go_forward(0.4)
                # si estoy lejos de la pelota, voy hacia adelante porque ya estoy posicionado

        elif angle_to_rotate>0:
            self.rotate_right(0.3)
        else:
            self.rotate_left(0.3)

    def go_atack(self, all_players):

        if (self.role == 'lateral_right'):
            goal = {'x':LATERAL_RIGHT_ATACKING_POSITION_X, 'y':LATERAL_RIGHT_ATACKING_POSITION_Y}
        elif (self.role == 'lateral_left'):
            goal = {'x':LATERAL_LEFT_ATACKING_POSITION_X, 'y':LATERAL_LEFT_ATACKING_POSITION_Y}
        elif (self.role == 'central_defender'):
            goal = {'x':CENTRAL_DEFENDER_ATACKING_POSITION_X, 'y':CENTRAL_DEFENDER_ATACKING_POSITION_Y}
        elif (self.role == 'stricker'):
            goal = {'x':STRICKER_ATACKING_POSITION_X, 'y':STRICKER_ATACKING_POSITION_Y}
        else:
            goal = {'x':GOALKEEPER_ATACKING_POSITION_X, 'y':GOALKEEPER_ATACKING_POSITION_Y}

        angle_to_rotate = get_angle_player_object(self.get_position(), goal, self.get_angle())
        if angle_to_rotate < 0.03 and angle_to_rotate > -0.03:
            # estoy mirando 
            self.stop_rotate()
            distance = get_distance_player_object(self.get_position(), goal)
            if distance < 105:
                # estoy en posicion
                self.stop_go()
            else:
                closer_player, distance_closer_player = get_closer_player(all_players, self)

                if distance_closer_player < safe_distance_rival_player:
                    # estoy por chocar a otro jugador
                    self.stop_go()
                    self.go_sideways(velocity_sideways) # trato de esquivarlo

                else:
                    self.stop_go_sideways()
                    self.go_forward(0.4)
                # si estoy lejos de la pelota, voy hacia adelante porque ya estoy posicionado

        elif angle_to_rotate>0:
            self.rotate_right(0.4)
        else:
            self.rotate_left(0.4)
