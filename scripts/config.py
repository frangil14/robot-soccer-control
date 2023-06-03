# constants

FIELD_X_MIN = -2000
FIELD_X_MAX = 2000
FIELD_Y_MIN = -2000
FIELD_Y_MAX = 2000

CENTRAL_CIRCLE_RADIUS = 500

BLUE_SMALL_AREA_X_MAX = -1000
BLUE_SMALL_AREA_X_MIN = FIELD_Y_MIN
BLUE_SMALL_AREA_Y_MIN = -1000
BLUE_SMALL_AREA_Y_MAX = 1000

YELLOW_GOAL_Y_MIN = -500
YELLOW_GOAL_Y_MAX = 500


YELLOW_SMALL_AREA_X_MAX = FIELD_X_MAX
YELLOW_SMALL_AREA_X_MIN = 1000
YELLOW_SMALL_AREA_Y_MAX = 1000
YELLOW_SMALL_AREA_Y_MIN = -1000



# ------ areas ------

BLUE_LATERAL_RIGHT_ACTION_AREA_X_MIN = FIELD_X_MIN
BLUE_LATERAL_RIGHT_ACTION_AREA_X_MAX = FIELD_X_MAX
BLUE_LATERAL_RIGHT_ACTION_AREA_Y_MIN = FIELD_Y_MIN
BLUE_LATERAL_RIGHT_ACTION_AREA_Y_MAX = YELLOW_GOAL_Y_MIN

BLUE_LATERAL_LEFT_ACTION_AREA_X_MIN = FIELD_X_MIN
BLUE_LATERAL_LEFT_ACTION_AREA_X_MAX = FIELD_X_MAX
BLUE_LATERAL_LEFT_ACTION_AREA_Y_MIN = YELLOW_GOAL_Y_MAX
BLUE_LATERAL_LEFT_ACTION_AREA_Y_MAX = FIELD_Y_MAX

BLUE_CENTRAL_DEFENDER_ACTION_AREA_X_MIN = FIELD_X_MIN
BLUE_CENTRAL_DEFENDER_ACTION_AREA_X_MAX = CENTRAL_CIRCLE_RADIUS
BLUE_CENTRAL_DEFENDER_ACTION_AREA_Y_MIN = BLUE_SMALL_AREA_Y_MIN + (FIELD_Y_MIN - BLUE_SMALL_AREA_Y_MIN)/2
BLUE_CENTRAL_DEFENDER_ACTION_AREA_Y_MAX = BLUE_SMALL_AREA_Y_MAX + (FIELD_Y_MAX - BLUE_SMALL_AREA_Y_MAX)/2

BLUE_GOALKEEPER_ACTION_AREA_X_MIN = FIELD_X_MIN
BLUE_GOALKEEPER_ACTION_AREA_X_MAX = BLUE_SMALL_AREA_X_MAX
BLUE_GOALKEEPER_ACTION_AREA_Y_MIN = BLUE_SMALL_AREA_Y_MIN
BLUE_GOALKEEPER_ACTION_AREA_Y_MAX = BLUE_SMALL_AREA_Y_MAX

YELLOW_LATERAL_RIGHT_ACTION_AREA_X_MIN = FIELD_X_MIN
YELLOW_LATERAL_RIGHT_ACTION_AREA_X_MAX = FIELD_X_MAX
YELLOW_LATERAL_RIGHT_ACTION_AREA_Y_MIN = YELLOW_GOAL_Y_MAX
YELLOW_LATERAL_RIGHT_ACTION_AREA_Y_MAX = FIELD_Y_MAX

YELLOW_LATERAL_LEFT_ACTION_AREA_X_MIN = FIELD_X_MIN
YELLOW_LATERAL_LEFT_ACTION_AREA_X_MAX = FIELD_X_MAX
YELLOW_LATERAL_LEFT_ACTION_AREA_Y_MIN = FIELD_Y_MIN
YELLOW_LATERAL_LEFT_ACTION_AREA_Y_MAX = YELLOW_GOAL_Y_MIN

YELLOW_CENTRAL_DEFENDER_ACTION_AREA_X_MIN = -CENTRAL_CIRCLE_RADIUS
YELLOW_CENTRAL_DEFENDER_ACTION_AREA_X_MAX = FIELD_X_MAX
YELLOW_CENTRAL_DEFENDER_ACTION_AREA_Y_MIN = YELLOW_SMALL_AREA_Y_MIN + (FIELD_Y_MIN - YELLOW_SMALL_AREA_Y_MIN)/2
YELLOW_CENTRAL_DEFENDER_ACTION_AREA_Y_MAX = YELLOW_SMALL_AREA_Y_MAX + (FIELD_Y_MAX - YELLOW_SMALL_AREA_Y_MAX)/2

YELLOW_GOALKEEPER_ACTION_AREA_X_MIN = YELLOW_SMALL_AREA_X_MIN
YELLOW_GOALKEEPER_ACTION_AREA_X_MAX = FIELD_X_MAX
YELLOW_GOALKEEPER_ACTION_AREA_Y_MIN = BLUE_SMALL_AREA_Y_MIN
YELLOW_GOALKEEPER_ACTION_AREA_Y_MAX = BLUE_SMALL_AREA_Y_MAX

# ------ defending positions ------

BLUE_LATERAL_RIGHT_DEFENDING_POSITION_X = BLUE_SMALL_AREA_X_MAX
BLUE_LATERAL_RIGHT_DEFENDING_POSITION_Y = -BLUE_SMALL_AREA_Y_MAX

BLUE_LATERAL_LEFT_DEFENDING_POSITION_X = BLUE_SMALL_AREA_X_MAX
BLUE_LATERAL_LEFT_DEFENDING_POSITION_Y = BLUE_SMALL_AREA_Y_MAX

BLUE_CENTRAL_DEFENDER_DEFENDING_POSITION_X = BLUE_SMALL_AREA_X_MAX - CENTRAL_CIRCLE_RADIUS
BLUE_CENTRAL_DEFENDER_DEFENDING_POSITION_Y = 0

BLUE_STRICKER_DEFENDING_POSITION_X = 0
BLUE_STRICKER_DEFENDING_POSITION_Y = 0

BLUE_GOALKEEPER_DEFENDING_POSITION_X = FIELD_X_MIN
BLUE_GOALKEEPER_DEFENDING_POSITION_Y = 0

YELLOW_LATERAL_RIGHT_DEFENDING_POSITION_X = YELLOW_SMALL_AREA_X_MIN
YELLOW_LATERAL_RIGHT_DEFENDING_POSITION_Y = YELLOW_SMALL_AREA_Y_MAX

YELLOW_LATERAL_LEFT_DEFENDING_POSITION_X = YELLOW_SMALL_AREA_X_MIN
YELLOW_LATERAL_LEFT_DEFENDING_POSITION_Y = YELLOW_SMALL_AREA_Y_MIN

YELLOW_CENTRAL_DEFENDER_DEFENDING_POSITION_X = YELLOW_SMALL_AREA_X_MIN + CENTRAL_CIRCLE_RADIUS
YELLOW_CENTRAL_DEFENDER_DEFENDING_POSITION_Y = 0

YELLOW_STRICKER_DEFENDING_POSITION_X = 0
YELLOW_STRICKER_DEFENDING_POSITION_Y = 0

YELLOW_GOALKEEPER_DEFENDING_POSITION_X = FIELD_X_MAX
YELLOW_GOALKEEPER_DEFENDING_POSITION_Y = 0

# ------ atacking positions ------

BLUE_LATERAL_RIGHT_ATACKING_POSITION_X = YELLOW_SMALL_AREA_X_MIN
BLUE_LATERAL_RIGHT_ATACKING_POSITION_Y = -BLUE_SMALL_AREA_Y_MAX

BLUE_LATERAL_LEFT_ATACKING_POSITION_X = YELLOW_SMALL_AREA_X_MIN
BLUE_LATERAL_LEFT_ATACKING_POSITION_Y = BLUE_SMALL_AREA_Y_MAX

BLUE_CENTRAL_DEFENDER_ATACKING_POSITION_X = 0
BLUE_CENTRAL_DEFENDER_ATACKING_POSITION_Y = 0

BLUE_STRICKER_ATACKING_POSITION_X = YELLOW_SMALL_AREA_X_MIN
BLUE_STRICKER_ATACKING_POSITION_Y = 0

BLUE_GOALKEEPER_ATACKING_POSITION_X = FIELD_X_MIN + 100
BLUE_GOALKEEPER_ATACKING_POSITION_Y = 0

YELLOW_LATERAL_RIGHT_ATACKING_POSITION_X = BLUE_SMALL_AREA_X_MAX
YELLOW_LATERAL_RIGHT_ATACKING_POSITION_Y = BLUE_SMALL_AREA_Y_MAX

YELLOW_LATERAL_LEFT_ATACKING_POSITION_X = BLUE_SMALL_AREA_X_MAX
YELLOW_LATERAL_LEFT_ATACKING_POSITION_Y = -BLUE_SMALL_AREA_Y_MAX

YELLOW_CENTRAL_DEFENDER_ATACKING_POSITION_X = 0
YELLOW_CENTRAL_DEFENDER_ATACKING_POSITION_Y = 0

YELLOW_STRICKER_ATACKING_POSITION_X = BLUE_SMALL_AREA_X_MAX
YELLOW_STRICKER_ATACKING_POSITION_Y = 0

YELLOW_GOALKEEPER_ATACKING_POSITION_X = FIELD_X_MAX - 100
YELLOW_GOALKEEPER_ATACKING_POSITION_Y = 0

# ------ game configs ------

safe_distance_rival_player = 225
reaching_limit_area = -100
velocity_sideways = 0.1