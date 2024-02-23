uint8 ERROR_NONE = 0
uint8 ERROR_TIMEOUT = 1
uint8 ERROR_STUCK = 2
uint8 ERROR_OBSTACLE = 3
uint8 ERROR_MISSING_TRANSFORM = 4

uint8 STATE_IDLE = 0
uint8 STATE_ROTATING_TOWARDS_GOAL = 1
uint8 STATE_TRANSLATING = 2
uint8 STATE_ROTATING_AT_GOAL = 3
uint8 STATE_STOPPING = 4

uint8 error
uint8 state
float64 distance_x
float64 distance_xy
float64 distance_yaw

