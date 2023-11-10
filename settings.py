
# GENERAL SETTINGS
RESOLUTION = (1080, 720)
HALF_RESOLUTION = (RESOLUTION[0] // 2, RESOLUTION[1] // 2)
BACKGROUND = (100, 100, 100)
FOREGROUND = (255, 255, 255)

# GAME SETTINGS

DT = 0.0001             # in seconds
GRAVITY = 9.81          # in meter / second^2
SPATIAL_LIMITS = 20     # in meter (is edge of HOLY CUBE)
SPHERE_RADIUS = 1       # in meter
SIZE_RATIO = SPHERE_RADIUS / SPATIAL_LIMITS

PENDULUM_ROPE_LENGTH = 2   # in meter

OBJECT_COUNT = 3
OBJECT_LAYOUT = (0, 1, 2)
OBJECT_INIT_VEL_LAYOUT = ((0, 0, 0),
                          (0, 0, 0),
                          (0, 0, 0))    # in meter / second
OBJECT_MASS_LAYOUT = (10, 10, 10)  # in kilogram
OBJECT_INIT_POS_LAYOUT = ((-0.6, -1, 0),
                          (0, -1, 0),
                          (0.6, -1, 0))
OBJECT_RADIUS_LAYOUT = (1, 1, 1)

SPRING_COUNT = 2
SPRING_LAYOUT = ((0, 1), (1, 2))
SPRING_K_LAYOUT = (100, 100)
SPRING_RELAXED_LEN_LAYOUT = (2, 2)


# CAMERA SETTINGS
# The simulation center is at center of a sphere
# The camera can move with keep distance
BALL_SIZE = 5  # 5 by default all other values creates illusion and doesn't represent real space

# MOUSE CONTROL
SENSITIVITY = 10    # smaller is better



