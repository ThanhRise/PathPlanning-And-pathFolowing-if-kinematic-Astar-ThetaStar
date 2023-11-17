import pygame

class Constant:
    WIDTH = 800
    ROWS = 50
    GREY = (128, 128, 128)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    PURPLE = (128, 0, 128)
    ORANGE = (255, 165, 0)
    TURQUOISE = (64, 224, 208)
    
    # MAPFILE
    MAPFILE = "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/MAP.txt"
    DYNOBSTACLEFILE = "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/dynamicObstacles.txt"
    COLLISIONFILE = "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/log_collison.txt"
    INFO_PATH_FILE = "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/log_info_path.csv"

    OBSTACLE_DETECTION_DISTANCE = 100
    OBSTACLE_REPLANNING_DISTANCE = 150
    MIN_APROXIMATE_ANGLE = 0.1
    MIN_APROXIMATE_DISTANCE = 3
    ABILITY_REPLAN_LOW = -1
    ABILITY_REPLAN_HIGH = 1
