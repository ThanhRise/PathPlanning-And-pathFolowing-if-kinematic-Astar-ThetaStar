import pygame
import math
from utils import Utils
from constant import Constant
from spot import Spot
from dynamicObstacle import DynamicObstacle
from robot import Robot
from scipy.spatial import KDTree
from environment import Environment
from sensor import Sensor
# threading
import threading
from threading import Thread
from algorithm import *
import time
import copy

lock = threading.Lock()
robot = None
MAP = None
dynamicObstacles = None
sensor = None
run = True
planed = False
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
# NOTE:
# KDTree: không có thư viẹn có thể insert thêm node
# NHiều trường hợp không replan đuọc
# Thời gian replan quá lâu
# nếu replan mà vị trí robot sẽ trở thành chướng ngại vật thì sẽ gặp lỗi
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def path_planing(robot_move_thread, dynamic_obstacle_move_thread, sensor_detect_thread, display_thread): 
    global robot, MAP, planed, run, dynamicObstacles
    MAP.make_barrier_edge()
    pygame.display.update()
    fixEndPoint = MAP.end.get_real_pos(MAP.GAP)
    init_time = pygame.time.get_ticks()
    init_index = 0
    init_satety_score = 0
    robot.is_algorithm = "Svm_non_vision"
    # deep copy grid
    robot.grid_in_vison_robot = copy.deepcopy(MAP.grid)

    # reset grid for robot
    for row in range(MAP.ROWS):
        for spot in range(MAP.ROWS):
            if not robot.grid_in_vison_robot[row][spot].is_start() and not robot.grid_in_vison_robot[row][spot].is_end():
                robot.grid_in_vison_robot[row][spot].reset()

    for i in range(MAP.ROWS):
        for j in range(MAP.ROWS):
            robot.grid_in_vison_robot[i][j].update_neighbors_one_for_all(robot.grid_in_vison_robot)
            if i == 0 or j == 0 or i == MAP.ROWS - 1 or j == MAP.ROWS - 1:
                robot.grid_in_vison_robot[i][j].make_barrier()
    
    END_pos = (MAP.end.row, MAP.end.col)
    START_pos = (MAP.start.row, MAP.start.col)
    robot.detect_obstacle(MAP)   
    planed = True  
    print("robot.pathRb : OKKKKKKKKKKKKKKKKKKKKKKKKKKKK")

    robot.target = fixEndPoint
    last_time = pygame.time.get_ticks()
    last_time_back = None
    time_backwards = 0
    while Utils.distance_real((robot.x, robot.y), robot.target) > 15:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                print("Quit")
                pygame.quit()
                return
            
        if pygame.time.get_ticks() - last_time > 200:
            robot.detect_obstacle(MAP)
            delta_time = pygame.time.get_ticks()
            if robot.next_pos_move != None:

                next_pos_temp = int(robot.next_pos_move[0]/MAP.GAP), int(robot.next_pos_move[1]/MAP.GAP)
                robot.distance_to_end = BFS(robot.grid_in_vison_robot, END_pos)
                # robot.distance_to_end =  Partial_BFS(robot.grid_in_vison_robot, next_pos_temp, END_pos)
            else:
                robot.u = 0
                # time.sleep(0.5)
            last_time = pygame.time.get_ticks()

        if robot.backwards == True and pygame.time.get_ticks() - last_time_back >= time_backwards:
            if robot.theta > 0 and robot.theta < math.pi:
                robot.theta = robot.theta - math.pi
            elif robot.theta < 0 and robot.theta > -math.pi:
                robot.theta = robot.theta + math.pi
            elif robot.theta > math.pi:
                robot.theta = robot.theta - math.pi
            elif robot.theta < -math.pi:
                robot.theta = robot.theta + math.pi
            robot.backwards = False

        robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot(MAP, False)

        if robot.next_pos_move == None or robot.next_angle_move == None:
            if robot.theta > 0 and robot.theta < math.pi:
                robot.theta = robot.theta - math.pi
            elif robot.theta < 0 and robot.theta > -math.pi:
                robot.theta = robot.theta + math.pi
            elif robot.theta > math.pi:
                robot.theta = robot.theta - math.pi
            elif robot.theta < -math.pi: 
                robot.theta = robot.theta + math.pi
            robot.backwards = True
            robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot(MAP, True) 
            last_time_back = pygame.time.get_ticks()
            time_backwards = robot.time_step
        # ==================================================================================================
        if planed == True:
            # print("Start moving")
            robot_move_thread.start()
            # print("Start dynamic obstacle")
            dynamic_obstacle_move_thread.start()
            # print("Start sensor")
            sensor_detect_thread.start()
            # print("Start display")
            display_thread.start()
            planed = False

        # ==================================================================================================
        init_satety_score += robot.KDTree.query((robot.x, robot.y))[0]
        init_index += 1
    
    # print len path
    len_path = 0
    for i in range(len(robot.trail_set) - 1):
        len_path += Utils.distance_real(robot.trail_set[i], robot.trail_set[i + 1])
    
    # rounding 2 decimal
    print(round(len_path, 2))
    # print safety score
    safety_score = init_satety_score / init_index
    print(round(safety_score, 2))
    print(pygame.time.get_ticks() - init_time)
    # print number of collision
    print(robot.num_collision)
    run = False


def robot_move():
    global robot, MAP, run
    robot.draw(MAP.WIN)
    while run:
        while Utils.distance_real((robot.x, robot.y), robot.target) > 2 and Utils.distance_real((robot.x, robot.y), MAP.end.get_real_pos(MAP.GAP)) > 2 and run:
            robot.move_to()



def dynamic_obstacle_move():
    global MAP, dynamicObstacles, run
    last_time = pygame.time.get_ticks()
    while run:
        # MAP.draw_not_update()
        dt = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        for obstacle in dynamicObstacles:
            obstacle.move(MAP.grid, dt)
        #     obstacle.draw(MAP.WIN)
        # for obstacle in dynamicObstacles:


def sensor_detect():
    global robot, MAP, dynamicObstacles, sensor, run
    last_time = pygame.time.get_ticks()
    while run:
        if pygame.time.get_ticks() - last_time > 200:
            robot.dynamic_obstacles_replan = list(
                sensor.detect_replan(robot, dynamicObstacles))
            last_time = pygame.time.get_ticks()
        # write log collision file
            for obs in dynamicObstacles:
                if Utils.distance_real((robot.x, robot.y), (obs.x, obs.y)) < obs.d + robot.width/2 -1:
                    print("Collision")
                    robot.num_collision += 1
                    # pygame.image.save(MAP.WIN, f'C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/collision/' + str(pygame.time.get_ticks()) + '.png')
                    Utils.log_collision(Constant.COLLISIONFILE, pygame.time.get_ticks(), MAP.MAP_NAME, robot, dynamicObstacles)


def display():
    global robot, MAP, dynamicObstacles, run
    while run:
        
        # MAP.draw_not_update(robot.grid_in_vison_robot)
        MAP.draw_not_update_check_obs(robot.grid_in_vison_robot, MAP.grid)
        pygame.draw.circle(MAP.WIN, Constant.RED, (robot.x, robot.y), 100, 2)
        for obstacle in dynamicObstacles:
            obstacle.draw(MAP.WIN)
        # for location in range(len(robot.pathRb) - 1):
        #     pygame.draw.line(MAP.WIN, Constant.BLUE, robot.pathRb[location], robot.pathRb[location + 1], 2)
        pygame.draw.circle(MAP.WIN, Constant.RED, robot.target, 20)
    
        robot.rotated = pygame.transform.rotozoom(robot.img, math.degrees(-robot.theta), 1)
        robot.rect = robot.rotated.get_rect(center=(robot.x, robot.y))
        robot.draw(MAP.WIN)
        robot.trail((robot.x, robot.y), MAP.WIN, Constant.GREEN)
        pygame.display.update()

# Initialize pygame


def main(test=False, map_name=None, num_dynamic_obstacle=None):
    global MAP, run, planed, dynamicObstacles, robot, sensor

    run = True
    WIDTH = Constant.WIDTH
    ROWS = Constant.ROWS
    pygame.init()
    MAP = Environment(WIDTH, ROWS)
    fileName = Constant.MAPFILE
    dynamicFileName = Constant.DYNOBSTACLEFILE

    robot_move_thread = Thread(target=robot_move, name="robot_move")
    dynamic_obstacle_move_thread = Thread(
        target=dynamic_obstacle_move, name="dynamic_obstacle_move")
    sensor_detect_thread = Thread(target=sensor_detect, name="sensor_detect")
    display_thread = Thread(target=display, name="display")

    MASK = False
    if test:
        MAP.MAP_NAME = map_name
        MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(fileName, MAP.grid, MAP.MAP_NAME, MAP.ROWS)
        dynamicObstacles = Utils.generate_dynamic_obstacles(num_dynamic_obstacle, MAP)
        MAP.draw()
        pygame.display.update()
        robot = Robot(MAP.start.get_real_pos(MAP.GAP), "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/Image.png", 10)
        sensor = Sensor(dynamicObstacles)
        path_planing(robot_move_thread, dynamic_obstacle_move_thread,
                     sensor_detect_thread, display_thread)
    while run:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit")
                run = False

            # load map
            if MASK == False:
                MAP.draw()
            list_key = [pygame.K_1, pygame.K_2, pygame.K_3,
                        pygame.K_4, pygame.K_5, pygame.K_6,
                        pygame.K_7, pygame.K_8, pygame.K_9,
                        pygame.K_0, pygame.K_F1, pygame.K_F2,
                        pygame.K_F3, pygame.K_F4, pygame.K_F5,
                        pygame.K_F6, pygame.K_F7, pygame.K_F8,
                        pygame.K_F9, pygame.K_F10, pygame.K_F11]
            

            if event.type == pygame.KEYDOWN:
                for i in range(len(list_key)):
                    if event.key == list_key[i]:
                        print("Load map")
                        MAP.MAP_NAME = "map" + str(i + 1) + "_"
                        MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(fileName, MAP.grid, MAP.MAP_NAME, MAP.ROWS)
                        dynamicObstacles = Utils.generate_dynamic_obstacles(30, MAP)
                        MAP.draw()
                        pygame.display.update()
            

            if pygame.mouse.get_pressed()[0]:  # LEFT 
                pos = pygame.mouse.get_pos()
                row, col = MAP.get_clicked_pos(pos)
                print(row, col)
                spot = MAP.grid[row][col]
                if not MAP.start and spot != MAP.end:
                    MAP.start = spot
                    MAP.start.make_start()

                elif not MAP.end and spot != MAP.start:
                    MAP.end = spot
                    MAP.end.make_end()

                elif spot != MAP.end and spot != MAP.start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = MAP.get_clicked_pos(pos)
                spot = MAP.grid[row][col]
                spot.reset()
                if spot == MAP.start:
                    MAP.start = None
                elif spot == MAP.end:
                    MAP.end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and MAP.start and MAP.end:
                    MASK = True
                    robot = Robot(MAP.start.get_real_pos(MAP.GAP), "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/Image.png", 10)
                    sensor = Sensor(dynamicObstacles)
                    path_planing(robot_move_thread, dynamic_obstacle_move_thread,
                                 sensor_detect_thread, display_thread)
                    break

    robot_move_thread.join()
    dynamic_obstacle_move_thread.join()
    sensor_detect_thread.join()
    display_thread.join()

    print("Done")
    pygame.quit() 


if __name__ == '__main__':
    # main()
    for i in range(10):
        main(True, "map3_", 50) 
