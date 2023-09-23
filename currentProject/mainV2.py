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
    global robot, MAP, planed, run
    MAP.make_barrier_edge()
    pygame.display.update()
    fixEndPoint = MAP.end.get_real_pos(MAP.GAP)

    # deep copy grid
    robot.grid_in_vison_robot = copy.deepcopy(MAP.grid)

    # reset grid for robot
    for row in range(MAP.ROWS):
        for spot in range(MAP.ROWS):
            if not robot.grid_in_vison_robot[row][spot].is_start() and not robot.grid_in_vison_robot[row][spot].is_end():
                robot.grid_in_vison_robot[row][spot].reset()

    for i in range(MAP.ROWS):
        for j in range(MAP.ROWS):
            if i == 0 or j == 0 or i == MAP.ROWS - 1 or j == MAP.ROWS - 1:
                robot.grid_in_vison_robot[i][j].make_barrier()
    
    END_pos = (MAP.end.col, MAP.end.row)
    robot.detect_obstacle(MAP)   
    robot.distance_to_end = BFS(robot.grid_in_vison_robot, END_pos)


    planed = True  
    print("robot.pathRb : OKKKKKKKKKKKKKKKKKKKKKKKKKKKK")

    robot.target = fixEndPoint
    last_time = pygame.time.get_ticks()
    while Utils.distance_real((robot.x, robot.y), robot.target) > 5:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                print("Quit")
                pygame.quit()
                return
            
        if pygame.time.get_ticks() - last_time > 500:
            robot.detect_obstacle(MAP)

            delta_time = pygame.time.get_ticks()
            next_pos_temp = int(robot.next_pos_move[0]/MAP.GAP), int(robot.next_pos_move[1]/MAP.GAP)
            # robot.distance_to_end = BFS(robot.grid_in_vison_robot, END_pos)
            robot.distance_to_end=  Partial_BFS(robot.grid_in_vison_robot, next_pos_temp, END_pos)


            # TEMP_distance_to_end =  Astar(robot.grid_in_vison_robot, next_pos_temp, END_pos)
            # for row in robot.grid_in_vison_robot:
            #     for spot in row:
            #         if TEMP_distance_to_end[spot] != 0:
            #             robot.distance_to_end[spot] = TEMP_distance_to_end[spot]


            print("Time Astar: ", pygame.time.get_ticks() - delta_time)
            last_time = pygame.time.get_ticks() 

        # if Utils.distance_real((robot.x, robot.y), robot.target) < 100:
        robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot()
        # print("robot.next_pos_move: ", robot.next_pos_move)
        # next_pos_temp = robot.next_pos_move
        # ==================================================================================================
        if planed == True:
            print("Start moving")
            robot_move_thread.start()
            print("Start dynamic obstacle")
            dynamic_obstacle_move_thread.start()
            print("Start sensor")
            sensor_detect_thread.start()
            print("Start display")
            display_thread.start()
            planed = False

        # ==================================================================================================

    robot.u = 0
    # while Utils.distance_real((robot.x, robot.y), fixEndPoint) > 1:
    #     robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot(
    #         robot.grid_in_vison_robot, fixEndPoint, MAP.WIN, robot.distance_to_end, robot.KDTree)
    #     # next_pos_temp = robot.next_pos_move

    run = False


def robot_move():
    global robot, MAP, run
    robot.draw(MAP.WIN)
    while run:
        while Utils.distance_real((robot.x, robot.y), robot.target) > 10 and Utils.distance_real((robot.x, robot.y), MAP.end.get_real_pos(MAP.GAP)) > 10 and run:
            next_pos_temp = None
            # MAP.draw_not_update()
            if robot.next_pos_move != next_pos_temp:
                next_pos_temp = robot.next_pos_move
                robot.move_to()
            # else:
            #     robot.move_back(MAP)


def dynamic_obstacle_move():
    global MAP, dynamicObstacles, run
    last_time = pygame.time.get_ticks()
    while run:
        # MAP.draw_not_update()
        dt = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        for obstacle in dynamicObstacles:
            obstacle.move(MAP.grid, dt)
        # for obstacle in dynamicObstacles:
        #     obstacle.draw(MAP.WIN)


def sensor_detect():
    global robot, MAP, dynamicObstacles, sensor, run
    while run:
        robot.dynamic_obstacles_replan = list(
            sensor.detect_replan(robot, dynamicObstacles))
        # print(robot.theta)
        # if len(robot.dynamic_obstacles_replan) > 0:
        #     print("Replan", len(robot.dynamic_obstacles_replan))
        pygame.time.delay(500)


def display():
    global robot, MAP, dynamicObstacles, run
    while run:
        MAP.draw_not_update()
        pygame.draw.circle(MAP.WIN, Constant.RED, (robot.x, robot.y), 100, 2)
        for obstacle in dynamicObstacles:
            obstacle.draw(MAP.WIN)
        for location in range(len(robot.pathRb) - 1):
            pygame.draw.line(
                MAP.WIN, Constant.BLUE, robot.pathRb[location], robot.pathRb[location + 1], 2)
        pygame.draw.circle(MAP.WIN, Constant.RED, robot.target, 5)
        robot.rotated = pygame.transform.rotozoom(
            robot.img, math.degrees(-robot.theta), 1)
        robot.rect = robot.rotated.get_rect(center=(robot.x, robot.y))
        robot.draw(MAP.WIN)
        robot.trail((robot.x, robot.y), MAP.WIN, Constant.GREEN)
        pygame.display.update()

# Initialize pygame


def main():
    global MAP, run, planed, dynamicObstacles

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
    while run:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("Quit")
                run = False

            # load map
            if MASK == False:
                MAP.draw()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    print("Load map")
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map1", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles(
                        dynamicFileName)
                    MAP.draw()
                if event.key == pygame.K_2:
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map2", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles(
                        dynamicFileName)
                    MAP.draw()
                if event.key == pygame.K_3:
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map3", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles(
                        dynamicFileName)
                    MAP.draw()
                if event.key == pygame.K_4:
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map4", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles(
                        dynamicFileName)
                    MAP.draw()
                if event.key == pygame.K_5:
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map5", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles( 
                        dynamicFileName)
                    MAP.draw()
                if event.key == pygame.K_6:
                    MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(
                        fileName, MAP.grid, "map6", MAP.ROWS)
                    dynamicObstacles = Utils.load_dynamic_obstacles(
                        dynamicFileName)
                    MAP.draw()

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
                    global robot, sensor
                    robot = Robot(MAP.start.get_real_pos(
                        MAP.GAP), "C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/Image.png", 16)
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
    main()
