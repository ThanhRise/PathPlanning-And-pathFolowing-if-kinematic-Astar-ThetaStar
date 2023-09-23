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

lock = threading.Lock() 
lock_event = threading.Event()


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
    robot.distance_to_end = BFS(MAP.grid, MAP.end)
    # robot.distance_to_end_one_way = BFSv2(MAP.grid, MAP.start, "10")
    robot.distance_to_start = BFS(MAP.grid, MAP.start)
    obstacle = [spot.get_real_pos(MAP.GAP)
                for row in MAP.grid for spot in row if spot.is_barrier()]
    robot.KDTree = KDTree(obstacle)
    robot.index_path = 0

    # robot.pathRb = find_path(robot=robot, MAP=MAP, start=MAP.start)
    robot.pathRb = find_path_with_kinematic(robot=robot, MAP=MAP, start_pos_current=MAP.start)

    # robot.pathRb = find_path_one_way(robot=robot, MAP=MAP, start=MAP.start)
    # while Utils.distance_real((robot.x, robot.y), robot.pathRb[robot.index_path]) < 80:
    #     robot.index_path += 1
    
    planed = True
    print("robot.pathRb : OKKKKKKKKKKKKKKKKKKKKKKKKKKKK")
    while Utils.distance_real((robot.x, robot.y), fixEndPoint) > 15:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                print("Quit")
                pygame.quit()
                return
        if robot.index_path >= len(robot.pathRb):
            break
        robot.target = robot.pathRb[robot.index_path]  # target is float

        robot.index_path += 1
        while Utils.distance_real((robot.x, robot.y), robot.target) > 60:
            # if Utils.distance_real(robot.next_pos_move, next_pos_temp) < 20:
            robot.next_pos_move, robot.next_angle_move, __dict__ = robot.find_next_spot(
                MAP.grid, robot.target, MAP.WIN, robot.distance_to_end, robot.KDTree)
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

            lock_event.set()

            if len(robot.dynamic_obstacles_replan) > 0 and (robot.time_replan == 0 or pygame.time.get_ticks() - robot.time_replan > 8000):
                # =============================
                # other thread wait for this thread

                # =============================
                robot.u = 3
                robot.next_angle_move = 0
                robot.pathRb, robot.KDTree, robot.time_replan = replan(robot=robot, list_obstacles=robot.dynamic_obstacles_replan, MAP=MAP)
                # print('robot.pathRb', robot.pathRb)
                print("Replan", robot.time_replan)
                robot.replaned = True
                robot.index_path = 0

                # =============================
                # other thread continue
                # ==================================================================================================
                break

    while Utils.distance_real((robot.x, robot.y), fixEndPoint) > 1:
        robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot(
            MAP.grid, fixEndPoint, MAP.WIN, robot.distance_to_end, robot.KDTree)
        # next_pos_temp = robot.next_pos_move

    run = False


def robot_move():
    global robot, MAP, run
    robot.draw(MAP.WIN)
    while run:
        while Utils.distance_real((robot.x, robot.y), robot.target) > 15 and Utils.distance_real((robot.x, robot.y), MAP.end.get_real_pos(MAP.GAP)) > 1 and run:
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
        lock_event.wait()
        # MAP.draw_not_update()
        dt = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        for obstacle in dynamicObstacles:
            obstacle.move(MAP.grid, dt)
        # for obstacle in dynamicObstacles:
        #     obstacle.draw(MAP.WIN)
        lock_event.clear()



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
