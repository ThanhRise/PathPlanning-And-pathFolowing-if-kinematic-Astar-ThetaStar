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

lock_event = threading.Event()

# 10/11/2023
# thay doi target theo goc cua ( neu goc cua gap thi target se gan hon)
# voronoi tren khong gian khong biet truoc



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

    init_time = pygame.time.get_ticks()
    init_satety_score = 0
    init_index = 0


    fixEndPoint = MAP.end.get_real_pos(MAP.GAP)
    robot.distance_to_end = BFS(MAP.grid, MAP.end)
    # robot.distance_to_end_one_way = BFSv2(MAP.grid, MAP.start, "10")
    robot.distance_to_start = BFS(MAP.grid, MAP.start)
    obstacle = [spot.get_real_pos(MAP.GAP)
                for row in MAP.grid for spot in row if spot.is_barrier()]
    robot.KDTree = KDTree(obstacle)
    robot.index_path = 0

    # # robot.pathRb = find_path(robot=robot, MAP=MAP, start=MAP.start)
    # robot.pathRb = find_path_with_kinematic(robot=robot, MAP=MAP, start_pos_current=MAP.start)

    
    regions, robot.segments = partition_environment(MAP)
    robot.segments = [(p1, p2) for p1, p2 in robot.segments if Utils.line_of_sight((int(p1[0]/MAP.GAP), int(p1[1]/MAP.GAP)), (int(p2[0]/MAP.GAP), int(p2[1]/MAP.GAP)), MAP.grid)]

    robot.pathVoronoi = Astar_voronoi_kinematic(robot, MAP, MAP.start, robot.segments)
    robot.pathRb = robot.pathVoronoi
    
    planed = True
    print("robot.pathRb : OKKKKKKKKKKKKKKKKKKKKKKKKKKKK")
    while Utils.distance_real((robot.x, robot.y), fixEndPoint) > 5:
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
        while Utils.distance_real((robot.x, robot.y), robot.target) > 30:

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

            # if Utils.distance_real(robot.next_pos_move, next_pos_temp) < 20:
            robot.next_pos_move, robot.next_angle_move, __dict__ = robot.find_next_spot(MAP)
            # next_pos_temp = robot.next_pos_move



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
                robot.next_pos_move, robot.next_angle_move, _ = robot.find_next_spot(MAP) 
                last_time_back = pygame.time.get_ticks()
                time_backwards = robot.time_step

            if len(robot.dynamic_obstacles_replan) > 0 and (robot.time_replan == 0 or pygame.time.get_ticks() - robot.time_replan > 4000):
                # =============================
                # other thread wait for this thread
                # lock_event.set()
                # =============================
                # robot.u = 3
                # robot.next_angle_move = 0
                if robot.backwards == True:
                    if robot.theta > 0 and robot.theta < math.pi:
                        robot.theta = robot.theta - math.pi
                    elif robot.theta < 0 and robot.theta > -math.pi:
                        robot.theta = robot.theta + math.pi
                    elif robot.theta > math.pi:
                        robot.theta = robot.theta - math.pi
                    elif robot.theta < -math.pi:
                        robot.theta = robot.theta + math.pi
                    robot.backwards = False
                robot.pathRb, robot.KDTree, robot.time_replan = replanV2(robot=robot, list_obstacles=robot.dynamic_obstacles_replan, MAP=MAP)
                # print('robot.pathRb', robot.pathRb)
                print("Replan", robot.time_replan)
                robot.replaned = True
                robot.index_path = 0

                # =============================
                # other thread continue
                # ==================================================================================================
                break
            
        init_satety_score += robot.KDTree.query((robot.x, robot.y))[0]
        init_index += 1


    print('time move', pygame.time.get_ticks() - init_time)
    
    # print len path
    len_path = 0
    for i in range(len(robot.trail_set) - 1):
        len_path += Utils.distance_real(robot.trail_set[i], robot.trail_set[i + 1])
    print("len_path", len_path)

    # print safety score
    safety_score = init_satety_score / init_index
    print("safety_score", safety_score)

    # print number of collision
    print("num_collision", robot.num_collision)

    # write log info path
    Utils.log_info_path(Constant.INFO_PATH_FILE, MAP.MAP_NAME, len_path, pygame.time.get_ticks() - init_time, safety_score, robot.num_collision)

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
        # lock_event.wait()

        # MAP.draw_not_update()
        dt = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()
        for obstacle in dynamicObstacles:
            obstacle.move(MAP.grid, dt)
        # for obstacle in dynamicObstacles:
        #     obstacle.draw(MAP.WIN)

        # lock_event.clear()



def sensor_detect():
    global robot, MAP, dynamicObstacles, sensor, run
    last_time = pygame.time.get_ticks()
    last_time2 = pygame.time.get_ticks()
    while run:
        if pygame.time.get_ticks() - last_time > 300:
            for row in range(MAP.ROWS):
                for col in range(MAP.ROWS):
                    if MAP.grid[row][col].is_dynamic_obs():
                        MAP.grid[row][col].reset()
            robot.dynamic_obstacles_replan = list(
                sensor.detect_replan(robot, dynamicObstacles))
            last_time = pygame.time.get_ticks()


        if len(robot.dynamic_obstacles_replan) > 0:
            for obs in robot.dynamic_obstacles_replan:
                x, y, d = obs.x, obs.y, obs.d
                d = d // 2
                temp_spos = MAP.grid[int(x / MAP.GAP)][int(y / MAP.GAP)]
                if not temp_spos.is_barrier() and not temp_spos.is_dynamic_obs():
                    temp_spos.make_dynamic_obs()
                temp_spos = MAP.grid[int((x + d) / MAP.GAP)][int((y + d) / MAP.GAP)]
                if not temp_spos.is_barrier() and not temp_spos.is_dynamic_obs():
                    temp_spos.make_dynamic_obs()
                temp_spos = MAP.grid[int((x - d) / MAP.GAP)][int((y + d) / MAP.GAP)]
                if not temp_spos.is_barrier() and not temp_spos.is_dynamic_obs():
                    temp_spos.make_dynamic_obs()
                temp_spos = MAP.grid[int((x + d) / MAP.GAP)][int((y - d) / MAP.GAP)]
                if not temp_spos.is_barrier() and not temp_spos.is_dynamic_obs():
                    temp_spos.make_dynamic_obs()
                temp_spos = MAP.grid[int((x - d) / MAP.GAP)][int((y - d) / MAP.GAP)]
                if not temp_spos.is_barrier() and not temp_spos.is_dynamic_obs():
                    temp_spos.make_dynamic_obs()
                
        if pygame.time.get_ticks() - last_time2 > 1000:
            # write log collision file
            for obs in dynamicObstacles:
                if Utils.distance_real((robot.x, robot.y), (obs.x, obs.y)) < robot.width / 2 + obs.d:
                    print("Collision")
                    robot.num_collision += 1
                    # pygame.image.save(MAP.WIN, f'C:/Users/admin/LearningIT/20222/Project2/MRPP/currentProject/collision/' + str(pygame.time.get_ticks()) + '.png')
                    # Utils.log_collision(Constant.COLLISIONFILE, pygame.time.get_ticks(), MAP.MAP_NAME, robot, dynamicObstacles)

            last_time2 = pygame.time.get_ticks()


def display():
    global robot, MAP, dynamicObstacles, run
    while run:
        MAP.draw_not_update()   
        # for segment in robot.segments:    
        #     p1, p2 = segment
        #     pygame.draw.line(MAP.WIN, Constant.YELLOW, p1, p2, 3)
        
        # for i in range(len(robot.pathVoronoi) - 1):
        #     pygame.draw.line(
        #         MAP.WIN, Constant.RED, robot.pathVoronoi[i], robot.pathVoronoi[i + 1], 2)


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
            list_key = [pygame.K_1, pygame.K_2, pygame.K_3,
            pygame.K_4, pygame.K_5, pygame.K_6,
            pygame.K_7, pygame.K_8, pygame.K_9]
            if event.type == pygame.KEYDOWN:
                for i in range(len(list_key)):
                    if event.key == list_key[i]:
                        print("Load map")
                        MAP.MAP_NAME = "map" + str(i + 1)
                        MAP.grid, MAP.start, MAP.end, MAP.obstacles = Utils.load_map(fileName, MAP.grid, MAP.MAP_NAME, MAP.ROWS)
                        dynamicObstacles = Utils.load_dynamic_obstacles(dynamicFileName)
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

