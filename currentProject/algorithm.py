import math
from spot import Spot
from constant import Constant
import pygame
import numpy as np
from utils import Utils
from scipy.spatial import KDTree
import scipy.interpolate as interpolate
from environment import Environment
# from robot import Robot
from queue import PriorityQueue
from dubinsPath import dubins_path_length
from dubinsPath import Waypoint
from scipy.spatial import Voronoi, voronoi_plot_2d
from dijkstar import Graph, find_path
# defaultdict
from collections import defaultdict
import copy
from sklearn import svm
from matplotlib import pyplot as plt


def BFS(grid, end_pos):
    # update the neighbor_distance of each node
    end_pos = grid[end_pos[0]][end_pos[1]]
    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)
    distance = {spot: float(1000000) for row in grid for spot in row}
    
    distance[end_pos] = 0
    queue = []
    queue.append(end_pos)
    while queue:
        current = queue.pop(0)
        for neighbor, dist in current.neighbors_distance:
            if distance[current] + dist < distance[neighbor]:
                distance[neighbor] = distance[current] + dist
                queue.append(neighbor)
    return distance


def Partial_BFS(grid, start, end):
    # update the neighbor_distance of each node
    end_pos = grid[end[0]][end[1]]
    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)
    distance = {spot: float(1000000) for row in grid for spot in row}
    distance[end_pos] = 0
    # queue = PriorityQueue()
    # queue.put((0, end_pos))
    queue = []
    queue.append(end_pos)
    while queue:
        current = queue.pop(0)
        if current == start:
            break
        for neighbor, dist in current.neighbors_distance:
            if distance[current] + dist < distance[neighbor]:
                distance[neighbor] = distance[current] + dist
                # queue.put((distance[neighbor], neighbor))
                queue.append(neighbor)
    return distance

def Astar(grid, start, end):
    end_pos = grid[end[0]][end[1]]
    start_pos = grid[start[0]][start[1]]
    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start_pos))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start_pos] = 0
    f_score = {spot: float(0) for row in grid for spot in row}
    f_score[start_pos] = Utils.distance_spot(start_pos.get_pos(), end_pos.get_pos())

    open_set_hash = {start_pos}

    while not open_set.empty():

        current = open_set.get()[2]
        open_set_hash.remove(current)


        if current == end_pos:
            return f_score

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + Utils.distance_spot(neighbor.get_pos(), end_pos.get_pos())
                f_score[came_from[current]] = f_score[neighbor] + Utils.distance_spot(came_from[current].get_pos(), current.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)

    return f_score

def BFSv2(grid, start: Spot, angleStart):
    distance = {spot: float(10000) for row in grid for spot in row}
    distance[start] = 0
    queue = []    
    # open_set = set()
    # open_set.add((start, angleStart))
    queue.append((start, angleStart))
    while queue:
        current, angleCurrent = queue.pop(0)
        # open_set.remove(current)
        current.update_neighbors_with_angle(grid, angleCurrent)
        for neighbor, dist , angle in current.neighbors_distance_angleRobot:
            if distance[current] + dist < distance[neighbor]:
                distance[neighbor] = distance[current] + dist
                queue.append((neighbor, angle))
                # open_set.add((neighbor, angle))
    return distance


def find_path(robot, MAP: Environment, start: Spot):

    current_start = start.get_real_pos(
        MAP.GAP)[0], start.get_real_pos(MAP.GAP)[1]
    current_end = MAP.end.get_real_pos(
        MAP.GAP)[0], MAP.end.get_real_pos(MAP.GAP)[1]
    path_to_end = []
    path_to_end.append(current_start)
    path_to_start = []
    path_to_start.append(current_end)
    IsPlaning = True

    while IsPlaning:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        # print("current_start", current_start)
        dis_obs_current_to_end, _ = robot.KDTree.query([current_start])
        # pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_start[0]), int(current_start[1])), int(dis_obs_current_to_end), 3)
        for i, point in enumerate(path_to_start):
            dis_current_end = Utils.distance_real(current_start, point)
            if dis_obs_current_to_end > dis_current_end:
                # join path_to_start to path_to_end
                path_to_end = path_to_end + path_to_start[i::-1]
                # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_start[0]), int(current_start[1])), (int(point[0]), int(point[1])), 3)
                # pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(0, 360, 5):
                next_point = int(current_start[0] + dis_obs_current_to_end * math.cos(math.radians(
                    i))), int(current_start[1] + dis_obs_current_to_end * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > Constant.WIDTH or next_point[1] < 0 or next_point[1] > Constant.WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = robot.KDTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(next_point[0] / MAP.GAP), int(next_point[1] / MAP.GAP)
                score = 150*(1.0 / dis_obs_next) + \
                    robot.distance_to_end[MAP.grid[row][col]]
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_start[0]), int(current_start[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            # pygame.display.update()
            current_start = next_point_max
            path_to_end.append(current_start)

        if not IsPlaning:
            break
        dis_obs_current_to_start, _ = robot.KDTree.query([current_end])
        # pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_end[0]), int(current_end[1])), int(dis_obs_current_to_start), 3)
        for i, point in enumerate(path_to_end):
            dis_current_start = Utils.distance_real(current_end, current_start)
            if dis_obs_current_to_start > dis_current_start:
                # print("path_to_end", path_to_end)
                path_to_end = path_to_end[:i-1:] + path_to_start[::-1]
                # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_end[0]), int(current_end[1])), (int(current_start[0]), int(current_start[1])), 3)
                # pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(0, 360, 5):
                next_point = int(current_end[0] + dis_obs_current_to_start * math.cos(math.radians(
                    i))), int(current_end[1] + dis_obs_current_to_start * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > Constant.WIDTH or next_point[1] < 0 or next_point[1] > Constant.WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = robot.KDTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(
                    next_point[0] / MAP.GAP), int(next_point[1] / MAP.GAP)
                score = 150*(1.0 / dis_obs_next) + \
                    robot.distance_to_start[MAP.grid[row][col]]
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_end[0]), int(current_end[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            # pygame.display.update()
            current_end = next_point_max
            path_to_start.append(current_end)
    if len(path_to_end) < 3:
        return path_to_end
    point_control = np.array(path_to_end)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 3*Utils.distance_spot((start.row, start.col), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    # for i in range(len(B_spline) - 1):
        # pygame.draw.line(MAP.WIN, Constant.RED, (int(B_spline[i][0]), int(B_spline[i][1])), (int(B_spline[i+1][0]), int(B_spline[i+1][1])), 3)
    #     pygame.display.update()
    print('path planning done')
    return B_spline

def find_path_one_way(robot, MAP: Environment, start: Spot):
    current_start = start.get_real_pos(MAP.GAP)[0], start.get_real_pos(MAP.GAP)[1]
    current_end = MAP.end.get_real_pos(MAP.GAP)[0], MAP.end.get_real_pos(MAP.GAP)[1]
    path_to_end = []
    path_to_end.append(current_start)

    while True:
        MAX_POINT = 0
        next_point_max = None
        dis_obs_current_to_start, _ = robot.KDTree.query([current_start])
        if dis_obs_current_to_start > Utils.distance_real(current_start, current_end):
            path_to_end.append(current_end)
            break
        for i in range(0, 360, 5):
            next_point = int(current_start[0] + dis_obs_current_to_start * math.cos(math.radians(
                i))), int(current_start[1] + dis_obs_current_to_start * math.sin(math.radians(i)))
            if next_point[0] < 0 or next_point[0] > Constant.WIDTH or next_point[1] < 0 or next_point[1] > Constant.WIDTH:
                continue
            dis_obs_next, _ = robot.KDTree.query([next_point])
            if dis_obs_next == 0:
                continue
            row, col = int(next_point[0] / MAP.GAP), int(next_point[1] / MAP.GAP)
            score = dis_obs_next + robot.distance_to_end_one_way[MAP.grid[row][col]]
            if score > MAX_POINT:
                MAX_POINT = score
                next_point_max = next_point
        # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_start[0]), int(current_start[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
        # pygame.display.update()
        current_start = next_point_max
        # print("current_start: ", current_start)
        # pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_start[0]), int(current_start[1])), int(dis_obs_current_to_start), 10)
        # pygame.display.update()
        path_to_end.append(current_start)

        if Utils.distance_real(current_start, current_end) < 5:
            break
    if len(path_to_end) < 3:
        return path_to_end
    point_control = np.array(path_to_end)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 2*Utils.distance_spot((start.row, start.col), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    return B_spline

def replan(robot, list_obstacles, MAP):
    step_horizontal = MAP.GAP / 2
    print("replan")
    restore_grid = []
    for obs in list_obstacles:
        if Utils.distance_real((obs.x, obs.y), (robot.x, robot.y)) < 150:
            # print("angle", obs.theta, "atan2", math.atan2(obs.y - robot.y, obs.x - robot.x), "angle - atan2", obs.theta - math.atan2(obs.y - robot.y, obs.x - robot.x))
            x_next = obs.x + 5 * obs.velocity * math.cos(obs.theta)
            y_next = obs.y + 5 * obs.velocity * math.sin(obs.theta)
            if x_next <= 8:
                x_next = 9
            if x_next >= 792:
                x_next = 790
            if y_next <= 8:
                y_next = 9
            if y_next >= 792:
                y_next = 790

            x_current = int(obs.x)
            y_current = int(obs.y)
            
            x_next = int(x_next)
            y_next = int(y_next)

            while Utils.distance_real((x_current, y_current), (x_next, y_next)) > 15 and x_current < 800 and x_current > 0 and y_current < 800 and y_current > 0:
         
                if x_current + 8 >= 800 or x_current - 8 <= 0 or y_current + 8 >= 800 or y_current - 8 <= 0:
                    break
                
                if Utils.distance_real((x_current, y_current), (robot.x, robot.y)) < 28 and math.fabs(obs.theta - robot.theta) > math.pi / 2*0.8 and math.fabs(obs.theta - robot.theta) < math.pi / 2*1.2:
                    continue
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()

                x_current += 16 * math.cos(obs.theta)
                y_current += 16 * math.sin(obs.theta)
    # draw(win, MAP.grid, ROWS, WIDTH)



    obstacle = [spot.get_real_pos(MAP.GAP) for row in MAP.grid for spot in row if (spot.is_barrier() or spot.is_dynamic_obs())]
    start_pos_current = MAP.grid[int(robot.x // MAP.GAP)][int(robot.y // MAP.GAP)]
    MAP.start = start_pos_current
    robot.distance_to_end = BFS(MAP.grid, MAP.end)
    robot.KDTree = KDTree(obstacle)

    print("KDTREE OK")
    # robot.pathRb = find_path(robot, MAP, start_pos_current)

    index = 0
    end_pos_current = None
    for i in range(robot.index_path, len(robot.pathRb)):
        if Utils.distance_real((robot.pathRb[i][0], robot.pathRb[i][1]), (robot.x, robot.y)) > 200:
            end_pos_current = MAP.grid[int(robot.pathRb[i][0] // MAP.GAP)][int(robot.pathRb[i][1] // MAP.GAP)]
            index = i
            break
    path = robot.pathRb[index:] 

    if end_pos_current is not None:
        path_replan = find_path_with_kinematic(robot, MAP, start_pos_current, end_pos_current)
        robot.pathRb = path_replan + path
    else:
        robot.pathRb = find_path_with_kinematic(robot, MAP, start_pos_current, MAP.end)
    # robot.pathRb = find_path_two_KDtree(robot, MAP, start_pos_current, dynamicKDTree)

    # for spot in restore_grid:
    #     spot.reset_to_prev_color()
    return robot.pathRb, robot.KDTree, pygame.time.get_ticks()


def find_path_with_kinematic(robot, MAP, start_pos_current, end_pos_current):
    PATH = []
    one_degree = math.pi / 180
    angle = []
    angle.append(0)
    for i in range(1, 62, 10):
        angle.append(i * one_degree)
        angle.append(-i * one_degree)
    x_current, y_current = start_pos_current.get_real_pos(MAP.GAP)
    theta_current = robot.theta
    # print("theta_current: ", theta_current)
    PATH.append((x_current, y_current))
    while Utils.distance_real((x_current, y_current), end_pos_current.get_real_pos(MAP.GAP)) > 10:
        min_heuristic = 10000
        maxStep = 16
        step = 4
        x_next = 0
        y_next = 0
        theta_next = 0
        if end_pos_current == MAP.end:
            for i in range(len(angle)):
                    x_temp = x_current + (step*2 + 8) * math.cos(theta_current + angle[i])
                    y_temp = y_current + (step*2 + 8) * math.sin(theta_current + angle[i])
                    temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        distance_to_end = robot.distance_to_end[temp_spot]
                        distance_to_obs, _ = robot.KDTree.query([(int(x_temp), int(y_temp))])
                        heuristic = distance_to_end + 50 * (1.0 / distance_to_obs)
                        if heuristic < min_heuristic:
                            min_heuristic = heuristic
                            x_next = x_temp
                            y_next = y_temp
                            theta_next = theta_current + angle[i] 
        else:
            for i in range(len(angle)):
                    x_temp = x_current + (step*2 + 8) * math.cos(theta_current + angle[i])
                    y_temp = y_current + (step*2 + 8) * math.sin(theta_current + angle[i])
                    temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        distance_to_obs, _ = robot.KDTree.query([(int(x_temp), int(y_temp))])
                        heuristic = Utils.distance_real((x_temp, y_temp), end_pos_current.get_real_pos(MAP.GAP)) + 1000 * (1.0 / distance_to_obs)
                        if heuristic < min_heuristic:
                            min_heuristic = heuristic
                            x_next = x_temp
                            y_next = y_temp
                            theta_next = theta_current + angle[i] 

        if x_next == 0 and y_next == 0:
            print("Can't find path")
            break

        # pygame.draw.line(MAP.WIN, (255, 0, 0), (x_current, y_current), (x_next, y_next), 2)
        # pygame.display.update()
        # pygame.time.delay(100)

        x_current = x_next
        y_current = y_next
        PATH.append((x_current, y_current))
        theta_current = theta_next
        # print("theta_current: ", theta_current) 


    # -----------------------------------------------------------------------#
    # Add Dubins path to PATH                                                #
    # -----------------------------------------------------------------------#

    # while Utils.distance_real((x_current, y_current), MAP.end.get_real_pos(MAP.GAP)) > 5:
    #     min_heuristic = 10000
    #     x_next = 0
    #     y_next = 0
    #     theta_next = 0
    #     dubins_path_final = []
    #     for i in range(len(angle)):
    #         # for step in range(1, maxStep, 5):
    #         x_temp = x_current + (16) * math.cos(theta_current + angle[i])
    #         y_temp = y_current + (16) * math.sin(theta_current + angle[i])
    #         # pygame.draw.line(MAP.WIN, (255, 0, 0), (x_current, y_current), (x_temp, y_temp), 2)
    #         # pygame.display.update()

    #         temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
    #         if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
    #             temp_angle = (theta_current + angle[i]) * 180 / math.pi
    #             if temp_angle < 0:
    #                 temp_angle += 360
    #             if temp_angle > 360:
    #                 temp_angle -= 360
    #             temp_wpt = Waypoint(x_temp, y_temp, temp_angle)
    #             end_wpt = Waypoint(MAP.end.get_real_pos(MAP.GAP)[0], MAP.end.get_real_pos(MAP.GAP)[1], angleEnd)
    #             length_path_dubins, dubins_path = dubins_path_length(temp_wpt, end_wpt)
                
    #             # for k in range(len(dubins_path) - 1):
    #             #     pygame.draw.line(MAP.WIN, (255, 0, 0), (dubins_path[k][0], dubins_path[k][1]), (dubins_path[k+1][0], dubins_path[k+1][1]), 2)
    #             #     pygame.display.update()

    #             heuristic = length_path_dubins
    #             if heuristic < min_heuristic:
    #                 min_heuristic = heuristic
    #                 x_next = x_temp
    #                 y_next = y_temp
    #                 theta_next = theta_current + angle[i]
    #                 # dubins_path_final drop colunm 2
    #                 dubins_path_final = dubins_path[:, :2]

        # if x_next == 0 and y_next == 0:
        #     print("Can't find path")
        #     break
        # x_current = x_next
        # y_current = y_next
        # theta_current = theta_next
        # # add dubins path to PATH
        # # PATH = PATH + dubins_path_final.tolist()
        # break

    # pygame.time.delay(100000)



    PATH.append(end_pos_current.get_real_pos(MAP.GAP))
    if len(PATH) <= 3:
        return PATH
    # use Bspline to smooth the path
    PATH = np.array(PATH)
    x = PATH[:, 0]
    y = PATH[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 3*Utils.distance_spot((start_pos_current.row, start_pos_current.col), (end_pos_current.row, end_pos_current.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)
    B_spline = list(zip(x_bspline, y_bspline))
    return B_spline    

    
def find_path_two_KDtree(robot, MAP: Environment, start: Spot, DynamicKDtree):


    current_start = start.get_real_pos(MAP.GAP)[0], start.get_real_pos(MAP.GAP)[1]
    current_end = MAP.end.get_real_pos( MAP.GAP)[0], MAP.end.get_real_pos(MAP.GAP)[1]
    path_to_end = []
    path_to_end.append(current_start)
    path_to_start = []
    path_to_start.append(current_end)
    IsPlaning = True

    while IsPlaning:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        # print("current_start", current_start)
        dis_obs_current_to_end, _ = robot.KDTree.query([current_start])
        dis_obs_current_to_end_2, _ = DynamicKDtree.query([current_start])
        if dis_obs_current_to_end_2 < dis_obs_current_to_end:
            dis_obs_current_to_end = dis_obs_current_to_end_2
        pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_start[0]), int(current_start[1])), int(dis_obs_current_to_end), 3)
        for i, point in enumerate(path_to_start):
            dis_current_end = Utils.distance_real(current_start, point)
            if dis_obs_current_to_end > dis_current_end:
                # join path_to_start to path_to_end
                path_to_end = path_to_end + path_to_start[i::-1]
                # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_start[0]), int(current_start[1])), (int(point[0]), int(point[1])), 3)
                # pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(0, 360, 5):
                next_point = int(current_start[0] + dis_obs_current_to_end * math.cos(math.radians(
                    i))), int(current_start[1] + dis_obs_current_to_end * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > Constant.WIDTH or next_point[1] < 0 or next_point[1] > Constant.WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = robot.KDTree.query([next_point])
                dis_obs_next_2, _ = DynamicKDtree.query([next_point])
                if dis_obs_next == 0 or dis_obs_next_2 == 0:
                    continue
                row, col = int(next_point[0] / MAP.GAP), int(next_point[1] / MAP.GAP)
                # if MAP.grid[row][col].is_dynamic_obs:
                #     continue
                score = 150*(1.0 / dis_obs_next) + robot.distance_to_end[MAP.grid[row][col]] + 150*(1.0 / dis_obs_next_2)
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_start[0]), int(current_start[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            # pygame.display.update()
            current_start = next_point_max
            path_to_end.append(current_start)

        if not IsPlaning:
            break
        dis_obs_current_to_start, _ = robot.KDTree.query([current_end])
        dis_obs_current_to_start_2, _ = DynamicKDtree.query([current_end])
        if dis_obs_current_to_start_2 < dis_obs_current_to_start:
            dis_obs_current_to_start = dis_obs_current_to_start_2
        pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_end[0]), int(current_end[1])), int(dis_obs_current_to_start), 3)
        for i, point in enumerate(path_to_end):
            dis_current_start = Utils.distance_real(current_end, current_start)
            if dis_obs_current_to_start > dis_current_start:
                # print("path_to_end", path_to_end)
                path_to_end = path_to_end[:i-1:] + path_to_start[::-1]
                # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_end[0]), int(current_end[1])), (int(current_start[0]), int(current_start[1])), 3)
                # pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(0, 360, 5):
                next_point = int(current_end[0] + dis_obs_current_to_start * math.cos(math.radians(
                    i))), int(current_end[1] + dis_obs_current_to_start * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > Constant.WIDTH or next_point[1] < 0 or next_point[1] > Constant.WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = robot.KDTree.query([next_point])
                dis_obs_next_2, _ = DynamicKDtree.query([next_point])
                if dis_obs_next == 0 or dis_obs_next_2 == 0:
                    continue
                row, col = int(
                    next_point[0] / MAP.GAP), int(next_point[1] / MAP.GAP)
                # if MAP.grid[row][col].is_dynamic_obs:
                #     continue
                score = 150*(1.0 / dis_obs_next) + robot.distance_to_start[MAP.grid[row][col]] + 150*(1.0 / dis_obs_next_2)
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            # pygame.draw.line(MAP.WIN, Constant.GREEN, (int(current_end[0]), int(current_end[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            # pygame.display.update()
            current_end = next_point_max
            path_to_start.append(current_end)
    if len(path_to_end) < 3:
        return path_to_end
    point_control = np.array(path_to_end)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 3*Utils.distance_spot((start.row, start.col), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    # for i in range(len(B_spline) - 1):
        # pygame.draw.line(MAP.WIN, Constant.RED, (int(B_spline[i][0]), int(B_spline[i][1])), (int(B_spline[i+1][0]), int(B_spline[i+1][1])), 3)
    #     pygame.display.update()
    print('path planning done')
    return B_spline


def partition_environment(MAP):
    # Create a list of obstacle points from the grid
    obstacle_points = []
    for row in MAP.grid:
        for spot in row:
            if spot.is_barrier() or spot.is_dynamic_obs():
                obstacle_points.append((spot.get_real_pos(MAP.GAP)[0], spot.get_real_pos(MAP.GAP)[1]))

    # Compute Voronoi diagram
    MAP.voronoi = Voronoi(obstacle_points, incremental=True)
    
    # Create a list of line segments representing the Voronoi diagram
    segments = []
    for i, j in MAP.voronoi.ridge_vertices:
        if i >= 0 and j >= 0:
            p1 = (MAP.voronoi.vertices[i][0], MAP.voronoi.vertices[i][1])
            p2 = (MAP.voronoi.vertices[j][0], MAP.voronoi.vertices[j][1])
            segments.append((p1, p2))
    # print(segments)

    
    # Partition the environment into regions based on the MAP.voronoi diagram
    regions = []
    for i, region in enumerate(MAP.voronoi.regions):
        if -1 not in region:
            polygon = [MAP.voronoi.vertices[i] for i in region]
            regions.append(polygon)

    # Convert point in segments to integer
    for i, segment in enumerate(segments):
        segments[i] = ((int(segment[0][0]), int(segment[0][1])), (int(segment[1][0]), int(segment[1][1])))

    return regions, segments
 

def Astar_voronoi_find_path(robot, map , start_pos_current, segments ):
    
    pair = defaultdict(list)
    for p1, p2 in segments:
        pair[p1].append(p2)
        pair[p2].append(p1)

    # set point
    point_voronoi = set()
    for point1, point2 in segments:
        point_voronoi.add(point1)
        point_voronoi.add(point2)

    # Astar
    start = start_pos_current.get_real_pos(map.GAP)
    x_start = start[0] + 32 * math.cos(robot.theta)
    y_start = start[1] + 32 * math.sin(robot.theta)
    if map.grid[int(x_start // map.GAP)][int(y_start // map.GAP)].is_barrier():
        x_start = x_start - 32 * math.cos(robot.theta)
        y_start = y_start - 32 * math.sin(robot.theta)
    start = (x_start, y_start)
    end = map.end.get_real_pos(map.GAP)
    start_pos = start_pos_current.get_pos()
    end_pos = map.end.get_pos()

    # point nearest start
    start_point = None
    dis = float(1000000)
    for point in point_voronoi:
        if Utils.distance_real(start, point) < dis:
            dis = Utils.distance_real(start, point)
            start_point = point

    # point nearest end
    end_point = None
    dis = float(1000000)
    for point in point_voronoi:
        if Utils.distance_real(end, point) < dis:
            dis = Utils.distance_real(end, point)
            end_point = point

    queue = PriorityQueue()
    queue.put((0, start_point))
    came_from = {}
    g_score = {point: float("inf") for point in point_voronoi}
    g_score[start_point] = 0
    f_score = {point: float(0) for point in point_voronoi}
    f_score[start_point] = Utils.distance_real(start_point, end)

    while not queue.empty():
        current = queue.get()[1]

        if current == end_point:
            break

        for neighbor in pair[current]:
            temp_g_score = g_score[current] + Utils.distance_real(current, neighbor)

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + Utils.distance_real(neighbor, end)
                queue.put((f_score[neighbor], neighbor))

    # find path
    path = []
    # current = end_point
    path.append(current)
    while current != start_point:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path
    
def Astar_voronoi_kinematic(robot, MAP, start_pos_current, segments):

    path = Astar_voronoi_find_path(robot, MAP, start_pos_current, segments)

    end = MAP.end.get_real_pos(MAP.GAP)
    start_pos = start_pos_current.get_pos()
    end_pos = MAP.end.get_pos()

    PATH = []

    # one_degree = math.pi / 180
    # angle = []
    # angle.append(0)
    # for i in range(1, 62, 10):
    #     angle.append(i * one_degree)
    #     angle.append(-i * one_degree)

    # x_current, y_current = robot.x, robot.y
    # x_current = x_current + 16 * math.cos(robot.theta)
    # y_current = y_current + 16 * math.sin(robot.theta)
    # if MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)].is_barrier():
    #     x_current = x_current - 16 * math.cos(robot.theta)
    #     y_current = y_current - 16 * math.sin(robot.theta)

    # theta_current = robot.theta

    # PATH.append((x_current, y_current))
    # point_nearest = path[0]
    # nearest = Utils.distance_real((x_current, y_current), point_nearest)
    # index_nearest = 0

    # while Utils.distance_real((x_current, y_current), point_nearest) > 10:
    #     min_heuristic = 10000
    #     maxStep = 17
    #     x_next = None
    #     y_next = None
    #     theta_next = None

    #     for i in range(len(angle)):
    #         for step in range(1, maxStep, 8):
    #             x_temp = x_current + (step*2 + 8) * math.cos(theta_current + angle[i])
    #             y_temp = y_current + (step*2 + 8) * math.sin(theta_current + angle[i])
    #             if x_temp >= 800 or x_temp <= 0 or y_temp >= 800 or y_temp <= 0:
    #                 break

    #             temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
    #             if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
    #                 heuristic = Utils.distance_real((x_temp, y_temp), point_nearest)
    #                 if heuristic < min_heuristic:
    #                     min_heuristic = heuristic
    #                     x_next = x_temp
    #                     y_next = y_temp
    #                     theta_next = theta_current + angle[i] 

    #     if x_next == None and y_next == None:
    #         print("Can't find path")
    #         break

        
    #     for i in range(int(math.sqrt(len(path)))):
    #         if Utils.distance_real((x_next, y_next), path[i]) < nearest:
    #             point_nearest = path[i]
    #             index_nearest = i
    #             nearest = Utils.distance_real((x_next, y_next), path[i])

    #     x_current = x_next
    #     y_current = y_next
        
    #     PATH.append((x_current, y_current))
    #     theta_current = theta_next

    # path = PATH + path[index_nearest::]

    x_current, y_current = robot.x, robot.y
    # PATH.append((x_current, y_current))
    
    for i in range(len(path) - 1):
        if Utils.distance_real(path[i], path[i+1]) > 100:
            # insert point between path[i] and path[i+1]
            x_temp = (path[i][0] + path[i+1][0]) / 2
            y_temp = (path[i][1] + path[i+1][1]) / 2
            path.insert(i+1, (x_temp, y_temp))

    path.append(end)
    if len(path) <= 3:
        return path
    point_control = np.array(path)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), Utils.distance_spot((start_pos), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))

    return B_spline


def replanV2(robot, list_obstacles, MAP):
    step_horizontal = MAP.GAP / 2
    print("replan")
    restore_grid = []
    for obs in list_obstacles:
            step_horizontal = obs.d * 2 /3
            x_next = obs.x + 5 * obs.velocity * math.cos(obs.theta)
            y_next = obs.y + 5 * obs.velocity * math.sin(obs.theta)
            if x_next <= 8:
                x_next = 9
            if x_next >= 792:
                x_next = 790
            if y_next <= 8:
                y_next = 9
            if y_next >= 792:
                y_next = 790

            x_current = int(obs.x)
            y_current = int(obs.y)

            x_next = int(x_next)
            y_next = int(y_next)

            while Utils.distance_real((x_current, y_current), (x_next, y_next)) > 15 and x_current < 800 and x_current > 0 and y_current < 800 and y_current > 0:

                if x_current + 8 >= 800 or x_current - 8 <= 0 or y_current + 8 >= 800 or y_current - 8 <= 0:
                    break

                if Utils.distance_real((x_current, y_current), (robot.x, robot.y)) < 32 and math.fabs(obs.theta - robot.theta) > math.pi / 2*0.8 and math.fabs(obs.theta - robot.theta) < math.pi / 2*1.2:
                    continue

                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()

                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    # temp_spot.make_dynamic_obs()


                x_current += 16 * math.cos(obs.theta)
                y_current += 16 * math.sin(obs.theta)



    print('len restore_grid: ', len(restore_grid))

    # new_obs = [spot.get_real_pos(MAP.GAP) for spot in restore_grid]
    # MAP.voronoi.add_points(new_obs)

    new_obs = [spot.get_real_pos(MAP.GAP) for row in MAP.grid for spot in row if spot.is_barrier()] + [spot.get_real_pos(MAP.GAP) for spot in restore_grid]
    MAP.voronoi = Voronoi(new_obs, incremental=True)


    # Create a list of line segments representing the Voronoi diagram
    segments = []
    for i, j in MAP.voronoi.ridge_vertices:
        if i >= 0 and j >= 0:
            p1 = (MAP.voronoi.vertices[i][0], MAP.voronoi.vertices[i][1])
            p2 = (MAP.voronoi.vertices[j][0], MAP.voronoi.vertices[j][1])
            segments.append((p1, p2))

    # Convert point in segments to integer
    for i, segment in enumerate(segments):
        segments[i] = ((int(segment[0][0]), int(segment[0][1])), (int(segment[1][0]), int(segment[1][1])))

    robot.segments = segments
    robot.segments = [(p1, p2) for p1, p2 in robot.segments if Utils.line_of_sight((int(p1[0]/MAP.GAP), int(p1[1]/MAP.GAP)), (int(p2[0]/MAP.GAP), int(p2[1]/MAP.GAP)), MAP.grid)]


    start_pos = MAP.grid[int(robot.x // MAP.GAP)][int(robot.y // MAP.GAP)]

    robot.pathRb = Astar_voronoi_kinematic(robot, MAP, start_pos, robot.segments)


    # for spot in restore_grid:
    #     spot.reset_to_prev_color()
    return robot.pathRb, robot.KDTree, pygame.time.get_ticks()

def replan_svm(robot, list_obstacles, MAP):
    step_horizontal = MAP.GAP / 2
    print("replan")
    restore_grid = []
    for obs in list_obstacles:
        if Utils.distance_real((obs.x, obs.y), (robot.x, robot.y)) < 150:
            # print("angle", obs.theta, "atan2", math.atan2(obs.y - robot.y, obs.x - robot.x), "angle - atan2", obs.theta - math.atan2(obs.y - robot.y, obs.x - robot.x))
            x_next = obs.x + 5 * obs.velocity * math.cos(obs.theta)
            y_next = obs.y + 5 * obs.velocity * math.sin(obs.theta)
            if x_next <= 8:
                x_next = 9
            if x_next >= 792:
                x_next = 790
            if y_next <= 8:
                y_next = 9
            if y_next >= 792:
                y_next = 790

            x_current = int(obs.x)
            y_current = int(obs.y)
            
            x_next = int(x_next)
            y_next = int(y_next)

            while Utils.distance_real((x_current, y_current), (x_next, y_next)) > 15 and x_current < 800 and x_current > 0 and y_current < 800 and y_current > 0:
         
                if x_current + 8 >= 800 or x_current - 8 <= 0 or y_current + 8 >= 800 or y_current - 8 <= 0:
                    break
                
                if Utils.distance_real((x_current, y_current), (robot.x, robot.y)) < 28 and math.fabs(obs.theta - robot.theta) > math.pi / 2*0.8 and math.fabs(obs.theta - robot.theta) < math.pi / 2*1.2:
                    continue
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                temp_spot = MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    restore_grid.append(temp_spot)
                    MAP.grid[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()

                x_current += 16 * math.cos(obs.theta)
                y_current += 16 * math.sin(obs.theta)
    
    obstacle = [spot.get_real_pos(MAP.GAP) for row in MAP.grid for spot in row if (spot.is_barrier() or spot.is_dynamic_obs())]
    robot.KDTree = KDTree(obstacle)

    start_pos = MAP.grid[int(robot.x // MAP.GAP)][int(robot.y // MAP.GAP)]
    path_svm, _, _ = SVM_path_planning(start_pos, MAP.end, MAP.grid, MAP.GAP)

    # for spot in restore_grid:
    #     spot.reset_to_prev_color()
    return path_svm, robot.KDTree, pygame.time.get_ticks()

def Astar_find_path_in_grid(start, end, grid):
    def h(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x1 - x2) + abs(y1 - y2)

    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)

    path = []


    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}

    while not open_set.empty():

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            break

        for neighbor, dist in current.neighbors_distance:
            temp_g_score = g_score[current] + dist

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
    if current == end:
        print("found path")
    path.append(current)
    while current != start:
        current = came_from[current]
        path.append(current)
    # path.append(start)
    path.reverse()
    return path

def SVM_path_planning(start, end, grid, gap):

    path = Astar_find_path_in_grid(start, end, grid)
    print('a* path done')
    obs1 = []
    obs2 = []
    search = 4

    for i in range(len(path) - 1):
        x1, y1 = path[i].get_pos()
        x2, y2 = path[i+1].get_pos()
        x_delta = x2 - x1
        y_delta = y2 - y1
        if x_delta == 0:
            if y_delta > 0:
                flag = False
                for j in range(1, search):
                    if grid[x2+ j][y2].is_barrier() or grid[x2+ j][y2].is_dynamic_obs():
                        obs1.append((x2+ j, y2))
                        flag = True
                        break
                if not flag:
                    obs1.append((x2+ search, y2))
                flag = False
                for j in range(1, search):
                    if grid[x2- j][y2].is_barrier() or grid[x2- j][y2].is_dynamic_obs():
                        obs2.append((x2- j, y2))
                        flag = True
                        break
                if not flag:
                    obs2.append((x2- search, y2))
            else:
                flag = False
                for j in range(1, search):
                    if grid[x2+ j][y2].is_barrier() or grid[x2+ j][y2].is_dynamic_obs():
                        obs2.append((x2+ j, y2))
                        flag = True
                        break
                if not flag:
                    obs2.append((x2+ search, y2))
                flag = False
                for j in range(1, search):
                    if grid[x2- j][y2].is_barrier() or grid[x2- j][y2].is_dynamic_obs():
                        obs1.append((x2- j, y2))
                        flag = True
                        break
                if not flag:
                    obs1.append((x2- search, y2))
        elif y_delta == 0:
            if x_delta > 0:
                flag = False
                for j in range(1, search):
                    if grid[x2][y2 - j].is_barrier() or grid[x2][y2 - j].is_dynamic_obs():
                        obs1.append((x2, y2 - j))
                        flag = True
                        break
                if not flag:
                    obs1.append((x2, y2 - search))
                flag = False
                for j in range(1, search):
                    if grid[x2][y2 + j].is_barrier() or grid[x2][y2 + j].is_dynamic_obs():
                        obs2.append((x2, y2 + j))
                        flag = True
                        break
                if not flag:
                    obs2.append((x2, y2 + search))
            else:
                flag = False
                for j in range(1, search):
                    if grid[x2][y2 - j].is_barrier() or grid[x2][y2 - j].is_dynamic_obs():
                        obs2.append((x2, y2 - j))
                        flag = True
                        break
                if not flag:
                    obs2.append((x2, y2 - search))
                flag = False
                for j in range(1, search):
                    if grid[x2][y2 + j].is_barrier() or grid[x2][y2 + j].is_dynamic_obs():
                        obs1.append((x2, y2 + j))
                        flag = True
                        break
                if not flag:
                    obs1.append((x2, y2 + search))
        elif x_delta > 0 and y_delta > 0:
            flag = False
            for j in range(1, search):
                if grid[x2 + j - 1][y2 - j].is_barrier() or grid[x2 + j - 1][y2 - j].is_dynamic_obs():
                    obs1.append((x2 + j - 1, y2 - j))
                    flag = True
                    break
            if not flag:
                obs1.append((x2 + search - 1, y2 - search))
            flag = False
            for j in range(1, search):
                if grid[x2 - j][y2 + j - 1].is_barrier() or grid[x2 - j][y2 + j - 1].is_dynamic_obs():
                    obs2.append((x2 - j, y2 + j - 1))
                    flag = True
                    break
            if not flag:
                obs2.append((x2 - search, y2 + search - 1))
        elif x_delta > 0 and y_delta < 0:
            flag = False
            for j in range(1, search):
                if grid[x2 - j + 1][y2 - j].is_barrier() or grid[x2 - j + 1][y2 - j].is_dynamic_obs():
                    obs1.append((x2 - j + 1, y2 - j))
                    flag = True
                    break
            if not flag:
                obs1.append((x2 - search + 1, y2 - search))
            flag = False
            for j in range(1, search):
                if grid[x2 + j][y2 + j - 1].is_barrier() or grid[x2 + j][y2 + j - 1].is_dynamic_obs():
                    obs2.append((x2 + j, y2 + j - 1))
                    flag = True
                    break
            if not flag:
                obs2.append((x2 + search, y2 + search - 1))
        elif x_delta < 0 and y_delta > 0:
            flag = False
            for j in range(1, search):
                if grid[x2 + j][y2 + j - 1].is_barrier() or grid[x2 + j][y2 + j - 1].is_dynamic_obs():
                    obs1.append((x2 + j, y2 + j - 1))
                    flag = True
                    break
            if not flag:
                obs1.append((x2 + search, y2 + search - 1))
            flag = False
            for j in range(1, search):
                if grid[x2 - j - 1][y2 - j].is_barrier() or grid[x2 - j - 1][y2 - j].is_dynamic_obs():
                    obs2.append((x2 - j - 1, y2 - j))
                    flag = True
                    break
            if not flag:
                obs2.append((x2 - search - 1, y2 - search))
        elif x_delta < 0 and y_delta < 0:
            flag = False
            for j in range(1, search):
                if grid[x2 -j + 1][y2 + j].is_barrier() or grid[x2 -j + 1][y2 + j].is_dynamic_obs():
                    obs1.append((x2 - j + 1, y2 + j))
                    flag = True
                    break
            if not flag:
                obs1.append((x2 - search + 1, y2 + search))
            flag = False
            for j in range(1, search):
                if grid[x2 + j][y2 - j + 1].is_barrier() or grid[x2 + j][y2 - j + 1].is_dynamic_obs():
                    obs2.append((x2 + j, y2 - j + 1))
                    flag = True
                    break
            if not flag:
                obs2.append((x2 + search, y2 - search + 1))


    clf = svm.SVC(kernel='rbf', C=1000)
    # TODO: find optimal prameters

    obs1_train = []
    obs2_train = []
    for obs_pos in obs1:
        obs1_train.append((obs_pos[0]*gap, obs_pos[1]*gap))
    for obs_pos in obs2:
        obs2_train.append((obs_pos[0]*gap, obs_pos[1]*gap))

    # training data
    X = []
    y = []
    for obs_pos in obs1_train:
        X.append(obs_pos)
        y.append(1)
    for obs_pos in obs2_train:
        X.append(obs_pos)
        y.append(2)
    clf.fit(X, y)

    # plot the hyperplane
    # create grid to evaluate model
    # generate grid along first two dimensions followed by path
    x_min = min([spot.get_real_pos(gap)[0] for spot in path])
    x_max = max([spot.get_real_pos(gap)[0] for spot in path])
    y_min = min([spot.get_real_pos(gap)[1] for spot in path])
    y_max = max([spot.get_real_pos(gap)[1] for spot in path])
    xx = np.linspace(x_min, x_max, 200)
    yy = np.linspace(y_min, y_max, 200)
    
    # add start point and end point to xx, yy
    start_pos = start.get_real_pos(gap)
    end_pos = end.get_real_pos(gap)


    # YY, XX = np.meshgrid(yy, xx)
    # xy = np.vstack([XX.ravel(), YY.ravel()]).T
    # Z = clf.decision_function(xy).reshape(XX.shape)
    # print('Z: ', Z.shape)
    
    # plt.gca().invert_yaxis()

    # plt.contour(XX, YY, Z, colors='k', levels=[-1, 0, 1], alpha=0.5,
    #         linestyles=['--', '-', '--'])
    # # plot support vectors
    # plt.scatter(clf.support_vectors_[:, 0], clf.support_vectors_[:, 1], s=100,
    #         linewidth=1, facecolors='none', edgecolors='k')
    # plt.show()
    # x = []
    # y = []
    # for i in range(XX.shape[0]):
    #     for j in range(XX.shape[1]):
    #         if abs(Z[i][j]) < 0.01:
    #             x.append(XX[i][j])
    #             y.append(YY[i][j])
    # path_svm = []

    # # find nearest start point
    # xxx = np.where(xx == start_pos[0])
    # yyy = np.where(yy == start_pos[1])
    # path_svm.append((xx[xxx[0][0]], yy[yyy[0][0]]))
    # current = xxx[0][0]
    # i = current + 1
    # while(i < len(x)):
    #     if Utils.distance_real((x[current], y[current]), (x[i], y[i])) < 200:
    #         path_svm.append((x[i], y[i]))
    #         current =  i

    angle = []
    for i in range(0, 120, 10):
        angle.append(i * math.pi / 180)
        angle.append(-i * math.pi / 180)
    print('angle: ', len(angle))
    path_svm = []
    path_svm.append(start_pos)
    theta_current = 0
    while Utils.distance_real(path_svm[-1], end_pos) > 20:
        min_heuristic = 10000
        x_next = None
        y_next = None
        theta_next = None
        for i in range(len(angle)):
            x_temp = path_svm[-1][0] + gap * math.cos(theta_current + angle[i])
            y_temp = path_svm[-1][1] + gap * math.sin(theta_current + angle[i])
            if x_temp >= 800 or x_temp <= 0 or y_temp >= 800 or y_temp <= 0:
                continue
            temp_spot = grid[int(x_temp // gap)][int(y_temp // gap)]
            if not temp_spot.is_barrier():
                #TODO: change to distance to end point not distance real
                heuristic = np.abs(clf.decision_function([(x_temp, y_temp)])) + np.log10(Utils.distance_real((x_temp, y_temp), end_pos))
                if heuristic < min_heuristic :
                    min_heuristic = heuristic
                    x_next = x_temp
                    y_next = y_temp
                    theta_next = theta_current + angle[i]
        if x_next == None and y_next == None:
            print("Can't find path")
            break
        path_svm.append((x_next, y_next))
        theta_current = theta_next
       

    return path_svm, obs1, obs2
    # return path_svm