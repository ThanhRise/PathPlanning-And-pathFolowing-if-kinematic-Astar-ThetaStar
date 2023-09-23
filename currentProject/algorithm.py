import math
from spot import Spot
from constant import Constant
import pygame
import numpy as np
from utils import Utils
from scipy.spatial import KDTree
import scipy.interpolate as interpolate
from environment import Environment
from robot import Robot
from queue import PriorityQueue
from dubinsPath import dubins_path_length
from dubinsPath import Waypoint

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


def find_path(robot: Robot, MAP: Environment, start: Spot):

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

def find_path_one_way(robot: Robot, MAP: Environment, start: Spot):
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
        pygame.draw.circle(MAP.WIN, Constant.GREY, (int(current_start[0]), int(current_start[1])), int(dis_obs_current_to_start), 10)
        pygame.display.update()
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
    u_new = np.linspace(u.min(), u.max(), 3*Utils.distance_spot((start.row, start.col), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    return B_spline

def replan(robot: Robot, list_obstacles, MAP):
    step_horizontal = MAP.GAP / 2
    print("replan")
    restore_grid = []
    for obs in list_obstacles:
        if Utils.distance_real((obs.x, obs.y), (robot.x, robot.y)) < 200:
            # print("angle", obs.theta, "atan2", math.atan2(obs.y - robot.y, obs.x - robot.x), "angle - atan2", obs.theta - math.atan2(obs.y - robot.y, obs.x - robot.x))
            x_next = obs.x + 6 * obs.velocity * math.cos(obs.theta)
            y_next = obs.y + 6 * obs.velocity * math.sin(obs.theta)
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
         
                x_current += 16 * math.cos(obs.theta)
                y_current += 16 * math.sin(obs.theta)
                if x_current + 8 >= 800 or x_current - 8 <= 0 or y_current + 8 >= 800 or y_current - 8 <= 0:
                    break
                if Utils.distance_real((x_current, y_current), (robot.x, robot.y)) > 24:
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

    # draw(win, MAP.grid, ROWS, WIDTH)
    obstacle = [spot.get_real_pos(MAP.GAP) for row in MAP.grid for spot in row if (spot.is_barrier() or spot.is_dynamic_obs())]
    DObstacle = [spot.get_real_pos(MAP.GAP) for row in MAP.grid for spot in row if spot.is_dynamic_obs()]
    start_pos_current = MAP.grid[int(robot.x // MAP.GAP)][int(robot.y // MAP.GAP)]
    MAP.start = start_pos_current
    # print distance to end
    robot.distance_to_end = BFS(MAP.grid, MAP.end)
    robot.distance_to_start = BFS(MAP.grid, start_pos_current)
    # robot.KDTree = KDTree(obstacle)
    dynamicKDTree = KDTree(DObstacle)

    print("KDTREE OK")
    # robot.pathRb = find_path(robot, MAP, start_pos_current)
    robot.pathRb = find_path_with_kinematic(robot, MAP, start_pos_current)
    # robot.pathRb = find_path_two_KDtree(robot, MAP, start_pos_current, dynamicKDTree)

    for spot in restore_grid:
        spot.reset_to_prev_color()
    return robot.pathRb, robot.KDTree, pygame.time.get_ticks()


def find_path_with_kinematic(robot, MAP, start_pos_current, angleEnd = 180):
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
    while Utils.distance_real((x_current, y_current), MAP.end.get_real_pos(MAP.GAP)) > 100:
        min_heuristic = 10000
        maxStep = 16
        x_next = 0
        y_next = 0
        theta_next = 0
        for i in range(len(angle)):
            for step in range(1, maxStep, 5):
                x_temp = x_current + (step*2 + 8) * math.cos(theta_current + angle[i])
                y_temp = y_current + (step*2 + 8) * math.sin(theta_current + angle[i])
                # pygame.draw.line(MAP.WIN, (255, 0, 0), (x_current, y_current), (x_temp, y_temp), 2)
                # pygame.display.update()

                temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    distance_to_end = robot.distance_to_end[temp_spot]
                    distance_to_obs, _ = robot.KDTree.query([(int(x_temp), int(y_temp))])
                    # print("angle: ", angle[i])
                    # print("distance_to_end: ", distance_to_end)
                    # print("distance_to_obs: ", (2000 - len(PATH) * 10) * (1.0 / distance_to_obs))
                    heuristic = distance_to_end + 150 * (1.0 / distance_to_obs)
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
    while Utils.distance_real((x_current, y_current), MAP.end.get_real_pos(MAP.GAP)) > 5:
        min_heuristic = 10000
        x_next = 0
        y_next = 0
        theta_next = 0
        dubins_path_final = []
        for i in range(len(angle)):
            # for step in range(1, maxStep, 5):
            x_temp = x_current + (16) * math.cos(theta_current + angle[i])
            y_temp = y_current + (16) * math.sin(theta_current + angle[i])
            # pygame.draw.line(MAP.WIN, (255, 0, 0), (x_current, y_current), (x_temp, y_temp), 2)
            # pygame.display.update()

            temp_spot = MAP.grid[int(x_temp // MAP.GAP)][int(y_temp // MAP.GAP)]
            if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                temp_angle = (theta_current + angle[i]) * 180 / math.pi
                if temp_angle < 0:
                    temp_angle += 360
                if temp_angle > 360:
                    temp_angle -= 360
                temp_wpt = Waypoint(x_temp, y_temp, temp_angle)
                end_wpt = Waypoint(MAP.end.get_real_pos(MAP.GAP)[0], MAP.end.get_real_pos(MAP.GAP)[1], angleEnd)
                length_path_dubins, dubins_path = dubins_path_length(temp_wpt, end_wpt)
                
                # for k in range(len(dubins_path) - 1):
                #     pygame.draw.line(MAP.WIN, (255, 0, 0), (dubins_path[k][0], dubins_path[k][1]), (dubins_path[k+1][0], dubins_path[k+1][1]), 2)
                #     pygame.display.update()

                heuristic = length_path_dubins
                if heuristic < min_heuristic:
                    min_heuristic = heuristic
                    x_next = x_temp
                    y_next = y_temp
                    theta_next = theta_current + angle[i]
                    # dubins_path_final drop colunm 2
                    dubins_path_final = dubins_path[:, :2]

        if x_next == 0 and y_next == 0:
            print("Can't find path")
            break
        x_current = x_next
        y_current = y_next
        theta_current = theta_next
        # add dubins path to PATH
        PATH = PATH + dubins_path_final.tolist()
        break

    # pygame.time.delay(100000)


    PATH.append(MAP.end.get_real_pos(MAP.GAP))
    if len(PATH) <= 3:
        return PATH
    # use Bspline to smooth the path
    PATH = np.array(PATH)
    x = PATH[:, 0]
    y = PATH[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 3*Utils.distance_spot((start_pos_current.row, start_pos_current.col), (MAP.end.row, MAP.end.col)))
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    return B_spline    

    
def find_path_two_KDtree(robot: Robot, MAP: Environment, start: Spot, DynamicKDtree):


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