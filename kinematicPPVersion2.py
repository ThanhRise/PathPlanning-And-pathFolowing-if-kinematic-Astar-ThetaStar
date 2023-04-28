import pygame
import math
# from queue import PriorityQueue
from sklearn.neighbors import KDTree
import scipy.interpolate as interpolate
import numpy as np


WIDTH = 800
ROWS = 50
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Theta* Path Finding Algorithm")

# def write_info(win, vl, vr, theta):
#     txt = f"vl = {vl} m/s | vr = {vr} m/s | theta = {int(math.degrees(theta))} rad"
#     font = pygame.font.SysFont("comicsans", 20)
#     text = font.render(txt, 1, (0, 0, 0))
#     win.blit(text, (WIDTH - 10 - text.get_width(), 10))


RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
path = []


class DynamicObstacle:
    possible_velocities = ((0, 1), (-1, 0), (1, 0), (0, -1))

    def __init__(self, x, y, velocity):
        self.x = x
        self.y = y
        self.velocity = velocity

    def move(self, grid):
        grid[self.x][self.y].reset_to_prev_color()
        self.x += self.velocity[0]
        self.y += self.velocity[1]
        if self.y > ROWS-2 or self.y < 2 or self.x > ROWS-2 or self.x < 2:
            self.velocity[1] = -self.velocity[1]
            self.velocity[0] = -self.velocity[0]

        grid[self.x][self.y].make_dynamic_obs()
        # pygame.time.delay(300)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.neighbors_distance = []
        self.width = width
        self.total_rows = total_rows
        self.prev_color = None  # Initialize prev_color to None

    def get_pos(self):
        return self.row, self.col

    def get_real_pos(self):
        gap = WIDTH // ROWS
        x = gap * self.row + 8   # để đường thẳng nằm giữa ô vuông
        y = gap * self.col + 8
        return x, y

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = WHITE

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def make_theta(self):
        self.color = YELLOW

    def reset_to_prev_color(self):
        if self.prev_color is not None:
            self.color = self.prev_color
            self.prev_color = None

    def make_dynamic_obs(self):
        self.prev_color = self.color  # save current color as prev_color
        self.color = GREY

    def time_became_dynamic_obs(self, list_dynamicobs):
        time_global = pygame.time.get_ticks()
        # print("time_global: ", time_global)
        for i in range(10):
            for obs in list_dynamicobs:
                x = obs.x + obs.velocity[0]*i
                y = obs.y + obs.velocity[1]*i
                if self.row == x and self.col == y:
                    # print("time_became_dynamic_obs: ", time_global + i*0.5)
                    return time_global + i*500

        return 1000000000

    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))

    # update the neighbors for BFS

    def update_neighbors_distance(self, grid):
        # self.neighbors_distance = []
        distance_diagonal = math.sqrt(2)
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors_distance.append((grid[self.row + 1][self.col], 1))

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors_distance.append((grid[self.row - 1][self.col], 1))

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors_distance.append((grid[self.row][self.col + 1], 1))

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors_distance.append((grid[self.row][self.col - 1], 1))

        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
            self.neighbors_distance.append(
                (grid[self.row + 1][self.col + 1], distance_diagonal))

        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
            self.neighbors_distance.append(
                (grid[self.row - 1][self.col + 1], distance_diagonal))

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
            self.neighbors_distance.append(
                (grid[self.row + 1][self.col - 1], distance_diagonal))

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
            self.neighbors_distance.append(
                (grid[self.row - 1][self.col - 1], distance_diagonal))

    def __lt__(self, other):
        return False


class Robot:
    def __init__(self, startPos, robotImg, width):

        self.x, self.y = startPos
        self.theta = 0
        self.width = width
        self.vr = 20  # right velocity
        self.vl = 20  # left velocity
        self.u = (self.vl + self.vr)/2  # linear velocity
        self.max_u = 30
        self.min_u = 10
        self.W = 0  # angular velocity
        self.a = 15  # width of robot
        self.trail_set = []
        self.dt = 0  # time step
        self.pathRb = []
        self.angle = []
        one_degree = math.pi / 180
        self.angle.append(0)
        for i in range(1, 60, 4):
            self.angle.append(i * one_degree)
            self.angle.append(-i * one_degree)
        self.img = pygame.image.load(robotImg)
        self.img = pygame.transform.scale(self.img, (16, 16))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def move(self, event=None):
        self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
        self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
        self.theta += (self.vr - self.vl)/self.width*self.dt
        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.following()

    def following(self):
        target = self.pathRb[0]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y

        self.u = delta_x * math.cos(self.theta) + \
            delta_y * math.sin(self.theta)
        self.W = (-1/self.a) * math.sin(self.theta) * delta_x + \
            (1/self.a) * math.cos(self.theta)*delta_y

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        if dist_real((self.x, self.y), target) < 20 and len(self.pathRb) > 1:
            self.pathRb.pop(0)

    def chage_linear_velocity(self, delta_u):
        self.u = self.u + delta_u
        if self.u > self.max_u:
            self.u = self.max_u
        elif self.u < self.min_u:
            self.u = self.min_u

    def move_to(self, angle, win, grid, target, total_dt):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        self.W = angle

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        # print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 1500 and dist_real((self.x, self.y), target) > 10:
            
            drawNotUpDate(win, grid, ROWS, WIDTH)
            if (total_dt > 0.5):
                # update dynamic obstacles
                for obs in dynamic_obstacles:
                    # check if obstacle will move out of bounds
                    obs.move(grid)
                total_dt = 0
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            self.theta += (self.vr - self.vl)/self.width*self.dt/1.5

            # self.theta += self.W / 400

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0

            self.rotated = pygame.transform.rotozoom(
                self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center=(self.x, self.y))
            self.draw(win)
            self.trail((self.x, self.y), win, GREEN)
            # time_step = (pygame.time.get_ticks() - last_time) / 1000
            # self.dt = time_step
            # last_time = pygame.time.get_ticks()
            total_dt += (pygame.time.get_ticks() - last_time) / 1000
            pygame.display.update()
        return total_dt

    def find_spot_with_angle(self, grid: list[list[Spot]], angle):
        gap = WIDTH // ROWS
        dt = 1.5
        theta = self.theta + angle
        x = self.x + ((self.vl + self.vr)/2)*math.cos(theta)*dt
        y = self.y + ((self.vl + self.vr)/2)*math.sin(theta)*dt
        if x > WIDTH or x < 0 or y > WIDTH or y < 0:
            print("out of range")
            return None, None, None
        row, col = int(x / gap), int(y / gap)
        row2, col2 = int((x + 10) / gap), int(y / gap)
        row3, col3 = int(x / gap), int((y + 10) / gap)
        row4, col4 = int((x + 12) / gap), int((y + 12) / gap)
        row5, col5 = int((x - 12) / gap), int((y - 12) / gap)
        row6, col6 = int((x - 10) / gap), int(y / gap)
        row7, col7 = int(x / gap), int((y - 10) / gap)
        row8, col8 = int((x + 12) / gap), int((y - 12) / gap)
        row9, col9 = int((x - 12) / gap), int((y + 12) / gap)

        spot = grid[row][col]
        if spot.is_barrier() or grid[row2][col2].is_barrier() or grid[row3][col3].is_barrier() or grid[row4][col4].is_barrier() or grid[row5][col5].is_barrier() or grid[row6][col6].is_barrier() or grid[row7][col7].is_barrier() or grid[row8][col8].is_barrier() or grid[row9][col9].is_barrier():
            return None, None, None
        
        time_global = pygame.time.get_ticks()

        if spot.time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row2][col2].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row3][col3].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row4][col4].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row5][col5].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row6][col6].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row7][col7].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row8][col8].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000 or \
            grid[row9][col9].time_became_dynamic_obs(dynamic_obstacles) < time_global + 4000:
            return None, None, None
        return spot, x, y

    def find_next_spot(self, grid: list[list[Spot]], target, win, distance, kdTree: KDTree):
        current = self.x, self.y
        gap = WIDTH // ROWS
        # the next point will be based on velocity, time and angle
        if current != target:
            neighbors = []
            for i in range(len(self.angle)):
                spot, x, y = self.find_spot_with_angle(
                    grid, self.angle[i], win)
                if spot is not None:
                    neighbors.append((spot, x, y, self.angle[i]))
            # print(len(neighbors))
            if len(neighbors) > 0:
                # find the spot with the smallest distance to the target
                min_spot = None
                obstacle_dist = 0
                f_cost = 100000
                for i in range(len(neighbors)):
                    # dist = distance[neighbors[i][0]]
                    dist = dist_real(
                        (neighbors[i][1], neighbors[i][2]), target)
                    nearest_obstacle_dist, _ = kdTree.query(
                        [(neighbors[i][1], neighbors[i][2])])
                    nearest_core = 1 / (nearest_obstacle_dist + 1)
                    if nearest_core * 250 + dist < f_cost:
                        f_cost = nearest_core * 250 + dist
                        min_spot = neighbors[i][0]
                        angle_selected = neighbors[i][3]
                        obstacle_dist = nearest_obstacle_dist

                    # if distance[neighbors[i][0]] < min_dist_spot and dist < min_dist:
                    #     min_dist_spot = distance[neighbors[i][0]]
                    #     min_spot = neighbors[i][0]
                    #     angle_selected = neighbors[i][3]

                return min_spot, angle_selected, obstacle_dist
            else:
                return None, None, None

    def move_back(self, grid, win, total_dt):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        vr_prev = self.vr
        vl_prev = self.vl
        self.vr = -self.u / 2
        self.vl = -self.u / 2
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 2000:
            drawNotUpDate(win, grid, ROWS, WIDTH)
            if (total_dt > 0.5):
                # update dynamic obstacles
                for obs in dynamic_obstacles:
                    # check if obstacle will move out of bounds
                    obs.move(grid)
                total_dt = 0
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            # self.theta -= (self.vr - self.vl)/self.width*self.dt/1.5

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0

            self.rotated = pygame.transform.rotozoom(
                self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center=(self.x, self.y))
            self.draw(win)
            self.trail((self.x, self.y), win, GREEN)
            total_dt += (pygame.time.get_ticks() - last_time) / 1000
            pygame.display.update()
        self.vr = vr_prev
        self.vl = vl_prev
        return total_dt

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def trail(self, pos, map, color):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
        self.trail_set.append(pos)


def h2(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # return abs(x1 - x2) + abs(y1 - y2)
    if abs(x1 - x2) >= abs(y1 - y2):
        return abs(x1 - x2)
    if abs(x1 - x2) < abs(y1 - y2):
        return abs(y1 - y2)


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def dist_real(point1, point2):
    (x1, y1) = point1
    (x2, y2) = point2
    x1 = float(x1)
    x2 = float(x2)
    y1 = float(y1)
    y2 = float(y2)

    px = (x1 - x2) ** 2
    py = (y1 - y2) ** 2
    distance = (px + py) ** 0.5
    return distance


def BFS(grid: list[list[Spot]], end: Spot):
    # update the neighbor_distance of each node
    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)
    distance = {spot: float(10000) for row in grid for spot in row}
    distance[end] = 0
    queue = []
    queue.append(end)
    while queue:
        current = queue.pop(0)
        for neighbor, dist in current.neighbors_distance:
            if distance[current] + dist < distance[neighbor]:
                distance[neighbor] = distance[current] + dist
                queue.append(neighbor)
    return distance


# robot move to goal with A* algorithm

def make_barrier_edge(grid: list[list[Spot]], win):
    # make the edge of the map as barrier
    for i in range(0, len(grid)):
        grid[i][0].make_barrier()
        grid[i][len(grid[0]) - 1].make_barrier()
    for i in range(0, len(grid[0])):
        grid[0][i].make_barrier()
        grid[len(grid) - 1][i].make_barrier()
    drawNotUpDate(win, grid, ROWS, WIDTH)


def find_path(grid: list[list[Spot]], start: Spot, end: Spot, distance_to_end, distance_to_start, kdTree, win):
    gap = WIDTH // ROWS
    current_start = start.get_real_pos()[0], start.get_real_pos()[1]
    current_end = end.get_real_pos()[0], end.get_real_pos()[1]
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
        dis_obs_current_to_end, _ = kdTree.query([current_start])
        pygame.draw.circle(win, GREY, (int(current_start[0]), int(
            current_start[1])), int(dis_obs_current_to_end), 3)
        for i, point in enumerate(path_to_start):
            dis_current_end = dist_real(current_start, point)
            if dis_obs_current_to_end > dis_current_end:
                # join path_to_start to path_to_end
                path_to_end = path_to_end + path_to_start[i::-1]
                pygame.draw.line(win, GREEN, (int(current_start[0]), int(
                    current_start[1])), (int(point[0]), int(point[1])), 3)
                pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(360):
                next_point = int(current_start[0] + dis_obs_current_to_end * math.cos(math.radians(
                    i))), int(current_start[1] + dis_obs_current_to_end * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > WIDTH or next_point[1] < 0 or next_point[1] > WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = kdTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(next_point[0] / gap), int(next_point[1] / gap)
                score = 150*(1.0 / dis_obs_next) + \
                    distance_to_end[grid[row][col]]
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            pygame.draw.line(win, GREEN, (int(current_start[0]), int(
                current_start[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            pygame.display.update()
            current_start = next_point_max
            path_to_end.append(current_start)

        if not IsPlaning:
            break
        dis_obs_current_to_start, _ = kdTree.query([current_end])
        pygame.draw.circle(win, GREY, (int(current_end[0]), int(
            current_end[1])), int(dis_obs_current_to_start), 3)
        for i, point in enumerate(path_to_end):
            dis_current_start = dist_real(current_end, current_start)
            if dis_obs_current_to_start > dis_current_start:
                # print("path_to_end", path_to_end)
                path_to_end = path_to_end[:i-1:] + path_to_start[::-1]
                pygame.draw.line(win, GREEN, (int(current_end[0]), int(
                    current_end[1])), (int(current_start[0]), int(current_start[1])), 3)
                pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            for i in range(360):
                next_point = int(current_end[0] + dis_obs_current_to_start * math.cos(math.radians(
                    i))), int(current_end[1] + dis_obs_current_to_start * math.sin(math.radians(i)))
                if next_point[0] < 0 or next_point[0] > WIDTH or next_point[1] < 0 or next_point[1] > WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next, _ = kdTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(next_point[0] / gap), int(next_point[1] / gap)
                score = 150*(1.0 / dis_obs_next) + \
                    distance_to_start[grid[row][col]]
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
            pygame.draw.line(win, GREEN, (int(current_end[0]), int(
                current_end[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            pygame.display.update()
            current_end = next_point_max
            path_to_start.append(current_end)
    print("path_to_end: ", path_to_end)
    point_control = np.array(path_to_end)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k=k)
    u_new = np.linspace(u.min(), u.max(), 100)
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    for i in range(len(B_spline) - 1):
        pygame.draw.line(win, RED, (int(B_spline[i][0]), int(
            B_spline[i][1])), (int(B_spline[i+1][0]), int(B_spline[i+1][1])), 3)
        pygame.display.update()

    return B_spline


def find_move_path(robot: Robot, draw, grid, start, end, win):
    global time_global
    fixEndPoint = end.get_real_pos()
    distance_to_end = BFS(grid, end)
    distance_to_start = BFS(grid, start)
    make_barrier_edge(grid, win)
    obstacle = [spot.get_real_pos()
                for row in grid for spot in row if spot.is_barrier()]
    kdTree = KDTree(obstacle)
    robot.pathRb = find_path(grid=grid, start=start, end=end, win=win,
                             distance_to_end=distance_to_end, distance_to_start=distance_to_start, kdTree=kdTree)
    kdTree_path = KDTree(robot.pathRb)
    pygame.time.delay(5000)
    # print(robot.pathRb.pop(0))
    time_global = 0

    # pop the point that is too close to the robot
    index = 0
    while dist_real((robot.x, robot.y), robot.pathRb[index]) < 70:
        index += 1
    print("index: ", index)
    print("distance: ", dist_real((robot.x, robot.y), robot.pathRb[index]))
    total_dt = 0
    last_time = pygame.time.get_ticks()
    while dist_real((robot.x, robot.y), fixEndPoint) > 20:
        target = robot.pathRb[index] # target is float
        index += 1
        while dist_real((robot.x, robot.y), target) > 20:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            drawNotUpDate(win, grid, ROWS, WIDTH)
            if (total_dt > 0.5):
                # update dynamic obstacles
                for obs in dynamic_obstacles:
                    # check if obstacle will move out of bounds
                    obs.move(grid)
                total_dt = 0
            mini_dis_to_target, target_min = kdTree_path.query([(robot.x, robot.y)])
            target_min = target_min[0][0]
            dis_to_current_target = dist_real((robot.x, robot.y), target)
            if target != robot.pathRb[target_min] and mini_dis_to_target < dis_to_current_target and target_min > index:
                target = robot.pathRb[target_min]
                index = target_min + 1
                
            # pygame.display.update()
            # robot.dt = (pygame.time.get_ticks() - lasttime) / 1000
            # total_dt += robot.dt
            # time_global += robot.dt
            # lasttime = pygame.time.get_ticks()
            # robot.move()
            # robot.draw(win)
            # robot.trail((robot.x, robot.y), win, RED)
            # pygame.display.update()

            # set_time = pygame.time.get_ticks()
            next_pos, angle, obstacle_dist = robot.find_next_spot(
                grid, target, win, distance_to_end, kdTree)
            # pygame.draw.line(win, RED, (int(robot.x), int(robot.y)), (int(target[0]), int(target[1])), 5)
            pygame.display.update()
            total_dt += (pygame.time.get_ticks() - last_time) / 1000
            if next_pos:
                # set_time = (pygame.time.get_ticks() - set_time) / 1000
                # print("set time 1: ", set_time)
                # set_time = pygame.time.get_ticks()
                total_dt = robot.move_to(angle, win, grid, target, total_dt)
                # set_time = (pygame.time.get_ticks() - set_time) / 1000
                # print("move time 2: ", set_time)

                # chage velocity
                # delta_u = 0
                # if obstacle_dist_pre != 0:
                #     delta_u = int(obstacle_dist - obstacle_dist_pre)*0.1
                #     print("delta_u: ", delta_u)
                # obstacle_dist_pre = obstacle_dist
                # robot.chage_linear_velocity(delta_u)
            else:
                total_dt = robot.move_back(grid, win, total_dt)
            last_time = pygame.time.get_ticks()

    return True


def load_map(file_name, grid: list[list[Spot]], name_map):
    # open file

    file_name = open(file_name, "r")
    for line in file_name:
        # with line start with "map1" is map 1
        if line.startswith(name_map):
            # read next line
            for i in range(ROWS):
                line1 = file_name.readline()
                for j in range(ROWS*2):
                    if line1[j] == "1":
                        grid[i][int(j/2)].make_barrier()
                    elif line1[j] == "2":
                        start = grid[i][int(j/2)]
                        start.make_start()
                    elif line1[j] == "3":
                        end = grid[i][int(j/2)]
                        end.make_end()
    # close file
    file_name.close()
    return grid, start, end


def load_dynamic_obstacles(file_name):
    dynamic_obstacles = []
    with open(file_name, 'r') as f:
        for line in f:
            # parse obstacle position and velocity from file
            x, y, vx, vy = map(int, line.strip().replace(
                '[', '').replace(']', '').split(','))

            # create dynamic obstacle with given velocity and position
            obs = DynamicObstacle(x, y, [vx, vy])

            # add obstacle to the list
            dynamic_obstacles.append(obs)
    return dynamic_obstacles


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows, width):
    # gap = width // rows
    # for i in range(rows):
    #     pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
    #     for j in range(rows):
    #         pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))
    pass

def draw(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()


def drawNotUpDate(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    x, y = pos

    row = x // gap
    col = y // gap

    return row, col


def main(win, width):
    global dynamic_obstacles
    pygame.init()
    grid = make_grid(ROWS, width)
    start = None
    end = None
    MARK = False
    run = True
    Map = None
    filename = "map.txt"

    while run:
        if MARK == False:
            draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map1")
                    dynamic_obstacles = load_dynamic_obstacles(
                        "dynamic_obstacles.txt")
                    Map = "map1"
                if event.key == pygame.K_2:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map2")
                    dynamic_obstacles = load_dynamic_obstacles(
                        "dynamic_obstacles.txt")
                    Map = "map2"
                if event.key == pygame.K_3:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map3")
                    dynamic_obstacles = load_dynamic_obstacles(
                        "dynamic_obstacles.txt")
                    Map = "map3"
                if event.key == pygame.K_4:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map4")
                    dynamic_obstacles = load_dynamic_obstacles(
                        "dynamic_obstacles.txt")
                    Map = "map4"
                if event.key == pygame.K_5:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map5")
                    dynamic_obstacles = load_dynamic_obstacles(
                        "dynamic_obstacles.txt")
                    Map = "map5"
            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                time_global = 0
                robot = Robot(start.get_real_pos(
                ), "Robot.png", 16)
                # if Map == "map5":
                #     robot.theta = 3*math.pi/2
                if event.key == pygame.K_SPACE and start and end:
                    MARK = find_move_path(robot, lambda: draw(win, grid, ROWS, width),
                                          grid, start, end, win)

                if event.key == pygame.K_c:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()


main(WIN, WIDTH)

# --------------------------------------------------------------------------------------------------------------
# kiểm tra obs trong bán kính an toàn trong khoảng t s --> replan
# làm đến đầu chụp lại hình đưa vào slide

# ------------------------------------------------------------
# thay doi thoi gina va van toc trong viec tim cac diem tiep theo
