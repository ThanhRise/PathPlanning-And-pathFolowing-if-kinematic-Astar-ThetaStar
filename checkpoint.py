import pygame
import math
from queue import PriorityQueue
from sklearn.neighbors import KDTree
import scipy.interpolate as interpolate
import numpy as np
import matplotlib.pyplot as plt

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

    def chage_linear_velocity(self, delta_u):
        self.u = self.u + delta_u
        if self.u > self.max_u:
            self.u = self.max_u
        elif self.u < self.min_u:
            self.u = self.min_u

    def move_to(self, angle, win, grid, end):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        self.W = angle

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        # print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 1500 and self.dist((self.x, self.y), end.get_real_pos()) > 10:
            drawNotUpDate(win, grid, ROWS, WIDTH)
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
            pygame.display.update()

    def find_spot_with_angle(self, grid, angle, win):
        gap = WIDTH // ROWS
        dt = 1.5
        theta = self.theta + angle
        x = self.x + ((self.vl + self.vr)/2)*math.cos(theta)*dt
        y = self.y + ((self.vl + self.vr)/2)*math.sin(theta)*dt
        row, col = int(x / gap), int(y / gap)
        row2, col2 = int((x + 10) / gap), int(y / gap)
        row3, col3 = int(x / gap), int((y + 10) / gap)
        row4, col4 = int((x + 8) / gap), int((y + 8) / gap)
        row5, col5 = int((x - 8) / gap), int((y - 8) / gap)
        row6, col6 = int((x - 10) / gap), int(y / gap)
        row7, col7 = int(x / gap), int((y - 10) / gap)
        row8, col8 = int((x + 8) / gap), int((y - 8) / gap)
        row9, col9 = int((x - 8) / gap), int((y + 8) / gap)

        spot = grid[row][col]
        # pygame.draw.line(win, RED, (self.x, self.y), (x, y), 1)
        # pygame.display.update()
        if spot.is_barrier() or grid[row2][col2].is_barrier() or grid[row3][col3].is_barrier() or grid[row4][col4].is_barrier() or grid[row5][col5].is_barrier() or grid[row6][col6].is_barrier() or grid[row7][col7].is_barrier() or grid[row8][col8].is_barrier() or grid[row9][col9].is_barrier():
            return None, None, None
        return spot, x, y

    def find_next_spot(self, grid, end, win, distance, kdTree: KDTree):
        current = self.x, self.y
        gap = WIDTH // ROWS
        fixEndPos = end.get_real_pos()[0], end.get_real_pos()[1]

        # the next point will be based on velocity, time and angle
        if current != end.get_real_pos():
            neighbors = []
            for i in range(len(self.angle)):
                spot, x, y = self.find_spot_with_angle(
                    grid, self.angle[i], win)
                if spot is not None:
                    neighbors.append((spot, x, y, self.angle[i]))
            # print(len(neighbors))
            if len(neighbors) > 0:
                # find the spot with the smallest distance to the end
                min_dist_spot = 100000
                min_dist = 100000
                min_spot = None
                obstacle_dist = 0
                f_cost = 100000
                for i in range(len(neighbors)):
                    # dist = self.dist((neighbors[i][1], neighbors[i][2]), fixEndPos)

                    dist = distance[neighbors[i][0]]
                    nearest_obstacle_dist, _ = kdTree.query(
                        [(neighbors[i][1], neighbors[i][2])])
                    nearest_core = 1 / (nearest_obstacle_dist + 1)
                    if nearest_core * 300 + dist < f_cost:
                        f_cost = nearest_core * 300 + dist
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

    def move_back(self, grid, win):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        vr_prev = self.vr
        vl_prev = self.vl
        self.vr = -self.u / 2
        self.vl = -self.u / 2
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 2000:
            drawNotUpDate(win, grid, ROWS, WIDTH)
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
            pygame.display.update()
        self.vr = vr_prev
        self.vl = vl_prev

    def dist(self, point1, point2):
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

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def trail(self, pos, map, color):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
        self.trail_set.append(pos)


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


def find_move_path(robot: Robot, draw, grid, start, end, win):
    gap = WIDTH // ROWS
    distance_to_end = BFS(grid, end)
    if distance_to_end[start] == float(10000):
        return False
    distance_to_start = BFS(grid, start)
    make_barrier_edge(grid, win)
    obstacle = [spot.get_real_pos() for row in grid for spot in row if spot.is_barrier()]
    kdTree = KDTree(obstacle)
    fixEndPos = end.get_real_pos()[0], end.get_real_pos()[1]
    fixStartPos = start.get_real_pos()[0], start.get_real_pos()[1]
    current_start = start.get_real_pos()[0], start.get_real_pos()[1]
    current_end = end.get_real_pos()[0], end.get_real_pos()[1]
    path_to_end = [] 
    path_to_end.append((current_start, 0))
    path_to_start = []
    path_to_start.append((current_end, 0))
    path = []
    IsPlaning = True

    while IsPlaning:
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
        # print("current_start", current_start)
        dis_obs_current_to_end , _ = kdTree.query([current_start])
        pygame.draw.circle(win, GREY, (int(current_start[0]), int(current_start[1])), int(dis_obs_current_to_end), 3)
        for i, point in enumerate(path_to_start):
            dis_current_end = dist_real(current_start, point[0])
            if dis_obs_current_to_end > dis_current_end:
                # join path_to_start to path_to_end
                # path = path_to_end[:] + path_to_start[i::-1]
                path = [x[0] for x in path_to_end] + [x[0] for x in path_to_start[i::-1]]
                pygame.draw.line(win, GREEN, (int(current_start[0]), int(current_start[1])), (int(point[0][0]), int(point[0][1])), 3)
                pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            dist_point_max = 0
            for i in range(360):
                next_point = int(current_start[0] + dis_obs_current_to_end * math.cos(math.radians(i))), int(current_start[1] + dis_obs_current_to_end* math.sin(math.radians(i)))
                for pre_point in path_to_end:
                    if dist_real(pre_point[0], next_point) < pre_point[1]:
                        continue
                if next_point[0] < 0 or next_point[0] > WIDTH or next_point[1] < 0 or next_point[1] > WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next , _ = kdTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(next_point[0]/ gap), int(next_point[1] / gap)
                score = 150*(1.0 / dis_obs_next) + distance_to_end[grid[row][col]]
                # score = 1000*(1.0 / dis_obs_next) + dist_real(next_point, fixEndPos)
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
                    dist_point_max = dis_obs_next
            pygame.draw.line(win, GREEN, (int(current_start[0]), int(current_start[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            pygame.display.update()
            current_start = next_point_max
            path_to_end.append((current_start, dist_point_max))

        if not IsPlaning:
            break

        dis_obs_current_to_start , _ = kdTree.query([current_end])
        pygame.draw.circle(win, GREY, (int(current_end[0]), int(current_end[1])), int(dis_obs_current_to_start), 3)
        for i, point in enumerate(path_to_end):
            dis_current_start = dist_real(current_end, point[0])
            if dis_obs_current_to_start > dis_current_start:
                # path_to_end = path_to_end[:i:1] + path_to_start[ : : -1]
                path = [x[0] for x in path_to_end[:i:1]] + [x[0] for x in path_to_start[::-1]]
                pygame.draw.line(win, GREEN, (int(current_end[0]), int(current_end[1])), (int(point[0][0]), int(point[0][1])), 3)
                pygame.display.update()
                IsPlaning = False
                break
        else:
            Min_score = 10000
            next_point_max = None
            dist_point_max = 0
            for i in range(360):
                next_point = int(current_end[0] + dis_obs_current_to_start * math.cos(math.radians(i))), int(current_end[1] + dis_obs_current_to_start* math.sin(math.radians(i)))
                for pre_point in path_to_start:
                    if dist_real(pre_point[0], next_point) < pre_point[1]:
                        continue
                if next_point[0] < 0 or next_point[0] > WIDTH or next_point[1] < 0 or next_point[1] > WIDTH:
                    continue
                # print("next_point: ", next_point[0], next_point[1])
                dis_obs_next , _ = kdTree.query([next_point])
                if dis_obs_next == 0:
                    continue
                row, col = int(next_point[0]/ gap), int(next_point[1] / gap)
                score = 150*(1.0 / dis_obs_next) + distance_to_start[grid[row][col]]
                # score = 1000*(1.0 / dis_obs_next) + dist_real(next_point, fixStartPos)
                if score < Min_score:
                    Min_score = score
                    next_point_max = next_point
                    dist_point_max = dis_obs_next
            pygame.draw.line(win, GREEN, (int(current_end[0]), int(current_end[1])), (int(next_point_max[0]), int(next_point_max[1])), 3)
            pygame.display.update()
            current_end = next_point_max
            path_to_start.append((current_end, dist_point_max))
    # print("fixStartPos: ", fixStartPos)
    # print("fixEndPos: ", fixEndPos)
    print("path_to_end: ", path)

    point_control = np.array(path)
    x = point_control[:, 0]
    y = point_control[:, 1]
    k = 3
    tck, u = interpolate.splprep([x, y], k = k)
    u_new = np.linspace(u.min(), u.max(), 100)
    x_bspline, y_bspline = interpolate.splev(u_new, tck)

    B_spline = list(zip(x_bspline, y_bspline))
    for i in range(len(B_spline) - 1):
        pygame.draw.line(win, RED, (int(B_spline[i][0]), int(B_spline[i][1])), (int(B_spline[i+1][0]), int(B_spline[i+1][1])), 3)
        pygame.display.update()

    return True


def load_map(file_name, grid: list[list[Spot]], name_map):
    # open file
    end = None
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
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


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
                    Map = "map1"
                if event.key == pygame.K_2:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map2")
                    Map = "map2"
                if event.key == pygame.K_3:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map3")
                    Map = "map3"
                if event.key == pygame.K_4:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map4")
                    Map = "map4"
                if event.key == pygame.K_5:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map(filename, grid, "map5")
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
