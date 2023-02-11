import pygame
import math
from queue import PriorityQueue

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
        x = gap * self.row   # để đường thẳng nằm giữa ô vuông
        y = gap * self.col
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
        self.neighbors_distance = []
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors_distance.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors_distance.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors_distance.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors_distance.append(grid[self.row][self.col - 1])

        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
            self.neighbors_distance.append(grid[self.row + 1][self.col + 1])

        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
            self.neighbors_distance.append(grid[self.row - 1][self.col + 1])

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
            self.neighbors_distance.append(grid[self.row + 1][self.col - 1])

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
            self.neighbors_distance.append(grid[self.row - 1][self.col - 1])

    def update_neighbors(self, robot, grid, start, end):
        self.neighbors = []
        #  get the angle of the robot
        # theta normalization
        if robot.theta < 0:
            theta_norm = robot.theta + 2 * math.pi
        else:
            theta_norm = robot.theta
        # calculate angle of neighbor point, current point and end point
        # if current point is not the start point, then calculate the angle
        angle = math.pi
        if self.x != start.x or self.y != start.y:
            angle = math.atan2(end.y - self.y, end.x - self.x) - \
                math.atan2(robot.y - self.y, robot.x - self.x)
            angle = abs(angle)
            if angle > 2 * math.pi:
                angle = angle - 2 * math.pi

        # if the middle angle of the neighboring point, the current point and the end point is pi, then the robot moves to the right
        if angle < math.pi/4:
            # right up
            if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
                if theta_norm <= math.pi / 3 or (theta_norm <= 3*math.pi / 2 + math.pi / 6 and theta_norm >= 3*math.pi / 2 - math.pi / 6):
                    self.neighbors.append(grid[self.row + 1][self.col - 1])
            # right down
            if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
                if (theta_norm <= math.pi / 2 + math.pi / 6 and theta_norm >= math.pi / 3) or theta_norm <= math.pi / 3:
                    self.neighbors.append(grid[self.row + 1][self.col + 1])
            # left up
            if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
                if (theta_norm <= 3 * math.pi / 2 + math.pi / 3 and theta_norm >= 3*math.pi / 2 - math.pi/3) or (theta_norm <= math.pi + math.pi/3 and theta_norm >= math.pi - math.pi / 3):
                    self.neighbors.append(grid[self.row - 1][self.col - 1])
            # left down
            if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
                if (theta_norm <= math.pi + math.pi / 6 and theta_norm >= math.pi - math.pi / 6) or (theta_norm <= 2 * math.pi - math.pi / 3 and theta_norm >= 2 * math.pi - math.pi / 3):
                    self.neighbors.append(grid[self.row - 1][self.col + 1])

        else:
            # NOTE : chiều của đường tròn lượng giác là cùng chiều kim đồng hồ
            # right
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
                if theta_norm < math.pi / 2 or theta_norm > 2 * math.pi - math.pi / 2:
                    self.neighbors.append(grid[self.row + 1][self.col])
            # left
            if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
                if theta_norm > math.pi - math.pi / 2 and theta_norm < math.pi + math.pi / 2:
                    self.neighbors.append(grid[self.row - 1][self.col])
            # up
            if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
                if theta_norm > (3 * math.pi / 2 - math.pi / 2) and theta_norm < (3 * math.pi / 2 + math.pi / 2):
                    self.neighbors.append(grid[self.row][self.col - 1])
            # down
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
                if theta_norm > math.pi / 2 - math.pi / 2 and theta_norm < math.pi / 2 + math.pi / 2:
                    self.neighbors.append(grid[self.row][self.col + 1])
            # right up
            if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
                if (theta_norm < 2 * math.pi and theta_norm > 5 * math.pi / 4) or (theta_norm < math.pi / 4):
                    self.neighbors.append(grid[self.row + 1][self.col - 1])
            # right down
            if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
                if (theta_norm < math.pi / 4 + math.pi / 2 and theta_norm >= 0) or (theta_norm < 2 * math.pi and theta_norm > 7 * math.pi / 4):
                    self.neighbors.append(grid[self.row + 1][self.col + 1])
            # left up
            if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
                if (theta_norm > 3 * math.pi / 4 and theta_norm < 7 * math.pi / 4):
                    self.neighbors.append(grid[self.row - 1][self.col - 1])
            # left down
            if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
                if theta_norm < 5 * math.pi / 4 + math.pi / 2 and theta_norm > math.pi / 4:
                    self.neighbors.append(grid[self.row - 1][self.col + 1])

    def __lt__(self, other):
        return False


class Robot:
    def __init__(self, startPos, robotImg, width):

        self.x, self.y = startPos
        self.theta = 0
        self.width = width
        self.vr = 15  # right velocity
        self.vl = 15  # left velocity
        self.u = (self.vl + self.vr)/2  # linear velocity
        self.W = 0  # angular velocity
        self.a = 15  # width of robot
        self.trail_set = []
        self.dt = 0  # time step
        self.pathRb = []
        self.angle = []
        one_degree = math.pi / 180
        self.angle.append(0)
        for i in range(1, 60, 5):
            self.angle.append(i * one_degree)
            self.angle.append(-i * one_degree)
        self.img = pygame.image.load(robotImg)
        self.img = pygame.transform.scale(self.img, (20, 20))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def move(self, event=None):
        # self.x += (self.u * math.cos(self.theta) - self.a *
        #            math.sin(self.theta) * self.W)*self.dt
        # self.y += (self.u * math.sin(self.theta) + self.a *
        #            math.cos(self.theta) * self.W)*self.dt
        # self.theta += self.W*self.dt

        self.vr = (self.u + self.W * self.width)/2
        self.vl = (self.u - self.W * self.width)/2
        self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
        self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
        # self.theta += (self.vr - self.vl)/self.width*self.dt

        # self.x += (self.width*(self.vr + self.vl)*(math.sin(self.theta + self.theta*self.dt) -  math.sin(self.theta) ))/ (2*(self.vr -self.vl))
        # self.y += (self.width*(self.vr + self.vl)*(- math.cos(self.theta + self.theta*self.dt) +  math.cos(self.theta) ))/ (2*(self.vr -self.vl))
        # self.theta += self.theta*self.dt

        if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
            self.theta = 0

        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        # self.following()

    def following(self):
        target = self.pathRb[0]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y

        # self.u = delta_x * math.cos(self.theta) + \
        #     delta_y * math.sin(self.theta)
        self.W = (-1/self.a) * math.sin(self.theta) * delta_x + \
            (1/self.a) * math.cos(self.theta)*delta_y

        self.vr = (self.u + self.W * self.width)/2
        self.vl = (self.u - self.W * self.width)/2
        if self.dist((self.x, self.y), target) <= 10:
            self.pathRb.pop(0)

    def move_to(self, angle, win, grid, end):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        self.W = angle

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 2000 and self.dist((self.x, self.y), end.get_real_pos()) > 10:
            drawNotUpDate(win, grid, ROWS, WIDTH)
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            self.theta += (self.vr - self.vl)/self.width*self.dt/2

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
        dt = 2
        theta = self.theta + angle
        x = self.x + ((self.vl + self.vr)/2)*math.cos(theta)*dt
        y = self.y + ((self.vl + self.vr)/2)*math.sin(theta)*dt
        row, col = int(x / gap), int(y / gap)
        row2, col2 = int((x + 16) / gap), int(y / gap)
        row3, col3 = int(x / gap), int((y + 16) / gap)
        row4, col4 = int((x + 16) / gap), int((y + 16) / gap)
        row5, col5 = int((x - 8) / gap), int((y - 8) / gap)
        row6, col6 = int((x - 8) / gap), int(y / gap)
        row7, col7 = int(x / gap), int((y - 8) / gap)

        spot = grid[row][col]
        pygame.draw.line(win, RED, (self.x, self.y), (x, y), 1)
        pygame.display.update()
        if spot.is_barrier() or grid[row2][col2].is_barrier() or grid[row3][col3].is_barrier() or grid[row4][col4].is_barrier() or grid[row5][col5].is_barrier() or grid[row6][col6].is_barrier() or grid[row7][col7].is_barrier():
            return None, None, None
        return spot, x, y

    def find_next_spot(self, grid, end, win):
        current = self.x, self.y

        # the next point will be based on velocity, time and angle
        if current != end.get_real_pos():
            neighbors = []
            for i in range(len(self.angle)):
                spot, x, y = self.find_spot_with_angle(
                    grid, self.angle[i], win)
                if spot is not None:
                    neighbors.append((spot, x, y, self.angle[i]))
            print(len(neighbors))
            if len(neighbors) > 0:
                min_dist = 100000
                min_spot = None
                for i in range(len(neighbors)):
                    dist = self.dist(
                        (neighbors[i][1], neighbors[i][2]), end.get_real_pos())
                    dist2 = self.dist(
                        (neighbors[i][1], neighbors[i][2]), current)
                    # print("dist: ", dist2, "angle: ", angle[i])
                    # print("dist: ", dist, "angle: ", angle[i])
                    if dist < min_dist:
                        min_dist = dist
                        min_spot = neighbors[i][0]
                        angle_selected = neighbors[i][3]
                print(angle_selected)
                return min_spot, angle_selected

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


def reconstruct_path(came_from, current, draw):  # đổi màu các ô đi qua
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def reconstruct_line_path(came_from, current, win):  # vẽ đường đi
    while current in came_from:
        x = current.get_real_pos()
        current = came_from[current]
        pygame.draw.line(win, BLACK, current.get_real_pos(), x)

# use BFS algorithm to save the distance of all nodes to the end node


def BFS(grid, end):
    # update the neighbor_distance of each node
    for row in grid:
        for spot in row:
            spot.update_neighbors_distance(grid)
    distance = {spot: float("inf") for row in grid for spot in row}
    distance[end] = 0
    queue = []
    queue.append(end)
    while queue:
        current = queue.pop(0)
        for neighbor in current.neighbors_distance:
            if distance[neighbor] == float("inf"):
                distance[neighbor] = distance[current] + 1
                queue.append(neighbor)
    return distance


# robot move to goal with A* algorithm


def find_move_path(robot, draw, grid, start, end, win, distance):

    # lasttime = pygame.time.get_ticks()
    while robot.dist((robot.x, robot.y), end.get_real_pos()) > 10:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        # set_time = pygame.time.get_ticks()
        next_pos, angle = robot.find_next_spot(grid, end, win)
        if next_pos:
            # set_time = (pygame.time.get_ticks() - set_time) / 1000
            # print("set time 1: ", set_time)
            # set_time = pygame.time.get_ticks()
            robot.move_to(angle, win, grid, end)
            # set_time = (pygame.time.get_ticks() - set_time) / 1000
            # print("move time 2: ", set_time)
        # drawNotUpDate(win, grid, ROWS, WIDTH)
        # robot.dt = (pygame.time.get_ticks() - lasttime) / 1000
        # # lasttime = pygame.time.get_ticks()
        # robot.move()
        # robot.draw(win)
        # robot.trail((robot.x, robot.y), win, RED)
        # pygame.display.update()

    return True


def load_map(file_name, grid, name_map):
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
                    grid, start, end = load_map("map.txt", grid, "map1")
                if event.key == pygame.K_2:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map("map.txt", grid, "map2")
                if event.key == pygame.K_3:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map("map.txt", grid, "map3")
                if event.key == pygame.K_4:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map("map.txt", grid, "map4")
                if event.key == pygame.K_5:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    grid, start, end = load_map("map.txt", grid, "map5")
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
                ), "Robot.png", 20)
                if event.key == pygame.K_SPACE and start and end:
                    distance = BFS(grid, end)
                    MARK = find_move_path(robot, lambda: draw(win, grid, ROWS, width),
                                          grid, start, end, win, distance)

                if event.key == pygame.K_c:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()


main(WIN, WIDTH)
