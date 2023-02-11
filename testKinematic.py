import pygame
import math
from queue import PriorityQueue

WIDTH = 800
ROWS = 50
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Theta* Path Finding Algorithm")

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
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def get_real_pos(self):
        gap = WIDTH // ROWS
        x = gap * self.row + 8  # để đường thẳng nằm giữa ô vuông
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

    def update_neighbors(self, grid):
        self.neighbors = []
        # DOWN
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        # RIGHT
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

        # DOWN RIGHT
        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col + 1])

        # UP RIGHT
        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col + 1])

        # DOWN LEFT
        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col - 1])

        # UP LEFT
        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col - 1])

    def __lt__(self, other):
        return False


class Robot:
    def __init__(self, startPos, robotImg, width):

        self.x, self.y = startPos
        self.theta = 0
        self.width = width
        self.vr = 30
        self.vl = 30
        self.u = 20
        self.W = 0
        self.a = 15
        self.trail_set = []
        self.dt = 0
        self.pathRb = []
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

        self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
        self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
        self.theta += (self.vr - self.vl)/self.width*self.dt

        # self.x += (self.width*(self.vr + self.vl)*(math.sin(self.theta + self.theta*self.dt) -  math.sin(self.theta) ))/ (2*(self.vr -self.vl))
        # self.y += (self.width*(self.vr + self.vl)*(- math.cos(self.theta + self.theta*self.dt) +  math.cos(self.theta) ))/ (2*(self.vr -self.vl))
        # self.theta += self.theta*self.dt

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

        self.vr = (self.u + self.W*self.width)/2
        self.vl = (self.u - self.W * self.width)/2
        if self.dist((self.x, self.y), target) < 10 and len(self.pathRb) > 1:
            self.pathRb.pop(0)

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
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 5)
        self.trail_set.append(pos)


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)
    # if abs(x1 - x2) >= abs(y1 - y2):
    #     return abs(x1 - x2)
    # if abs(x1 - x2) < abs(y1 - y2):
    #     return abs(y1 - y2)


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


def algorithm(draw, grid, start, end, win):  # theta Star
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
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        temp = open_set.get()
        x = temp[1]
        current = temp[2]
        open_set_hash.remove(current)

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if current != start and LineOfSight(came_from[current].get_pos(), neighbor.get_pos(), grid):

                if g_score[came_from[current]] + h(came_from[current].get_pos(), neighbor.get_pos()) < g_score[neighbor]:

                    g_score[neighbor] = g_score[came_from[current]] + \
                        h(came_from[current].get_pos(), neighbor.get_pos())
                    f_score[neighbor] = g_score[current] + \
                        h(neighbor.get_pos(), end.get_pos())
                    came_from[neighbor] = came_from[current]

                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        neighbor.make_theta()
            elif temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + \
                    h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

            if current == end:
                global path
                while current in came_from:
                    path.append(current.get_real_pos())
                    current = came_from[current]
                path.append(start.get_real_pos())
                reconstruct_path(came_from, end, draw)
                end.make_end()
                draw()
                reconstruct_line_path(came_from, end, win)
                pygame.display.update()
                return True

        draw()
        if current != start:
            current.make_closed()

    return False


def LineOfSight(p1, p2, gird):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    f = 0

    if dy < 0:
        dy = -dy
        sy = -1
    else:
        sy = 1
    if dx < 0:
        dx = -dx
        sx = -1
    else:
        sx = 1
    if dx >= dy:
        while x1 != x2:
            f = f + dy
            if f >= dx:
                if gird[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                    return False
                y1 = y1 + sy
                f = f - dx

            if f != 0 and gird[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                return False

            if (dy == 0 and gird[x1 + ((sx - 1) // 2)][y1].is_barrier()) or (dy == 0 and gird[x1 + ((sx - 1) // 2)][y1 + 1].is_barrier()):

                return False

            x1 = x1 + sx
    else:
        while y1 != y2:
            f = f + dx
            if f >= dy:
                if gird[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                    return False
                x1 = x1 + sx
                f = f - dy

            if f != 0 and gird[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier():
                return False

            if (dx == 0 and gird[x1][y1 + ((sy - 1) // 2)].is_barrier()) or (dx == 0 and gird[x1 + 1][y1 + ((sy - 1) // 2)].is_barrier()):
                return False

            y1 = y1 + sy

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
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    MARK = algorithm(lambda: draw(win, grid, ROWS, width),
                                     grid, start, end, win)

                if event.key == pygame.K_c:
                    MARK = False
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
        if MARK:
            robot = Robot(start.get_real_pos(
            ), "C:\\Users\\admin\\LearningIT\\20221\\Project1\\kinematic\\Robot.png", 20)

            path.reverse()
            robot.pathRb = path

            lasttime = pygame.time.get_ticks()
            while run:

                reset = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        run = False
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_SPACE:
                            reset = True
                            robot.pathRb.clear()

                if reset:
                    MARK = False
                    break
                # timesleep += (pygame.time.get_ticks() - lasttime) / 1000
                # if timesleep > 0.1:
                drawNotUpDate(win, grid, ROWS, width)
                # timesleep = 0
                robot.dt = (pygame.time.get_ticks() - lasttime) / 1000
                lasttime = pygame.time.get_ticks()
                if h((robot.x, robot.y), end.get_real_pos()) > 5:
                    robot.move(event=event)
                robot.draw(win)
                robot.trail((robot.x, robot.y), win, RED)
                pygame.display.update()

    pygame.quit()


main(WIN, WIDTH)
