import pygame
import math
from utils import Utils
from constant import Constant


class Spot:
    def __init__(self, row, col, grap, total_rows):
        self.row = row
        self.col = col
        self.x = row * grap
        self.y = col * grap
        self.color = Constant.WHITE
        self.neighbors = []
        self.neighbors_distance = []
        self.neighbors_distance_angleRobot = []
        self.grap = grap
        self.total_rows = total_rows
        self.prev_color = None  # Initialize prev_color to None

    def get_pos(self):
        return self.row, self.col

    def get_real_pos(self, gap):
        x = gap * self.row + 8   # để đường thẳng nằm giữa ô vuông
        y = gap * self.col + 8
        return x, y

    def is_closed(self):
        return self.color == Constant.RED

    def is_open(self):
        return self.color == Constant.GREEN

    def is_barrier(self):
        return self.color == Constant.BLACK

    def is_start(self):
        return self.color == Constant.ORANGE

    def is_end(self):
        return self.color == Constant.TURQUOISE

    def is_dynamic_obs(self):
        return self.color == Constant.GREY
    
    def is_future_dynamic_obs(self):
        return self.color == Constant.SUP_GREY

    def reset(self):
        self.color = Constant.WHITE

    def make_start(self):
        self.color = Constant.ORANGE

    def make_closed(self):
        self.color = Constant.RED

    def make_open(self):
        self.color = Constant.GREEN

    def make_barrier(self):
        self.prev_color = self.color
        self.color = Constant.BLACK

    def make_end(self):
        self.color = Constant.TURQUOISE

    def make_path(self):
        self.color = Constant.PURPLE

    def make_theta(self):
        self.color = Constant.YELLOW

    def reset_to_prev_color(self):
        if self.prev_color is not None:
            self.color = self.prev_color
            self.prev_color = None

    def make_dynamic_obs(self):
        self.prev_color = self.color  # save current color as prev_color
        self.color = Constant.GREY

    def make_future_dynamic_obs(self):
        self.prev_color = self.color
        self.color = Constant.SUP_GREY

    def draw(self, win, color=None):
        if color is None:
            pygame.draw.rect(win, self.color, (self.x, self.y, self.grap, self.grap))
        else:   
            pygame.draw.rect(win, color, (self.x, self.y, self.grap, self.grap))

    # update the neighbors for BFS

    def update_neighbors_distance(self, grid):
        self.neighbors_distance = []
        distance_diagonal = math.sqrt(2)
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()\
                and ((not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()) or (not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs())):
            self.neighbors_distance.append((grid[self.row + 1][self.col], 1))

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()\
                and ((not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs())):
            self.neighbors_distance.append((grid[self.row - 1][self.col], 1))

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs()\
                and ((not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs())):
            self.neighbors_distance.append((grid[self.row][self.col + 1], 1))

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs()\
                and ((not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs())):
            self.neighbors_distance.append((grid[self.row][self.col - 1], 1))

        if (self.row < self.total_rows - 1) and (self.col < self.total_rows - 1) and (not grid[self.row + 1][self.col + 1].is_barrier()) and (not grid[self.row + 1][self.col + 1].is_dynamic_obs()) and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
            self.neighbors_distance.append(
                (grid[self.row + 1][self.col + 1], distance_diagonal))

        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs()\
                and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
            self.neighbors_distance.append(
                (grid[self.row - 1][self.col + 1], distance_diagonal))

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()\
                and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
            self.neighbors_distance.append(
                (grid[self.row + 1][self.col - 1], distance_diagonal))

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs()\
                and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
            self.neighbors_distance.append(
                (grid[self.row - 1][self.col - 1], distance_diagonal))

    def update_neighbors_with_angle(self, grid, angle):
        # 10 is right
        # 01 is left
        # 11 is down
        # 00 is up
        distance_diagonal = math.sqrt(2)
        angle = angle
        if angle == "10": # only move right, right down, right up
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs()\
                and ((not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs())):   
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row][self.col + 1], 1, "10")) # right

            if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row + 1][self.col + 1], distance_diagonal, "11"))   

            if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs()\
                    and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row - 1][self.col + 1], distance_diagonal, "00"))         
            
        elif angle == "01": # only move left, left down, left up
            if self.col > 0 and not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append((grid[self.row][self.col - 1], 1, "01")) # left

            if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row + 1][self.col - 1], distance_diagonal, "11"))
                
            if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row - 1][self.col - 1], distance_diagonal, "00"))
                
        elif angle == "11": # only move down, down left, down right
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()) or (not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append((grid[self.row + 1][self.col], 1, "11")) # down

            if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier() and not grid[self.row + 1][self.col + 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row + 1][self.col + 1], distance_diagonal, "10"))
                
            if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col].is_barrier() and not grid[self.row + 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row + 1][self.col - 1], distance_diagonal, "01"))
                
        elif angle == "00": # only move up, up left, up right
            if self.col > 0 and not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row + 1][self.col - 1].is_barrier() and not grid[self.row + 1][self.col - 1].is_dynamic_obs()) or (not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append((grid[self.row][self.col - 1], 1, "00")) # up

            if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier() and not grid[self.row - 1][self.col - 1].is_dynamic_obs()\
                    and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col - 1].is_barrier() and not grid[self.row][self.col - 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row - 1][self.col - 1], distance_diagonal, "01"))
                
            if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier() and not grid[self.row - 1][self.col + 1].is_dynamic_obs()\
                    and ((not grid[self.row - 1][self.col].is_barrier() and not grid[self.row - 1][self.col].is_dynamic_obs()) or (not grid[self.row][self.col + 1].is_barrier() and not grid[self.row][self.col + 1].is_dynamic_obs())):
                self.neighbors_distance_angleRobot.append(
                    (grid[self.row - 1][self.col + 1], distance_diagonal, "10"))


        pass

    def __lt__(self, other):
        return False
