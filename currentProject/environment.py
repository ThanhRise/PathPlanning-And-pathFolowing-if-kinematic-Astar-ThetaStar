import pygame
from utils import Utils
from constant import Constant
import math
from spot import Spot


class Environment:
    def __init__(self, WIDTH, ROWS) -> None:
        self.WIDTH = WIDTH
        self.ROWS = ROWS
        self.GAP = self.WIDTH // self.ROWS
        self.WIN = pygame.display.set_mode((self.WIDTH, self.WIDTH))
        self.MAP_NAME = None
        pygame.display.set_caption("Kinematic model with KD-Tree")
        self.grid = []
        for i in range(self.ROWS):
            self.grid.append([])
            for j in range(self.ROWS):
                spot = Spot(i, j, self.GAP, self.ROWS)
                self.grid[i].append(spot)
        
        self.voronoi = None
        self.start = None
        self.end = None
        self.path = []
        self.open_set = []
        self.closed_set = []
        self.obstacles = []
        self.dynamic_obs = []


    def draw_grid(self):
        gap = self.GAP
        # for i in range(self.ROWS):
        #     pygame.draw.line(self.WIN, Constant.GREY,(0, i * gap), (self.WIDTH, i * gap))
        #     for j in range(self.ROWS):
        #         pygame.draw.line(self.WIN, Constant.GREY,(j * gap, 0), (j * gap, self.WIDTH))
        pass

    def draw(self):
        self.WIN.fill(Constant.WHITE)
        for row in range(self.ROWS):
            for spot in range(self.ROWS):
                self.grid[row][spot].draw(self.WIN)
        self.draw_grid()
        pygame.display.update()

    def get_clicked_pos(self, pos):
        gap = self.GAP
        y, x = pos
        row = y // gap
        col = x // gap
        return row, col
    
    def make_barrier_edge(self):
        for i in range(self.ROWS):
            for j in range(self.ROWS):
                if i == 0 or j == 0 or i == self.ROWS - 1 or j == self.ROWS - 1:
                    self.grid[i][j].make_barrier()
                    self.obstacles.append(self.grid[i][j])
    
    def draw_not_update(self, grid = None):
        if grid == None:
            grid = self.grid
        self.WIN.fill(Constant.WHITE)
        for row in range(self.ROWS):
            for spot in range(self.ROWS):
                grid[row][spot].draw(self.WIN)
        self.draw_grid()

    def draw_not_update_check_obs(self, grid = None, grid2 = None):
        if grid == None:
            grid = self.grid
        self.WIN.fill(Constant.WHITE)
        for row in range(self.ROWS):
            for spot in range(self.ROWS):
                if grid2[row][spot].is_barrier():
                    if grid[row][spot].is_barrier():
                        grid[row][spot].draw(self.WIN)
                    else:
                        grid[row][spot].draw(self.WIN, Constant.WHITE)
                else:
                    grid[row][spot].draw(self.WIN)
        
        self.draw_grid()
    
