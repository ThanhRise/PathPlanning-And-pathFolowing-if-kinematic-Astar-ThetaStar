import pygame
import math
from constant import Constant

class DynamicObstacle:

    def __init__(self, x, y, d, theta, velocity):
        self.x = x
        self.y = y
        self.d = d
        # change theta to radian
        self.theta = math.radians(theta)
        self.velocity = velocity

    def move(self, grid, dt):
        x = self.x + self.velocity * math.cos(self.theta) * dt
        y = self.y + self.velocity * math.sin(self.theta) * dt
        if x < 0:
            x = - x
            self.theta = math.pi - self.theta
        if x > 800 - 1:
            x = 800 - 1 - (x - 800 + 1)
            self.theta = math.pi - self.theta
        if y < 0:
            y = - y
            self.theta = - self.theta
        if y > 800 - 1:
            y = 800 - 1 - (y - 800 + 1)
            self.theta = - self.theta
        # change theta when x, y is on the barrier
        if self.theta < 0:
            self.theta = self.theta + math.pi * 2
        if self.theta >= math.pi * 2:
            self.theta = self.theta - math.pi * 2
        theta = self.theta
        
        if grid[int(x / 16)][int(y / 16)].is_barrier() or grid[int((x + self.d) / 16)][int(y / 16)].is_barrier() or grid[int(x / 16)][int((y + self.d) / 16)].is_barrier() or grid[int((x + self.d) / 16)][int((y + self.d) / 16)].is_barrier() \
            or grid[int((x - self.d)/ 16)][int(y / 16)].is_barrier() or grid[int(x / 16)][int((y - self.d) / 16)].is_barrier() or grid[int((x - self.d) / 16)][int((y - self.d) / 16)].is_barrier() or grid[int((x + self.d) / 16)][int((y - self.d) / 16)].is_barrier() or grid[int((x - self.d) / 16)][int((y + self.d) / 16)].is_barrier():
            if self.theta > 0 and self.theta < math.pi / 2:
                theta = math.pi - self.theta
                if grid[int(x /16)][int((y+ self.d) / 16)].is_barrier() or grid[int((x - self.d)/16)][int((y + self.d) / 16)].is_barrier() :
                # if grid[int(x /16)][int((y+ self.d) / 16)].is_barrier():
                    theta =  - self.theta
            
            if self.theta > math.pi / 2 and self.theta < math.pi:
                theta = math.pi - self.theta 
                if grid[int(x / 16)][int((y + self.d)/16)].is_barrier() or grid[int((x + self.d) / 16)][int((y + self.d)/16)].is_barrier():
                # if grid[int(x /16)][int((y+ self.d) / 16)].is_barrier():
                    theta =  - self.theta   
            
            if self.theta > math.pi and self.theta < math.pi * 3 / 2:
                theta = math.pi - self.theta
                if grid[int((x) / 16)][int((y - self.d)/16)].is_barrier() or grid[int((x + self.d) / 16)][int((y - self.d)/16)].is_barrier():
                # if grid[int((x) / 16)][int((y - self.d)/16)].is_barrier():
                    theta = - self.theta
                
            if self.theta > math.pi * 3 / 2 and self.theta < math.pi * 2:
                theta = math.pi - self.theta
                if grid[int((x)/16)][int((y- self.d) / 16)].is_barrier() or grid[int((x - self.d)/16)][int((y -self.d) / 16)].is_barrier():
                # if grid[int((x) / 16)][int((y - self.d)/16)].is_barrier():
                    theta = - self.theta

            if self.theta == math.pi or self.theta == 0:
                theta = math.pi - self.theta

            if self.theta == math.pi / 2 or self.theta == math.pi * 3 / 2:
                theta = - self.theta

        if self.theta == theta:
            self.x = x
            self.y = y
        else:
            self.theta = theta
            self.x = self.x + self.velocity * math.cos(self.theta) * dt
            self.y = self.y + self.velocity * math.sin(self.theta) * dt

    def draw(self, win):
        pygame.draw.circle(win, Constant.PURPLE, (int(self.x), int(self.y)), self.d, 100)