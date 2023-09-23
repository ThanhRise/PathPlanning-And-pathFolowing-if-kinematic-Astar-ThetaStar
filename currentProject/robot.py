
import pygame
import math
from utils import Utils
from spot import Spot
from constant import Constant
from dynamicObstacle import DynamicObstacle
from scipy.spatial import KDTree

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
        self.time_step = 0
        self.W = 0  # angular velocity
        self.a = 15  # width of robot
        self.trail_set = []
        self.dt = 0  # time step
        self.dynamic_obstacles_replan = []
        self.time_replan = 0
        self.replaned = False
        self.pathRb = []
        self.index_path = 0
        self.angle = []
        self.KDTree = None
        self.target = None
        self.distance_to_end = None
        self.distance_to_start = None
        self.distance_to_end_one_way = None
        self.next_pos_move = None
        self.next_angle_move = None
        self.grid_in_vison_robot = []
        one_degree = math.pi / 180
        self.angle.append(0)
        for i in range(0, 121, 10):
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
        if Utils.distance_real((self.x, self.y), target) < 20 and len(self.pathRb) > 1:
            self.pathRb.pop(0)

    def chage_linear_velocity(self, delta_u):
        self.u = self.u + delta_u
        if self.u > self.max_u:
            self.u = self.max_u
        elif self.u < self.min_u:
            self.u = self.min_u

    def move_to(self):
        time_move = pygame.time.get_ticks()
        self.W = self.next_angle_move
        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        # print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < self.time_step and Utils.distance_real((self.x, self.y), self.target) > 10:
            if self.next_angle_move != self.W:
                break
            # Map.draw_not_update()
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            self.theta += (self.vr - self.vl)/self.width * self.dt/(self.time_step/1000)


            # self.theta += self.W / 400

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0


    def find_spot_with_angle(self, grid: list[list[Spot]], angle, time_step, velocity):
        gap = Constant.WIDTH // Constant.ROWS
        dt = time_step / 1000.0
        theta = self.theta + angle
        vr = velocity + (angle * self.width)/2
        vl = velocity - (angle * self.width)/2
        x = self.x + ((vl + vr)/2)*math.cos(theta)*dt
        y = self.y + ((vl + vr)/2)*math.sin(theta)*dt
        if x > Constant.WIDTH or x < 0 or y > Constant.WIDTH or y < 0:
            print("out of range")
            return None, None, None
        row, col = int(x / gap), int(y / gap)
        row2, col2 = int((x + 12) / gap), int(y / gap)
        row3, col3 = int(x / gap), int((y + 12) / gap)
        row4, col4 = int((x + 15) / gap), int((y + 15) / gap)
        row5, col5 = int((x - 15) / gap), int((y - 15) / gap)
        row6, col6 = int((x - 12) / gap), int(y / gap)
        row7, col7 = int(x / gap), int((y - 12) / gap)
        row8, col8 = int((x + 15) / gap), int((y - 15) / gap)
        row9, col9 = int((x - 15) / gap), int((y + 15) / gap)

        spot = grid[row][col]
        global time_global
        time_global = pygame.time.get_ticks()
        if spot.is_barrier() or grid[row2][col2].is_barrier() or grid[row3][col3].is_barrier() or grid[row4][col4].is_barrier() or grid[row5][col5].is_barrier() or grid[row6][col6].is_barrier() or grid[row7][col7].is_barrier() or grid[row8][col8].is_barrier() or grid[row9][col9].is_barrier():
            return None, None, None
        if spot.is_dynamic_obs() or grid[row2][col2].is_dynamic_obs() or grid[row3][col3].is_dynamic_obs() or grid[row4][col4].is_dynamic_obs() or grid[row5][col5].is_dynamic_obs() or grid[row6][col6].is_dynamic_obs() or grid[row7][col7].is_dynamic_obs() or grid[row8][col8].is_dynamic_obs() or grid[row9][col9].is_dynamic_obs():
            return None, None, None
        return spot, x, y

    def find_next_spot(self):
        current = self.x, self.y
        gap = Constant.WIDTH // Constant.ROWS
        # the next point will be based on velocity, time and angle
        if current != self.target:
            neighbors = []
            for i in range(len(self.angle)):
                for j in range(10, 16, 5):
                    for k in range(10, 16, 6):
                        time_step = 1000 + j * 100
                        velocity = 5 + k
                        # time_step = 1500
                        # velocity = 15
                        spot, x, y = self.find_spot_with_angle(
                            self.grid_in_vison_robot, self.angle[i], time_step, velocity)
                        if spot is not None:
                            neighbors.append(
                                (spot, x, y, self.angle[i], time_step, velocity))

            if len(neighbors) > 0:
                # find the spot with the smallest distance to the target
                min_spot = None
                obstacle_dist = 0
                f_cost_best = 100000000
                angle_selected = self.next_angle_move
                # print('neighbors: ', len(neighbors))
                limit = Utils.distance_real((self.x, self.y), self.target)
                for i in range(len(neighbors)):

                    dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), self.target)

                    dist2 = self.distance_to_end[neighbors[i][0]]
                    
                    nearest_obstacle_dist, _ = self.KDTree.query([(neighbors[i][1], neighbors[i][2])])

                    nearest_core = 1 / (nearest_obstacle_dist + 1)
                    # f_cost = nearest_core * 30 + dist
                    if limit < 80:  
                        f_cost = dist
                    else:
                        f_cost = dist2 + nearest_core * 50 *(1 - 10/limit)
                    
                    if f_cost < f_cost_best:
                        f_cost_best = f_cost
                        min_spot = neighbors[i][1], neighbors[i][2]
                        angle_selected = neighbors[i][3]
                        obstacle_dist = nearest_obstacle_dist
                        self.time_step = neighbors[i][4]
                        self.u = neighbors[i][5]


                # find the spot with the smallest distance to the target
                # min_spot = None
                # obstacle_dist = 0
                # f_cost_best = 1000000
                # for i in range(len(neighbors)):

                #     dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), target)

                #     dist2 = self.distance_to_end[neighbors[i][0]]
                    
                #     nearest_obstacle_dist, _ = kdTree.query([(neighbors[i][1], neighbors[i][2])])

                #     nearest_core = 1 / (nearest_obstacle_dist + 1)
                #     f_cost = nearest_core * 30 + dist

                #     # f_cost = dist  + dist2 *20 
                #     # print('f_cost: ', f_cost, 'nearest_obstacle_dist: ', nearest_obstacle_dist, 'dist: ', dist, 'time step: ', neighbors[i][4], 'dist2', dist2)
                #     if f_cost < f_cost_best:

                #         f_cost_best = f_cost
                #         min_spot = neighbors[i][1], neighbors[i][2]
                #         # min_spot = neighbors[i][0]
                #         angle_selected = neighbors[i][3]
                #         obstacle_dist = nearest_obstacle_dist
                #         self.time_step = neighbors[i][4]
                #         self.u = neighbors[i][5]

                # # print('time step: ', self.time_step, 'velocity: ', self.u)
                return min_spot, angle_selected, obstacle_dist
            else:
                return None, None, None

    def move_back(self,Map):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        vr_prev = self.vr
        vl_prev = self.vl
        self.vr = -self.u / 2
        self.vl = -self.u / 2
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 2000:
            # Map.draw_not_update()
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0

            # self.rotated = pygame.transform.rotozoom(
            #     self.img, math.degrees(-self.theta), 1)
            # self.rect = self.rotated.get_rect(center=(self.x, self.y))
            # self.draw(Map.WIN)
            # self.trail((self.x, self.y), Map.WIN, Constant.GREEN)
            # pygame.display.update()

        self.vr = vr_prev
        self.vl = vl_prev


    def detect_obstacle(self, MAP):
        step_horizontal = MAP.GAP / 2 

        # reset dynamic obstacles
        for row in range(MAP.ROWS):
            for col in range(MAP.ROWS):
                if self.grid_in_vison_robot[row][col].is_dynamic_obs():
                    self.grid_in_vison_robot[row][col].reset()

        for row in range(MAP.ROWS):
            for col in range(MAP.ROWS):
                if Utils.distance_real((self.x, self.y), self.grid_in_vison_robot[row][col].get_real_pos(MAP.GAP)) < 100:
                    if MAP.grid[row][col].is_barrier():
                        self.grid_in_vison_robot[row][col].make_barrier()

        # dynamic obstacles
        for obs in self.dynamic_obstacles_replan:
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
                if Utils.distance_real((x_current, y_current), (self.x, self.y)) > 24:
                    temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)].make_dynamic_obs()
                    temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                    if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                        self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)].make_dynamic_obs()
                
                x_current += 16 * math.cos(obs.theta)
                y_current += 16 * math.sin(obs.theta)

        obstacle = [spot.get_real_pos(MAP.GAP)
            for row in MAP.grid for spot in row if spot.is_barrier() or spot.is_dynamic_obs()]
        self.KDTree = KDTree(obstacle)    


        

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def trail(self, pos, map, color):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
        self.trail_set.append(pos)