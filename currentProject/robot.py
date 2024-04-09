
import pygame
import math
from utils import Utils
from spot import Spot
from constant import Constant
from dynamicObstacle import DynamicObstacle
from scipy.spatial import KDTree
from scipy.spatial import Voronoi, voronoi_plot_2d
from algorithm import *

class Robot:
    def __init__(self, startPos, robotImg, width):

        self.x, self.y = startPos
        self.theta = 0
        self.width = width
        self.vr = 15  # right velocity
        self.vl = 15  # left velocity
        self.u = (self.vl + self.vr)/2  # linear velocity
        self.max_u = 20
        self.min_u = 5
        self.time_step = 0
        self.W = 0  # angular velocity
        # self.a = 15  # width of robot
        self.trail_set = []
        self.dt = 0  # time step
        self.dynamic_obstacles_replan = []
        self.time_replan = 0
        self.replaned = False
        self.pathRb = []
        self.pathVoronoi = []
        self.index_path = 0
        self.angle = []
        self.KDTree = None
        self.segments = None
        self.target = None
        self.backwards = False
        self.distance_to_end = None
        self.distance_to_start = None
        self.distance_to_end_one_way = None
        self.next_pos_move = None
        self.next_angle_move = None
        self.grid_in_vison_robot = []
        self.obstecls = []
        self.near_obstacle = []
        one_degree = math.pi / 180
        self.angle.append(0)
        for i in range(5, 91 , 9):
            self.angle.append(i * one_degree)
            self.angle.append(-i * one_degree)
        self.num_collision = 0
        self.img = pygame.image.load(robotImg)
        self.img = pygame.transform.scale(self.img, (self.width, self.width))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def move(self, event=None):
        
        self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
        self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
        self.theta += (self.vr - self.vl)/self.width*self.dt
        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        # self.following()

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
        self.W = self.next_angle_move
        if self.W == None:
            return
        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        # print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        time_move = pygame.time.get_ticks()
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < self.time_step and Utils.distance_real((self.x, self.y), self.target) > 5:
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
            self.rotated = pygame.transform.rotozoom(self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center=(self.x, self.y))

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0


            

    def find_spot_with_angle(self, grid: list[list[Spot]], angle, time_step, velocity, is_near_obstacle=False ):
        gap = Constant.GAP
        gap_2 = 0
        gap_3 = 0
        if is_near_obstacle:
            gap_2 = gap / 2
            gap_3 = gap / 2
        else:
            gap_2 = gap * 2 / 3
            gap_3 = gap * 2 / 3
        dt = time_step / 1000.0

        theta = self.theta + angle
        vr = velocity + (angle * self.width)/2
        vl = velocity - (angle * self.width)/2

        x_front_midle = self.x + ((vr + vl)/2)*math.cos(self.theta)*dt/2
        y_front_midle = self.y + ((vr + vl)/2)*math.sin(self.theta)*dt/2

        row_front_midle, col_front_midle = int(x_front_midle / gap), int(y_front_midle / gap)
        spot_front_midle = grid[row_front_midle][col_front_midle]
        if spot_front_midle.is_barrier() or spot_front_midle.is_dynamic_obs():
            return None, None, None



        x = self.x + ((vl + vr)/2)*math.cos(theta)*dt
        y = self.y + ((vl + vr)/2)*math.sin(theta)*dt
        if x > Constant.WIDTH or x < 0 or y > Constant.WIDTH or y < 0:
            print("out of range")
            return None, None, None
        
        # x_next = x + 16 * math.cos(theta)
        # y_next = y + 16 * math.sin(theta)
        # row_next, col_next = int(x_next / gap), int(y_next / gap)
        # if grid[row_next][col_next].is_barrier() or grid[row_next][col_next].is_dynamic_obs():
        #     return None, None, None

        
        x_midle = (self.x + x) / 2
        y_midle = (self.y + y) / 2
        row_midle, col_midle = int(x_midle / gap), int(y_midle / gap)
        if grid[row_midle][col_midle].is_barrier() or grid[row_midle][col_midle].is_dynamic_obs():
            return None, None, None

        row, col = int(x / gap), int(y / gap)
        if grid[row][col].is_barrier() or grid[row][col].is_dynamic_obs():
            return None, None, None
        row2, col2 = int((x + gap_2) / gap), int(y / gap)
        if grid[row2][col2].is_barrier() or grid[row2][col2].is_dynamic_obs():
            return None, None, None
        row3, col3 = int(x / gap), int((y + gap_2) / gap)
        if grid[row3][col3].is_barrier() or grid[row3][col3].is_dynamic_obs():
            return None, None, None
        row4, col4 = int((x + gap_2) / gap), int((y + gap_2) / gap)
        if grid[row4][col4].is_barrier() or grid[row4][col4].is_dynamic_obs():
            return None, None, None
        row5, col5 = int((x - gap_2) / gap), int((y - gap_2) / gap)
        if grid[row5][col5].is_barrier() or grid[row5][col5].is_dynamic_obs():
            return None, None, None
        row6, col6 = int((x - gap_2) / gap), int(y / gap)
        if grid[row6][col6].is_barrier() or grid[row6][col6].is_dynamic_obs():
            return None, None, None
        row7, col7 = int(x / gap), int((y - gap_2) / gap)
        if grid[row7][col7].is_barrier() or grid[row7][col7].is_dynamic_obs():
            return None, None, None
        row8, col8 = int((x + gap_2) / gap), int((y - gap_2) / gap)
        if grid[row8][col8].is_barrier() or grid[row8][col8].is_dynamic_obs():
            return None, None, None
        row9, col9 = int((x - gap_2) / gap), int((y + gap_2) / gap)
        if grid[row9][col9].is_barrier() or grid[row9][col9].is_dynamic_obs():
            return None, None, None

        

        x_midle_font_midle = (x_midle + x_front_midle) / 2
        y_midle_font_midle = (y_midle + y_front_midle) / 2
        row_midle_font_midle, col_midle_font_midle = int(x_midle_font_midle / gap), int(y_midle_font_midle / gap)
        if grid[row_midle_font_midle][col_midle_font_midle].is_barrier() or grid[row_midle_font_midle][col_midle_font_midle].is_dynamic_obs():
            return None, None, None

        row_M_M_2, col_M_M_2 = int((x_midle_font_midle + gap_3) / gap), int(y_midle_font_midle / gap)
        if grid[row_M_M_2][col_M_M_2].is_barrier() or grid[row_M_M_2][col_M_M_2].is_dynamic_obs():
            return None, None, None
        row_M_M_3, col_M_M_3 = int((x_midle_font_midle - gap_3) / gap), int(y_midle_font_midle / gap)
        if grid[row_M_M_3][col_M_M_3].is_barrier() or grid[row_M_M_3][col_M_M_3].is_dynamic_obs():
            return None, None, None
        row_M_M_4, col_M_M_4 = int(x_midle_font_midle / gap), int((y_midle_font_midle + gap_3) / gap)
        if grid[row_M_M_4][col_M_M_4].is_barrier() or grid[row_M_M_4][col_M_M_4].is_dynamic_obs():
            return None, None, None
        row_M_M_5, col_M_M_5 = int(x_midle_font_midle / gap ), int((y_midle_font_midle - gap_3) / gap)
        if grid[row_M_M_5][col_M_M_5].is_barrier() or grid[row_M_M_5][col_M_M_5].is_dynamic_obs():
            return None, None, None
        row_M_M_6, col_M_M_6 = int((x_midle_font_midle + gap_3) / gap), int((y_midle_font_midle + gap_3) / gap)
        if grid[row_M_M_6][col_M_M_6].is_barrier() or grid[row_M_M_6][col_M_M_6].is_dynamic_obs():
            return None, None, None
        row_M_M_7, col_M_M_7 = int((x_midle_font_midle - gap_3) / gap), int((y_midle_font_midle - gap_3) / gap)
        if  grid[row_M_M_7][col_M_M_7].is_barrier() or grid[row_M_M_7][col_M_M_7].is_dynamic_obs():
            return None, None, None
        row_M_M_8, col_M_M_8 = int((x_midle_font_midle + gap_3) / gap), int((y_midle_font_midle - gap_3) / gap)
        if grid[row_M_M_8][col_M_M_8].is_barrier() or grid[row_M_M_8][col_M_M_8].is_dynamic_obs():
            return None, None, None
        row_M_M_9, col_M_M_9 = int((x_midle_font_midle - gap_3) / gap), int((y_midle_font_midle + gap_3) / gap)
        if grid[row_M_M_9][col_M_M_9].is_barrier() or grid[row_M_M_9][col_M_M_9].is_dynamic_obs():
            return None, None, None

        spot = grid[row][col]
        return spot, x, y

    def find_next_spot(self, MAP, back=False, is_algorithm="KDTree"):
        current = self.x, self.y
        if is_algorithm == "KDTree_non_vision" or is_algorithm == "Voronoi_non_vision":
            spot_current = self.grid_in_vison_robot[int(self.x // MAP.GAP)][int(self.y // MAP.GAP)]
        else:
            spot_current = MAP.grid[int(self.x // MAP.GAP)][int(self.y // MAP.GAP)]
        u_min = 0
        u_max = 0
        sety_to_obstacle = 0
        is_near_obstacle = False
        if self.near_obstacle[spot_current] == 1:
            sety_to_obstacle = 50
            u_min = 5
            u_max = 12
            is_near_obstacle = True
        else:
            u_min = 10
            u_max = 20
            sety_to_obstacle = 150
            is_near_obstacle = False
        # the next point will be based on velocity, time and angle
        if current != self.target:
            neighbors = []
            for i in range(len(self.angle)):
                for j in range(10, 17, 5):
                    for k in range(u_min, u_max, 8):
                        time_step = 200 + j * 100
                        velocity = 5 + k
                        # time_step = 1500
                        # velocity = 15

                        if is_algorithm == "KDTree_non_vision":
                            spot, x, y = self.find_spot_with_angle(
                                self.grid_in_vison_robot, self.angle[i], time_step, velocity, is_near_obstacle)
                        if is_algorithm == "Voronoi_non_vision":
                            spot, x, y = self.find_spot_with_angle(
                                self.grid_in_vison_robot, self.angle[i], time_step, velocity)
                        if is_algorithm == "KDTree vision" or is_algorithm == "Voronoi vision":
                            spot, x, y = self.find_spot_with_angle(
                                MAP.grid , self.angle[i], time_step, velocity)

                        if spot is not None:
                            neighbors.append(
                                (spot, x, y, self.angle[i], time_step, velocity))

            if len(neighbors) > 0:
                time_step_re = 0
                angle_selected = 0
                u = 0
                # kd tree - non vison
                if is_algorithm == "KDTree_non_vision":
                    if back == False:
                        min_spot = None
                        obstacle_dist = 0
                        f_cost_best = 100000000

                        # print('neighbors: ', len(neighbors))
                        limit = Utils.distance_real((self.x, self.y), self.target)
                        for i in range(len(neighbors)):
                            dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), self.target)
                            dist2 = self.distance_to_end[neighbors[i][0]]
                            nearest_obstacle_dist, _ = self.KDTree.query([(neighbors[i][1], neighbors[i][2])])
                            nearest_core = 1 / (nearest_obstacle_dist + 1)
                            # f_cost = nearest_core * 30 + dist
                            if limit < 30:
                                f_cost = dist
                            else:
                                f_cost = dist2 + nearest_core * sety_to_obstacle * (1- 30/limit) 
                            if f_cost < f_cost_best:
                                f_cost_best = f_cost
                                min_spot = neighbors[i][1], neighbors[i][2]
                                angle_selected = neighbors[i][3]
                                obstacle_dist = nearest_obstacle_dist
                                time_step_re = neighbors[i][4]
                                u = neighbors[i][5]
                    else:
                        min_spot = None
                        obstacle_dist = 0
                        f_cost_best = 0
                        for i in range(len(neighbors)):
                            dist, _ = self.KDTree.query([(neighbors[i][1], neighbors[i][2])])
                            f_cost = dist
                            if f_cost > f_cost_best:
                                f_cost_best = f_cost
                                min_spot = neighbors[i][1], neighbors[i][2]
                                angle_selected = neighbors[i][3]
                                time_step_re = neighbors[i][4]
                                u = neighbors[i][5]

                # KD tree - all map
                if is_algorithm == "KDTree vision":
                    min_spot = None
                    angle_selected = None
                    obstacle_dist = 0
                    f_cost_best = 1000000
                    for i in range(len(neighbors)):

                        dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), self.target)

                        dist2 = self.distance_to_end[neighbors[i][0]]

                        nearest_obstacle_dist, _ = self.KDTree.query([(neighbors[i][1], neighbors[i][2])])

                        nearest_core = 1 / (nearest_obstacle_dist + 1)
                        f_cost = nearest_core * 30 + dist + dist2 * 10

                        # f_cost = dist

                        # f_cost = dist  + dist2 *20
                        # print('f_cost: ', f_cost, 'nearest_obstacle_dist: ', nearest_obstacle_dist, 'dist: ', dist, 'time step: ', neighbors[i][4], 'dist2', dist2)
                        if f_cost < f_cost_best:

                            f_cost_best = f_cost
                            min_spot = neighbors[i][1], neighbors[i][2]
                            # min_spot = neighbors[i][0]
                            angle_selected = neighbors[i][3]
                            # obstacle_dist = nearest_obstacle_dist
                            self.time_step = neighbors[i][4]
                            self.u = neighbors[i][5]

                # voronoi - non vison
                if is_algorithm == "Voronoi_non_vision" or is_algorithm == "Voronoi vision":
                    if back == False:
                        min_spot = None
                        obstacle_dist = 0
                        f_cost_best = 1000000
                        for i in range(len(neighbors)):

                            dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), self.target)
                            f_cost = dist
                            if f_cost < f_cost_best:

                                f_cost_best = f_cost
                                min_spot = neighbors[i][1], neighbors[i][2]
                                angle_selected = neighbors[i][3]
                                time_step_re = neighbors[i][4]
                                u = neighbors[i][5]
                    else:
                        min_spot = None
                        obstacle_dist = 0
                        f_cost_best = 1000000
                        for i in range(len(neighbors)):

                            dist = Utils.distance_real((neighbors[i][1], neighbors[i][2]), self.target)
                            f_cost = dist 
                            if f_cost < f_cost_best:
                                f_cost_best = f_cost
                                min_spot = neighbors[i][1], neighbors[i][2]
                                angle_selected = neighbors[i][3]
                                time_step_re = neighbors[i][4]
                                u = neighbors[i][5]



                self.u = u
                self.time_step = time_step_re
                # print('time step: ', self.time_step, 'velocity: ', self.u)
                return min_spot, angle_selected, obstacle_dist
            else:
                return None, None, None

    def move_back(self, Map):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        vr_prev = self.vr
        vl_prev = self.vl
        self.vr = -self.u / 2
        self.vl = -self.u / 2
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 1000:
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
        
        flag = False
        # reset dynamic obstacles
        for row in range(MAP.ROWS):
            for col in range(MAP.ROWS):
                if self.grid_in_vison_robot[row][col].is_dynamic_obs() or self.grid_in_vison_robot[row][col].is_future_dynamic_obs():
                    self.grid_in_vison_robot[row][col].reset()

        for row in range(MAP.ROWS):
            for col in range(MAP.ROWS):
                if Utils.distance_real((self.x, self.y), self.grid_in_vison_robot[row][col].get_real_pos(MAP.GAP)) < 100:
                    if MAP.grid[row][col].is_barrier() and not self.grid_in_vison_robot[row][col].is_barrier():
                        self.grid_in_vison_robot[row][col].make_barrier()
                        flag = True

        # dynamic obstacles
        for obs in self.dynamic_obstacles_replan:
            step_horizontal = obs.d *2 / 3
            x_next = obs.x + 4 * obs.velocity * math.cos(obs.theta)
            y_next = obs.y + 4 * obs.velocity * math.sin(obs.theta)
            if x_next <= Constant.GAP:
                x_next = Constant.GAP+2
            if x_next >= Constant.WIDTH - Constant.GAP:
                x_next = Constant.WIDTH - Constant.GAP - 2
            if y_next <= Constant.GAP:
                y_next = Constant.GAP+2
            if y_next >= Constant.WIDTH - Constant.GAP:
                y_next = Constant.WIDTH - Constant.GAP - 2

            x_current = int(obs.x)
            y_current = int(obs.y)

            x_next = int(x_next)
            y_next = int(y_next)
            index = -1
            while Utils.distance_real((x_current, y_current), (x_next, y_next)) > Constant.GAP and x_current < Constant.WIDTH and x_current > 0 and y_current < Constant.WIDTH and y_current > 0:
                index += 1
                if x_current + self.width >= Constant.WIDTH or x_current - self.width <= 0 or y_current + self.width >= Constant.WIDTH or y_current - self.width <= 0:
                    break
                if Utils.distance_real((x_current, y_current), (self.x, self.y)) < self.width*2 and math.fabs(obs.theta - self.theta) > math.pi / 2*0.8 and math.fabs(obs.theta - self.theta) < math.pi / 2*1.2:
                    x_current += Constant.GAP * math.cos(obs.theta)
                    y_current += Constant.GAP * math.sin(obs.theta)
                    continue

                temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int(y_current // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int(x_current // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current + step_horizontal) // MAP.GAP)][int((y_current - step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True
                temp_spot = self.grid_in_vison_robot[int((x_current - step_horizontal) // MAP.GAP)][int((y_current + step_horizontal) // MAP.GAP)]
                if not temp_spot.is_barrier() and not temp_spot.is_dynamic_obs():
                    if index == 0 or index == 1:
                        temp_spot.make_dynamic_obs()
                    else:
                        temp_spot.make_future_dynamic_obs()
                    flag = True


                x_current += Constant.GAP * math.cos(obs.theta)
                y_current += Constant.GAP * math.sin(obs.theta)

        # if flag == False:
        #     return False
        
        # ------------------------------------------------------------------#
        # Use KDTree to find the nearest obstacle
        # ------------------------------------------------------------------#
        self.obstacle = [spot.get_real_pos(MAP.GAP)
            for row in self.grid_in_vison_robot for spot in row if spot.is_barrier() or spot.is_dynamic_obs() or spot.is_future_dynamic_obs()]
        self.KDTree = KDTree(self.obstacle)
        # ------------------------------------------------------------------#
        
        # ------------------------------------------------------------------#
        # Use Voronoi to find the path
        MAP.voronoi = Voronoi(self.obstacle)
        segments = []
        for i, j in MAP.voronoi.ridge_vertices:
            if i >= 0 and j >= 0:
                p1 = (MAP.voronoi.vertices[i][0], MAP.voronoi.vertices[i][1])
                p2 = (MAP.voronoi.vertices[j][0], MAP.voronoi.vertices[j][1])
                segments.append((p1, p2))

        for i, segment in enumerate(segments):
            segments[i] = ((int(segment[0][0]), int(segment[0][1])), (int(segment[1][0]), int(segment[1][1])))
        self.segments = segments
        self.segments = [(p1, p2) for p1, p2 in self.segments if Utils.line_of_sight((int(p1[0]/MAP.GAP), int(p1[1]/MAP.GAP)), (int(p2[0]/MAP.GAP), int(p2[1]/MAP.GAP)), self.grid_in_vison_robot)]
        start_pos = MAP.grid[int(self.x // MAP.GAP)][int(self.y // MAP.GAP)]

        self.pathRb = Astar_voronoi_kinematic(self, MAP, start_pos, self.segments)
        # ------------------------------------------------------------------#
        self.near_obstacle = update_safety_spot(self.grid_in_vison_robot, self.obstacle, MAP)
        return True

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def trail(self, pos, map, color):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
        self.trail_set.append(pos)
