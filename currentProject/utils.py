from dynamicObstacle import DynamicObstacle
from constant import Constant
import math
import random

class Utils:
    
    def distance_real(point1, point2):
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
    
    def angle_betwent(point1, point2):
        (x1, y1) = point1
        (x2, y2) = point2
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)

        angle = math.atan2(y2 - y1, x2 - x1)
        return angle
    
    def distance_spot(h1, h2):
        x1, y1 = h1
        x2, y2 = h2
        return abs(x1 - x2) + abs(y1 - y2)
    
    def load_map(file_name, grid, name_map, rows):
    # open file
        obstacles = []
        file_name = open(file_name, "r")
        for line in file_name:
            # with line start with "map1" is map 1
            if line.startswith(name_map):
                # read next line
                for i in range(rows):
                    line1 = file_name.readline()
                    # print(line1)
                    for j in range(rows*2):
                        if line1[j] == "1":
                            grid[i][int(j/2)].make_barrier()
                            obstacles.append(grid[i][int(j/2)])
                        elif line1[j] == "2":
                            start = grid[i][int(j/2)]
                            start.make_start()
                        elif line1[j] == "3":
                            end = grid[i][int(j/2)]
                            end.make_end()
                        elif line1[j] == "0":
                            grid[i][int(j/2)].reset()
        # close file
        file_name.close()
        # print grid
        return grid, start, end , obstacles
    
    def load_dynamic_obstacles(file_name):
        dynamic_obstacles = []
        with open(file_name, 'r') as f:
            for line in f:
                # parse obstacle position and velocity from file
                x, y, d, theta, veloc = map(float, line.strip().split(','))

                # create dynamic obstacle with given velocity and position
                obs = DynamicObstacle(x, y, d, theta, veloc)

                # add obstacle to the list
                dynamic_obstacles.append(obs)
        return dynamic_obstacles
    
    def generate_dynamic_obstacles(num_obstacles, MAP):
        dynamic_obstacles = []
        min_size = Constant.MIN_SIZE
        max_size = Constant.MAX_SIZE
        min_velocity = Constant.MIN_VELOCITY
        max_velocity = Constant.MAX_VELOCITY
        for i in range(num_obstacles):
            d = random.randint(min_size, max_size)
            velocity = random.randint(min_velocity, max_velocity)
            while True:
                x = random.randint(MAP.GAP+d +1, MAP.WIDTH -MAP.GAP-d -1 )
                y = random.randint(MAP.GAP+d +1, MAP.WIDTH -MAP.GAP-d -1)
                if MAP.grid[int((x + d)/MAP.GAP)][int(y/MAP.GAP)].is_barrier() == False or \
                    MAP.grid[int(x/MAP.GAP)][int((y+ d)/MAP.GAP)].is_barrier() == False or \
                        MAP.grid[int((x - d)/MAP.GAP)][int(y/MAP.GAP)].is_barrier() == False or \
                            MAP.grid[int(x/MAP.GAP)][int((y - d)/MAP.GAP)].is_barrier() == False or \
                            Utils.distance_real((x, y), (MAP.start.get_real_pos(MAP.GAP))) > 100:
                    break

            theta = random.uniform(0, 360)
            obs = DynamicObstacle(x, y, d, theta, velocity)
            dynamic_obstacles.append(obs)
        return dynamic_obstacles

    def log_collision(file_name, time, map_name, robot, dynamic_obstacles):
        with open(file_name, 'a') as f:
            f.write(f'{time},{map_name},{robot.x},{robot.y},{robot.theta}')
            for obs in dynamic_obstacles:
                f.write(f',{obs.x},{obs.y},{obs.d},{obs.theta},{obs.velocity}')
            f.write('\n')

    def log_info_path(file_name, map_name, len_path, time_move, safety_score, num_collision):
        with open(file_name, 'a') as f:
            f.write(f'{map_name},{len_path},{time_move},{safety_score},{num_collision}\n')

    def line_of_sight(p1, p2, grid):
        # print(p1,p2)
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

        # print(x1 + ((sx - 1) // 2))
        # print(y1+1)
        if dx >= dy:
            while x1 != x2:
                f = f + dy
                if f >= dx:
                    if grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_dynamic_obs():
                        return False
                    y1 = y1 + sy
                    f = f - dx

                if f != 0 and grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_dynamic_obs():
                    return False

                # if (dy == 0 and grid[x1 + ((sx - 1) // 2)][y1].is_barrier()) or (dy == 0 and grid[x1 + ((sx - 1) // 2)][y1 + 1].is_barrier()) or \
                #         (dy == 0 and grid[x1 + ((sx - 1) // 2)][y1].is_barrier()) or (dy == 0 and grid[x1 + ((sx - 1) // 2)][y1 - 1].is_barrier()):
                #     return False
                
                if (dy == 0 and (grid[x1 + ((sx - 1) // 2)][y1].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1].is_dynamic_obs())) and (grid[x1 + ((sx - 1) // 2)][y1 - 1].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 - 1].is_dynamic_obs()):
                    return False

                x1 = x1 + sx
        else:
            while y1 != y2:
                f = f + dx
                if f >= dy:
                    if grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_dynamic_obs():
                        return False
                    x1 = x1 + sx
                    f = f - dy

                if f != 0 and grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 + ((sx - 1) // 2)][y1 + ((sy - 1) // 2)].is_dynamic_obs():
                    return False

                # if (dx == 0 and grid[x1][y1 + ((sy - 1) // 2)].is_barrier()) or (dx == 0 and grid[x1 + 1][y1 + ((sy - 1) // 2)].is_barrier()) or \
                #    (dx == 0 and grid[x1][y1 + ((sy - 1) // 2)].is_barrier()) or (dx == 0 and grid[x1 - 1][y1 + ((sy - 1) // 2)].is_barrier()):
                #     return False
                if (dx == 0 and (grid[x1][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1][y1 + ((sy - 1) // 2)].is_dynamic_obs())) and (grid[x1 - 1][y1 + ((sy - 1) // 2)].is_barrier() or grid[x1 - 1][y1 + ((sy - 1) // 2)].is_dynamic_obs()):
                    return False

                y1 = y1 + sy

        return True
