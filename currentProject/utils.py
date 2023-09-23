from dynamicObstacle import DynamicObstacle
import math


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

