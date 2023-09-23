from utils import Utils
from constant import Constant
import math

class Sensor:
    def __init__(self, list_dynamic_obstacles):
        # distance between robot and dynamic obstacle at the current time
        self.NOW_distance_dynamic = {}
        # angle between robot and dynamic obstacle at the current time
        self.NOW_angle_dynamic = {}
        # distance between robot and dynamic obstacle at the previous time
        self.PRE_distance_dynamic = {}
        # angle between robot and dynamic obstacle at the previous time
        self.PRE_angle_dynamic = {}
        # robot's covered distance
        self.robot_covered_distance = 0
        # minimum distance threshold between an obstacle and available unnavigated path of the robot
        self.min_path_distance_threshold = 50
        # distance between path and obstacle now
        self.NOW_distance_path_dynamic = {}
        # distance between path and obstacle pre
        self.PRE_distance_path_dynamic = {}
        # dynamic obstacle's covered distance
        self.dynamic_covered_distance = {}
        # robot's location pre
        self.robot_pre_location = None
        # robot's location now
        self.robot_now_location = None
        for obstacle in list_dynamic_obstacles:
            self.NOW_distance_dynamic[obstacle] = None
            self.NOW_angle_dynamic[obstacle] = None
            self.PRE_distance_dynamic[obstacle] = None
            self.PRE_angle_dynamic[obstacle] = None
            self.NOW_distance_path_dynamic[obstacle] = None
            self.PRE_distance_path_dynamic[obstacle] = None
            self.dynamic_covered_distance[obstacle] = None

    def get_angle_dynamic(self, robot, obstacle):
        """
        :param robot: Robot
        :param obstacle: Obstacle
        :return: angle between robot and obstacle
        """
        return Utils.angle_betwent((robot.x, robot.y), (obstacle.x, obstacle.y))

    def get_distance_dynamic(self, robot, obstacle):
        """
        :param robot: Robot
        :param obstacle: Obstacle
        :return: distance between robot and obstacle
        """
        return Utils.distance_real((robot.x, robot.y), (obstacle.x, obstacle.y))

    def get_distance_path_dynamic(self, robot, obstacle):
        """
        :param robot: Robot
        :param obstacle: Obstacle
        :return: distance between robot's path and obstacle
        """
        min_distance = 1000000
        index = robot.index_path
        for i in range(5):
            if index + i*2 < len(robot.pathRb):
                location = robot.pathRb[index + i*2]
                distance = Utils.distance_real((location[0], location[1]), (obstacle.x, obstacle.y))
                if distance < min_distance:
                    min_distance = distance

        return min_distance


    def get_dynamic_covered_distance(self, obstacle):
        """
        :param obstacle: Obstacle
        :return: obstacle's covered distance
        """
        return self.NOW_distance_dynamic[obstacle] - self.PRE_distance_dynamic[obstacle]
    
    def get_robot_covered_distance(self, robot):
        """
        :param robot: Robot
        :return: robot's covered distance
        """
        return Utils.distance_real(self.robot_pre_location, self.robot_now_location)

    def get_information(self, robot, list_obstacle):
        """
        :param robot: Robot
        :param obstacle: Obstacle
        :return: distance between robot and obstacle, angle between robot and obstacle, distance between robot's path and obstacle
        """
        self.PRE_angle_dynamic = self.NOW_angle_dynamic.copy()
        self.PRE_distance_dynamic = self.NOW_distance_dynamic.copy()
        self.PRE_distance_path_dynamic = self.NOW_distance_path_dynamic.copy()
        self.robot_pre_location = self.robot_now_location
        self.robot_now_location = (robot.x, robot.y)
        for obstacle in list_obstacle:
            if Utils.distance_real((robot.x, robot.y), (obstacle.x, obstacle.y)) < Constant.OBSTACLE_DETECTION_DISTANCE:
                self.NOW_distance_path_dynamic[obstacle] = self.get_distance_path_dynamic(robot, obstacle)
                self.NOW_distance_dynamic[obstacle] = self.get_distance_dynamic(robot, obstacle)
                self.NOW_angle_dynamic[obstacle] = self.get_angle_dynamic(robot, obstacle)
                if self.PRE_distance_dynamic[obstacle] is not None and self.NOW_distance_dynamic[obstacle] is not None:
                    self.dynamic_covered_distance[obstacle] = self.get_dynamic_covered_distance(obstacle)
            else:
                self.NOW_distance_path_dynamic[obstacle] = None
                self.NOW_distance_dynamic[obstacle] = None
                self.NOW_angle_dynamic[obstacle] = None
                self.dynamic_covered_distance[obstacle] = None

        if self.robot_pre_location is not None:
            self.robot_covered_distance = self.get_robot_covered_distance(robot)

    def detect_replan(self, robot, list_obstacle):
        self.get_information(robot, list_obstacle)
        target_obstacle = set()
        # check replan
        for obstacle in list_obstacle:
            if self.NOW_distance_dynamic[obstacle] is not None and self.PRE_distance_dynamic[obstacle] is not None and Utils.distance_real((robot.x, robot.y), (obstacle.x, obstacle.y)) < Constant.OBSTACLE_REPLANNING_DISTANCE:
                # print("distance between robot and obstacle: ", self.NOW_distance_dynamic[obstacle])
                # print("angle between robot and obstacle: ", self.NOW_angle_dynamic[obstacle])
                # print("distance between robot's path and obstacle: ", self.NOW_distance_path_dynamic[obstacle])
                # print("pre distance between robot and obstacle: ", self.PRE_distance_dynamic[obstacle])
                # print("pre angle between robot and obstacle: ", self.PRE_angle_dynamic[obstacle])
                # print("pre distance between robot's path and obstacle: ", self.PRE_distance_path_dynamic[obstacle])
                # print("obstacle's covered distance: ", self.dynamic_covered_distance[obstacle])
                # print("robot's covered distance: ", self.robot_covered_distance)
                # print("robot's index path: ", robot.index_path)
                # print("obstacle's location: ", (obstacle.x, obstacle.y))
                
                # if math.fabs(self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle]) < Constant.MIN_APROXIMATE_ANGLE:
                #     if self.PRE_distance_dynamic - (self.NOW_distance_dynamic + self.robot_covered_distance) > Constant.MIN_APROXIMATE_DISTANCE:
                #         target_obstacle.add(obstacle)
                #     elif self.NOW_distance_path_dynamic[obstacle] < self.PRE_distance_dynamic[obstacle]:
                #         target_obstacle.add(obstacle)
                
                # if self.PRE_angle_dynamic[obstacle] - self.NOW_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE:
                #     if math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                #         target_obstacle.add(obstacle)
                # elif self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE:
                #     if math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                #         target_obstacle.add(obstacle)


                if math.fabs(self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle]) < Constant.MIN_APROXIMATE_ANGLE and \
                    math.fabs((self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle]) < Constant.MIN_APROXIMATE_DISTANCE and \
                    self.NOW_distance_dynamic[obstacle] < self.PRE_distance_dynamic[obstacle]:
                    target_obstacle.add(obstacle)
                
                if math.fabs(self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle]) < Constant.MIN_APROXIMATE_ANGLE and \
                    (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle] > Constant.MIN_APROXIMATE_DISTANCE and \
                        self.NOW_distance_dynamic[obstacle] < self.PRE_distance_dynamic[obstacle]:
                        target_obstacle.add(obstacle)
                
                if math.fabs(self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle]) < Constant.MIN_APROXIMATE_ANGLE and \
                    self.PRE_distance_dynamic[obstacle] - (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) > Constant.MIN_APROXIMATE_DISTANCE:
                        target_obstacle.add(obstacle)

                if self.NOW_distance_path_dynamic[obstacle] < self.min_path_distance_threshold:
                    if self.PRE_angle_dynamic[obstacle] - self.NOW_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        math.fabs((self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle]) < Constant.MIN_APROXIMATE_DISTANCE and \
                        math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)

                    if self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        math.fabs((self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle]) < Constant.MIN_APROXIMATE_DISTANCE and \
                        math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)

                    if self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle]  > Constant.MIN_APROXIMATE_DISTANCE \
                        and  math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)
                    
                    if self.NOW_angle_dynamic[obstacle] - self.PRE_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        self.PRE_distance_dynamic[obstacle] - (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) > Constant.MIN_APROXIMATE_DISTANCE \
                        and  math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)

                    if self.PRE_angle_dynamic[obstacle] - self.NOW_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) - self.PRE_distance_dynamic[obstacle]  > Constant.MIN_APROXIMATE_DISTANCE \
                        and  math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)

                    if self.PRE_angle_dynamic[obstacle] - self.NOW_angle_dynamic[obstacle] > Constant.MIN_APROXIMATE_ANGLE and \
                        self.PRE_distance_dynamic[obstacle] - (self.NOW_distance_dynamic[obstacle] + self.robot_covered_distance) > Constant.MIN_APROXIMATE_DISTANCE \
                        and  math.fabs(self.NOW_distance_path_dynamic[obstacle] - self.PRE_distance_path_dynamic[obstacle]) > Constant.MIN_APROXIMATE_DISTANCE:
                            target_obstacle.add(obstacle)
        return target_obstacle    

        
                
                    
# distance between robot and obstacle:  17.687130482262294
# angle between robot and obstacle:  0.7381734476229826
# distance between robot's path and obstacle:  48.93143823963382
# pre distance between robot and obstacle:  17.687130482262294
# pre angle between robot and obstacle:  0.7381734476229826
# pre distance between robot's path and obstacle:  48.93143823963382
# obstacle's covered distance:  0.0
# robot's covered distance:  28.715200578338354
# robot's index path:  73
# obstacle's location:  (431.10399999980007, 645.316534616328)
# distance between robot and obstacle:  84.06356762229183
# angle between robot and obstacle:  0.4036657430932597
# distance between robot's path and obstacle:  5.022455839055446
# pre distance between robot and obstacle:  84.06356762229183
# pre angle between robot and obstacle:  0.4036657430932597
# pre distance between robot's path and obstacle:  5.022455839055446
# obstacle's covered distance:  0.0
                

    



