#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import math
import numpy as np

class ODGPF:
    def __init__(self):
        self.ackermann_data = AckermannDriveStamped()
        self.PI = 3.141592
        self.LOOK = 2.5
        self.SPEED_MAX = 8.0
        self.SPEED_MIN = 4.0
        self.RATE = 100
        self.ROBOT_SCALE = 0.35
        self.THRESHOLD = 6
        self.FILTER_SCALE = 1.3
        self.scan_range = 0
        self.desired_wp_rt = [0,0]

        self.wp_num = 1
        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.goal_point = [50,-24.1, 0]
        self.nearest_distance = 0

        self.current_position = [0,0,0]
        self.interval = 0.00435
        self.gamma = 1
        self.a_k = 3

        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        self.ackermann_data.drive.steering_angle = 0
        self.ackermann_data.drive.steering_angle_velocity = 0

        rospy.Subscriber('/scan', LaserScan, self.subCallback_scan, queue_size = 10)
        rospy.Subscriber('/odom', Odometry, self.Odome, queue_size = 10)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 10 )

        self.mode = 0

    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        
        return np.sqrt(dx**2 + dy**2)

    def transformPoint(self, origin, target):
        theta = self.PI/2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta
        
        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]
        
        return tf_point

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        #rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x*x + y*y))
        rtpoint.append(np.arctan2(y, x) - (self.PI/2))

        return rtpoint

    def get_waypoint(self):
    #file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        file_wps = np.genfromtxt('../f1tenth_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],0]
            temp_waypoint.append(wps_point)
            self.wp_num += 1
        #print("wp_num",self.wp_num)
        return temp_waypoint

    def find_desired_wp(self):
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp+=1

            if wp_index_temp >= self.wp_num-1:
                wp_index_temp = 0
            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.LOOK*1.2)) or (wp_index_temp == self.wp_index_current)):
                break
        
        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.wp_num-1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.LOOK: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])
        self.desired_wp_rt = self.xyt2rt(transformed_nearest_point)

    def define_obstacles(self, scan):
        obstacles = []
        
        i = 450

        while i < 630:

            if scan[i] < self.THRESHOLD:
                
                start_temp = scan[i]
                start_idx_temp = i
                end_temp = scan[i]
                end_idx_temp = i
                obstacle_count = 1

                # max_temp = scan[i]
                # max_idx_temp = i
                
                while ((scan[i] < self.THRESHOLD) and (i+1 < 630 )):#self.scan_range
                    i += 1
                    end_temp += scan[i]
                    obstacle_count += 1
                    # if scan[i] > max_temp:
                    #     max_temp = scan[i]
                    #     max_idx_temp = i
                if scan[i] > self.THRESHOLD:
                    end_temp += scan[i]
                    obstacle_count += 1
                    i += 1
                end_idx_temp = i

                distacne_obstacle = end_temp/obstacle_count
                angle_obstacle = np.fabs(end_idx_temp - start_idx_temp)*self.interval
                sigma_obstacle = np.arctan2(distacne_obstacle * np.tan(angle_obstacle/2) + (self.ROBOT_SCALE/2), distacne_obstacle)
                
                obstacle_imf = [angle_obstacle, sigma_obstacle, start_idx_temp, end_idx_temp]
                #print('obstacle', obstacle_imf)
                obstacles.append(obstacle_imf)

            i += 1
        return obstacles

    def rep_field(self, obstacles):
        f_rep_list = []
        for i in range(self.scan_range):
            f_rep = 0
            for j in range(len(obstacles)):
                f_rep += self.a_k * np.exp(-(((obstacles[j][0] - i*self.interval)**2) / (2 * (obstacles[j][1]**2))))
            f_rep_list.append(f_rep)
        # print('f_rep_list:',f_rep_list)
        return f_rep_list

    def att_field(self, goal_point):
        #transformed_nearest_point = self.transformPoint(self.current_position, goal_point)
        #self.goal_theta = self.xyt2rt(transformed_nearest_point)
        # print('current_position',self.current_position, 'goal_theta',self.goal_theta[1])#'goal_point:',goal_point, 

        f_att_list = []
        for i in range(self.scan_range):
            f_att = self.gamma * np.fabs(goal_point[1] - (540-i)*self.interval)
            f_att_list.append(f_att)
        return f_att_list

    def total_field(self, f_rep_list, f_att_list):
        
        f_total_list = []
        for i in range(self.scan_range):
            f_total = f_rep_list[i] + f_att_list[i]
            f_total_list.append(f_total)

        return f_total_list

    def angle(self, f_total_list):
        min_f = f_total_list[0]
        min_f_idx = 0
        for i in range(1,self.scan_range):
            if min_f > f_total_list[i]:
                min_f = f_total_list[i]
                min_f_idx = i

        return min_f_idx

    def main_drive(self, goal):
        #self.max_angle = 
        if self.mode == 1:

            distance = 1.0

            path_radius = distance / (2*np.sin(goal))
            steering_angle = np.arctan(self.ROBOT_SCALE/path_radius)
            
        elif self.mode == 0:
            # print(goal)
            steering_angle = (540-goal)*self.interval
            
            # if np.fabs(steering_angle - self.steering_angle_past) < 0.2: 
            #print(540-goal)
            # print(steering_angle)

        if (np.fabs(steering_angle) > self.PI/8):
            speed = self.SPEED_MIN
        else:
            speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(steering_angle)+self.SPEED_MAX)
            speed = np.fabs(speed)

        # print('steering_angle:',steering_angle)
        print('speed:',speed)
        self.ackermann_data.drive.steering_angle = steering_angle
        self.ackermann_data.drive.steering_angle_velocity = 0
        self.ackermann_data.drive.speed = speed
        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        
        self.steering_angle_past = steering_angle

        self.drive_pub.publish(self.ackermann_data)

    def Odome(self, odom_msg):
        qx = odom_msg.pose.pose.orientation.x 
        qy = odom_msg.pose.pose.orientation.y 
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w 

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        self.current_position = [current_position_x,current_position_y, current_position_theta]

        self.find_desired_wp()

    def subCallback_scan(self,msg_sub):
        self.scan_angle_min = msg_sub.angle_min
        self.scan_angle_max = msg_sub.angle_max
        self.interval = msg_sub.angle_increment
        self.scan_range = len(msg_sub.ranges)
        self.front_idx = (int)(self.scan_range/2)
        
        self.scan_origin = [0]*self.scan_range
        self.scan_filtered = [0]*self.scan_range
        for i in range(self.scan_range):
            
            self.scan_origin[i] = msg_sub.ranges[i]
            self.scan_filtered[i] = msg_sub.ranges[i]

        for i in range(self.scan_range):           
            if self.scan_origin[i] == 0:
                cont = 0 
                sum = 0
                for j in range(1,21):
                    if i-j >= 0:
                        if self.scan_origin[i-j] != 0:
                            cont += 1
                            sum += self.scan_origin[i-j]
                    if i+j < self.scan_range:
                        if self.scan_origin[i+j] != 0:
                            cont += 1
                            sum += self.scan_origin[i+j]
                self.scan_origin[i] = sum/cont
                self.scan_filtered[i] = sum/cont


        for i in range(self.scan_range - 1):
            if self.scan_origin[i]*self.FILTER_SCALE < self.scan_filtered[i+1]:
                unit_length = self.scan_origin[i]*self.interval 
                filter_num = self.ROBOT_SCALE/unit_length

                j = 1
                while j < filter_num+1:
                    if i+j < self.scan_range:
                        if self.scan_filtered[i+j] > self.scan_origin[i]:
                            self.scan_filtered[i+j] = self.scan_origin[i]
                        else: break
                    else: break 
                    j += 1
        
            elif self.scan_filtered[i] > self.scan_origin[i+1]*self.FILTER_SCALE:
                unit_length = self.scan_origin[i+1]*self.interval
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i-j > 0:
                        if self.scan_filtered[i-j] > self.scan_origin[i+1]:
                            self.scan_filtered[i-j] = self.scan_origin[i+1]
                        else: break
                    else: break
                    j += 1

    def driving(self):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():

            if self.scan_range == 0: continue
            
            obstacles = self.define_obstacles(self.scan_filtered)

            rep_list = self.rep_field(obstacles)
            att_list = self.att_field(self.desired_wp_rt)
            total_list = self.total_field(rep_list, att_list)

            desired_angle = self.angle(total_list)

            self.main_drive(desired_angle)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    A = ODGPF()
    A.driving()
    rospy.spin()