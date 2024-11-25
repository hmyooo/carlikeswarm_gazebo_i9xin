#! /usr/bin/env python3
import math
import numpy as np
import rospy
import pcl

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
import open3d as o3d
import random

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

import geometry_msgs.msg

class map_generator(object):

    def __init__(self):
        """ Constructor """
        self.map = None
        self.points = []
        self.has_odom = False
        self.has_map = False
        self.init_params()
        self.floor_bias = 0.50
        self.time_ = 0.0


    def init_params(self):
        """ Initializes ros parameters """
        
        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path("yolov4_trt_ros")
        self.dimx = rospy.get_param("map/x_size", default=10.0)
        self.dimy = rospy.get_param("map/y_size", default=10.0)
        self.dimz = rospy.get_param("map/z_size", default=3.0)
        self.mapfile =rospy.get_param('~map/mapfile', '/home/rancho/1hmy/kinematic-cbs/map/the0.3s_ceng_all_0_5smallerwei3Z6s_back_weitianchong.pcd')

        self.resolution = rospy.get_param("map/resolution", default=0.05)

        self.add_floor = rospy.get_param("map/add_floor", default=True)
        self.add_ceiling = rospy.get_param("map/add_ceiling", default=True)
        
        self.all_map_topic = rospy.get_param("all_map_topic", default="/map_generator/global_cloud")
        self.all_map_pub = rospy.Publisher(self.all_map_topic, PointCloud2, queue_size=1)
        # self.odom_sub = rospy.Subscriber( "odometry", Odometry, self.odom_callback, queue_size=50);

        self.rate = rospy.get_param("sensing/rate", default=1)
        self.rate = rospy.Rate(self.rate)

        print("dimx: ", self.dimx)
        print("dimy: ", self.dimy)
        print("dimz: ", self.dimz)

    def add_box(self, size, position):
        ''' size: [x,y,z]
            position: [x,y,z] --- center position
        '''
        position[2] -= self.floor_bias
        x_low = math.floor((position[0] - size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        x_high = math.floor((position[0] + size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_low = math.floor((position[1] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_high = math.floor((position[1] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_low = math.floor((position[2] - size[2] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[2] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution

        x = x_low
        while x <= x_high:
            y = y_low
            while y <= y_high:
                z = z_low
                while z <= z_high:
                    if (math.fabs(x - x_low) < self.resolution) or (math.fabs(x - x_high) < self.resolution) \
                        or (math.fabs(y - y_low) < self.resolution) or (math.fabs(y - y_high) < self.resolution) \
                        or (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                        self.points.append([x,y,z])
                    z += self.resolution
                y += self.resolution
            x += self.resolution

        return

    def add_cylinder(self, size, position):
        ''' size: [r, h]
            position: [x,y,z] --- center position
        '''
        position[2] -= self.floor_bias
        center_x = position[0]
        center_y = position[1]
        z_low = math.floor((position[2] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution

        radius_num = math.floor(size[0] / self.resolution)
        x = - radius_num
        while x <= radius_num:
            y = - radius_num
            while y <= radius_num:
                radius2 = x ** 2 + y ** 2
                if radius2 < (radius_num + 0.5) ** 2:
                    z = z_low
                    while z <= z_high:
                        if radius2 > (radius_num - 0.5) ** 2 or \
                            (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                            self.points.append([center_x + x * self.resolution, center_y + y * self.resolution, z])
                        z += self.resolution
                y += 1
            x += 1

        return

    def add_layer(self, size, position):
        ''' size: [x,y]
            position: [x,y,z] --- center position
        '''        
        x_low = math.floor((position[0] - size[0] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        x_high = math.floor((position[0] + size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_low = math.floor((position[1] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_high = math.floor((position[1] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z = position[2]

        x = x_low
        while x <= x_high:
            y = y_low
            while y <= y_high:
                self.points.append([x,y,z])
                y += self.resolution
            x += self.resolution
        
        return

    def add_chair(self, position):
        '''position: [x, y] '''
        x = np.random.rand() * 0.2 + 0.4
        h = np.random.rand() * 0.1 + 0.4

        self.add_box_on_floor([x, x, h], [position[0], position[1], h/2])
        direction = np.random.randint(0, 5)
        h_ratio = 2 # 1.5 + np.random.rand() * 0.5
        if direction == 0:
            return
        elif direction == 1:
            self.add_box_on_floor([x, 0.1, h * h_ratio], [position[0], position[1] + x / 2, h * h_ratio / 2])
        elif direction == 2:
            self.add_box_on_floor([x, 0.1, h * h_ratio], [position[0], position[1] - x / 2, h * h_ratio / 2])
        elif direction == 3:
            self.add_box_on_floor([0.1, x, h * h_ratio], [position[0] + x / 2, position[1] , h * h_ratio / 2])
        elif direction == 4:
            self.add_box_on_floor([0.1, x, h * h_ratio], [position[0] - x / 2, position[1], h * h_ratio / 2])

    def add_bed(self, position):
        ''' position[x,y]'''
        x = np.random.rand() * 0.6 + 1.6
        y = np.random.rand() * 0.6 + 1.2
        z = np.random.rand() * 0.2 + 0.3
        # left right
        if np.random.rand() > 0.5:
            self.add_box_on_floor([x, y, z], [-self.dimx / 2 + x / 2, position[1], z/2])
            if np.random.rand() > 0.3:
                self.add_box_on_floor([0.1, y, z*1.5], [-self.dimx / 2 + 0.1, position[1], z*1.5/2])
        else:
            self.add_box_on_floor([x, y, z], [self.dimx / 2 - x / 2, position[1], z/2])
            if np.random.rand() > 0.3:
                self.add_box_on_floor([0.1, y, z*1.5], [self.dimx / 2 - 0.1, position[1], z*1.5/2])

    def add_table(self, position):
        ''' position[x,y]'''
        x, y, z = np.random.rand(3)
        ratio = 0.3 + np.random.rand() * 0.5

        x = np.random.rand() * 1.0 + 0.4
        y = np.random.rand() * 1.0 + 0.4
        z = z * 0.6 + 0.4
        # upper part
        if np.random.rand() > 0.5:
            self.add_cylinder([x/2, 0.1], [position[0] * 0.6, position[1], z])
        else:
            self.add_box([x, y, 0.1], [position[0] * 0.6, position[1], z])
        # lower part
        if np.random.rand() > 0.5:
            self.add_cylinder_on_floor([x/2 * ratio, z], [position[0] * 0.6, position[1]])

        else:
            self.add_box_on_floor([x * ratio, y * ratio, z], [position[0] * 0.6, position[1]])

    def add_long_door(self, position):
        ''' position[x,y]'''
        # warning: need to check map
        x = np.random.rand() * 0.4 + 0.8
        y = np.random.rand() * 0.4 + 0.4

        pos_x = (position[0] + x / 2) + np.random.rand() * (self.dimx - x)
        pos_y = position[1] + self.step_size / 2.0
        # left side
        self.add_box_on_floor([pos_x - x / 2 + self.dimx / 2, y, self.dimz], [-self.dimx / 2 + (pos_x - x / 2 + self.dimx / 2) / 2, pos_y, self.dimz / 2])
        # right side
        self.add_box_on_floor([self.dimx / 2 - pos_x - x / 2, y, self.dimz], [self.dimx / 2 - (self.dimx / 2 - pos_x - x / 2) / 2, pos_y, self.dimz / 2])

    def add_box_on_floor(self, size, position):
        ''' size: [x, y, z]
        position: [x, y] '''
        self.add_box(size, position + [size[-1] / 2])

    def add_random_box(self, position):
        ''' position: [x, y] '''
        x = 10
        y = 10
        z = 10
        while x + y + z > 4:
            x = np.random.rand() * 0.7 + 0.4
            y = np.random.rand() * 0.7 + 0.5
            z = np.random.rand() * 1.4 + 0.4

        self.add_box_on_floor([x,y,z], position)

    def add_random_cylinder(self, position):
        ''' position: [x, y] '''
        r = 10
        h = 10
        while r * 2 + h > 3:
            r = np.random.rand() * 0.35 + 0.15
            h = np.random.rand() * 1.2 + 0.8

        self.add_cylinder_on_floor([r,h], position)

    def add_stride_on_ceiling(self, position):
        ''' 
        position: [x, y] '''
        y = np.random.rand() * 0.2 + 0.4
        z = np.random.rand() * 0.4 + 0.2
        self.add_box([self.dimx, y, z], [0, position[1], self.dimz - z / 2])

    def add_stride_on_floor(self, position):
        '''
        position: [x, y] '''
        y = np.random.rand() * 0.2 + 0.4
        z = np.random.rand() * 0.4 + 0.2
        self.add_box([self.dimx, y, z], [0, position[1], 0 + z / 2])

    def add_cylinder_on_floor(self, size, position):
        ''' size: [r, h]
        position: [x, y] '''
        self.add_cylinder(size, [position[0], position[1], size[1] / 2])

    def publish_map(self):
        self.point_deal_(self.mapfile)
        if not self.has_map:
            return False
        # self.all_map_pub.publish(self.map)
        self.all_map_pub.publish(self.point_deal(self.mapfile))

        
        return True

    def odom_callback(self, odom):
        self.has_odom = True
        self.publish_map()

    def make_random_corridor(self):
        self.dimx = 50.0
        self.dimy = 50.0
        self.dimz = 3.0
        self.step_size = 1.8
        steps = math.floor(self.dimy / self.step_size)
        for i in range(int(steps)):
            # clear start and end
            if i < 1 or i > steps - 2:
                continue
            obs_num = np.random.randint(2, 4)

            center_x = - self.dimx / 2
            center_y = - self.dimy / 2 + self.step_size * i

            if np.random.rand() < 0.2:
                self.add_long_door([center_x, center_y])
                continue

            for num in range(obs_num):
                pos_x = center_x + 0.2 + np.random.rand() * (self.dimx - 0.4)
                pos_y = center_y + 0.3 + np.random.rand() * (self.step_size - 0.6)
                object_type = np.random.randint(0, 8)

                if object_type < 3:
                    self.add_random_box([pos_x, pos_y])
                # elif object_type == 1:
                #     self.add_chair([pos_x, pos_y])
                # elif object_type == 2:
                #     self.add_bed([pos_x, pos_y])
                elif object_type < 4:
                    self.add_stride_on_ceiling([center_x, center_y])
                elif object_type < 5:
                    self.add_stride_on_floor([pos_x, pos_y])
                # elif object_type == 5:
                #     self.add_table([pos_x, pos_y])
                # elif object_type == 6:
                #     self.add_random_box([pos_x, pos_y])
                elif object_type < 7:
                    self.add_random_cylinder([pos_x, pos_y])
                else:
                    self.add_random_box([pos_x, pos_y])

    # def pass_through(cloud, limit_min=0, limit_max=10, filter_value_name="z"):
 
    #     points = np.asarray(cloud.points)
    
    #     if filter_value_name == "x":
    
    #         ind = np.where((points[:, 0] >= limit_min) & (points[:, 0] <= limit_max))[0]
    #         x_cloud = pcd.select_by_index(ind)
    #         return x_cloud
    
    #     elif filter_value_name == "y":
    
    #         ind = np.where((points[:, 1] >= limit_min) & (points[:, 1] <= limit_max))[0]
    #         y_cloud = cloud.select_by_index(ind)
    #         return y_cloud
    
    #     elif filter_value_name == "z":
    
    #         ind = np.where((points[:, 2] >= limit_min) & (points[:, 2] <= limit_max))[0]
    #         z_cloud = pcd.select_by_index(ind)
    #         return z_cloud

    def point_deal(self,file):
        pt = o3d.io.read_point_cloud(file)
        # self.time_ =self.time_ +0.1
        points = np.asarray(pt.points)
        # ind = np.where((points[:, 2] >= self.time_*10) & (points[:, 2] <= self.time_*10))[0]
        # z_cloud = pt.select_by_index(ind)
        # z_cloud_points=np.asarray(z_cloud.points)
        # print(points)
        # pt = pcl.load(file)
        # points = pt.to_array()
        pointcloud1 = PointCloud2()
        pointcloud1.height = 1
        pointcloud1.width = len(pt.points)
        pointcloud1.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        pointcloud1.point_step = 12 #12
        pointcloud1.row_step = 12 * len(pt.points)
        pointcloud1.is_bigendian = False
        pointcloud1.is_dense = False
        pointcloud1.data = np.asarray(points, np.float32).tostring()
        pointcloud1.header.frame_id = "map"
        self.has_map = True
        rospy.loginfo("finish making map")
        return pointcloud1
    def point_deal_(self,file):
        pt = o3d.io.read_point_cloud(file)
        self.has_map = True


    def make_map(self):
        rospy.loginfo("start making map")


        # # scene 0 : square room
        # self.dimx = 20
        # self.dimy = 20
        # self.diz = 2.0
        # for x in range(-4, 5):
        #     for y in range(-4, 5):
        #         # if x == 0 and y == 0:
        #         #     continue
        #         self.add_box([0.8, 0.8, self.dimz], [x * 2, y * 2, self.dimz / 2])


        # # scene 1 : corridor
        # self.make_random_corridor()

        # floor
        # self.add_layer([self.dimx, self.dimy], [0,0,-0.5])
        # # ceiling
        # # self.add_layer([self.dimx, self.dimy], [0,0, self.dimz - self.floor_bias])
        # # boundary
        # # left
        # self.add_box([0.10, self.dimy, self.dimz], [-self.dimx / 2, 0, self.dimz / 2.0])
        # # right
        # self.add_box([0.10, self.dimy, self.dimz], [self.dimx / 2, 0, self.dimz / 2.0])
        # # top
        # self.add_box([self.dimx, 0.10, self.dimz], [0, self.dimy / 2, self.dimz / 2.0])
        # # # button
        # self.add_box([self.dimx, 0.10, self.dimz], [0, -self.dimy / 2, self.dimz / 2.0])


        # # add items
        # # sofa 1
        # self.add_box([2.5, 0.2, 1.1], [-4, -3.8, 0.65])
        # self.add_box([2.5, 0.5, 0.5], [-4, -3.35, 0.3])

        # # table 1
        # self.add_box([1.5, 0.8, 0.6], [-4, -2.30, 0.3])

        # # table 2
        # self.add_cylinder([0.8, 0.6], [-3, -0.2, 0.4])

        # # chair 1
        # self.add_box([0.6, 0.6, 0.5], [-4.3, -0.5, 0.3])
        # self.add_box([0.1, 0.6, 1.4], [-4.55, -0.5, 0.75])

        # self.add_box([0.6, 0.6, 0.5], [-1.7, -0.5, 0.3])
        # self.add_box([0.1, 0.6, 1.4], [-1.45, -0.5, 0.75])

        # # washing machine
        # self.add_box([0.6, 0.9, 1.0], [-4.2, 2.3, 0.5])


        # # bed 1
        # self.add_box([2.5, 2.0, 0.5], [4.65, -2.0, 0.25])
        # # nightstand 1
        # self.add_box([0.5, 0.6, 0.6], [5.65, -0.3, 0.3])

        # wall 0
        # self.add_box([0.8, 3.0, 5.2], [0.0, 2.4, 0.6])
        # wall 1 / 2 / 3 (door)
        # self.add_box([3.5, 0.3, 3.0], [-4.3, 1.8, 1.5])
        # self.add_box([1.0, 0.3, 3.0], [-4.3, 1.8, 1.5])
        
        # self.add_box([0.5, 4.0, 3.0], [-10.0, -2.5, 1.5])

        # benchmark




        self.add_cylinder([1.5, 2.5], [-20, 30, 1.5])

        self.add_cylinder([1.5, 2.5], [2, 25, 1.5])

        self.add_cylinder([1.5, 2.5], [23, 28, 1.5])

        self.add_cylinder([1.5, 2.5], [-16, 10, 1.5])

        self.add_cylinder([1.5, 2.5], [20, 4, 1.5])

        self.add_cylinder([1.5, 2.5], [-20, -20, 1.5])

        self.add_cylinder([1.5, 2.5], [4, -25, 1.5])

        self.add_cylinder([1.5, 2.5], [25,-10, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-40.0, 30.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [10.0, 32.0, 1.5])
        
        self.add_box([3.0, 3.0, 3.0], [40.0, 28.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-32.0, 10.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [8.0, 8.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [30.0, 10.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-35.0, -10.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-18.0, -3.0, 1.5])

        self.add_box([4.0, 6.0, 3.0], [0.0, -3.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [38.0, -4.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-20.0, -30.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [20.0, -22.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-8.0, 18.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [-8.0, -18.0, 1.5])

        self.add_box([3.0, 3.0, 3.0], [10.0, -10.0, 1.5])

        # transfer to pcl
        self.map = PointCloud2()
        self.map.height = 1
        self.map.width = len(self.points)
        self.map.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        self.map.point_step = 12 #12
        self.map.row_step = 12 * len(self.points)
        self.map.is_bigendian = False
        self.map.is_dense = False
        self.map.data = np.asarray(self.points, np.float32).tostring()
        self.map.header.frame_id = "map"

        self.has_map = True
        rospy.loginfo("finish making map")

        return True
    
class DynamicObstacle(object):

    def __init__(self, start_x, start_y, velocity_x, velocity_y):

        self.current_x = start_x

        self.current_y = start_y

        self.velocity_x = velocity_x

        self.velocity_y = velocity_y
        self.history_path = Path()
        self.history_path.header.frame_id = "map"
        self.future_trajpath=Path()
        self.future_trajpath.header.frame_id = "map"
        self.future_x=start_x
        self.future_y=start_y
        self.last_time=rospy.Time.now()




        

    def update_position(self):

        # 更新障碍物位置
        if(self.current_x+self.velocity_x<495 or self.current_x+self.velocity_x*(rospy.Time.now()-self.last_time).to_sec()>660):
            self.velocity_x=0-self.velocity_x
        if(self.current_y+self.velocity_y<420 or self.current_y+self.velocity_y*(rospy.Time.now()-self.last_time).to_sec()>480):
            self.velocity_y=0-self.velocity_y

        self.current_x += self.velocity_x*(rospy.Time.now()-self.last_time).to_sec()
        self.current_y += self.velocity_y*(rospy.Time.now()-self.last_time).to_sec()
         
        if(len(self.history_path.poses)>10):
            self.history_path.poses.clear()

    def future_traj(self):
        self.future_trajpath.poses.clear()
        self.future_x=self.current_x
        self.future_y=self.current_y
        for i in range(5):
            obstacle_pose_fu = PoseStamped()

            obstacle_pose_fu.header.stamp = rospy.Time.now()


            obstacle_pose_fu.header.frame_id = "map"
            # if(self.future_x+self.velocity_x<0 or self.future_x+self.velocity_x>600):
            #     self.velocity_x=0-self.velocity_x
            # if(self.future_x+self.velocity_y<0 or self.future_x+self.velocity_y>500):
            #     self.velocity_y=0-self.velocity_y
            self.future_x+=self.velocity_x
            self.future_y += self.velocity_y
            obstacle_pose_fu.pose.position.x = self.future_x

            obstacle_pose_fu.pose.position.y = self.future_y
            obstacle_pose_fu.pose.position.z = 0.1

            self.future_trajpath.poses.append(obstacle_pose_fu)

        # 更新障碍物位置
            
        # if(len(self.future_trajpath.poses)>8):
        #     self.future_trajpath.poses.clear()

def generate_random_obstacle():

    # 随机生成初始位置和速度

    start_x = random.uniform(495, 660.0)

    start_y = random.uniform(420,480.0)

    velocity_x = random.uniform(-5.0, 5.0)  # 随机速度范围，可以根据需要修改

    velocity_y = random.uniform(-5.0, 5.0)


    return DynamicObstacle(start_x, start_y, velocity_x, velocity_y)


def publish_obstacle_positions(obstacles,publisher,publisher_path,publisher_maker,publisher_fupath,publisher_all_pose,all_pose_path):

    # rospy.init_node('dynamic_obstacle_publisher')

    # obstacle_pub = rospy.Publisher('/dynamic_obstacle/position', PoseStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz，根据需要调整发布频率
    all_pose_path.header.stamp = rospy.Time.now()
    all_pose_path.poses.clear()

    # while not rospy.is_shutdown():

        # 更新每个障碍物的位置
    for i in range(0,len(obstacles),1):
    # for obstacle in obstacles:

        obstacles[i].update_position()
        obstacles[i].future_traj()
        obstacles[i].last_time=rospy.Time.now()
        # 创建实时位置消息

        obstacle_pose = PoseStamped()
        obstacle_pose_2 = PoseStamped()
        obstacle_pose.header.stamp = rospy.Time.now()

        obstacle_pose.header.frame_id = "map"

        obstacle_pose.pose.position.x = obstacles[i].current_x

        obstacle_pose.pose.position.y = obstacles[i].current_y
        obstacle_pose.pose.position.z = 0.1
        obstacle_pose.pose.orientation.x=1

        obstacle_pose_2.header.stamp = rospy.Time.now()

        obstacle_pose_2.header.frame_id = "map"

        obstacle_pose_2.pose.position.x = obstacles[i].current_x

        obstacle_pose_2.pose.position.y = obstacles[i].current_y
        obstacle_pose_2.pose.position.z = 0.1
        obstacle_pose_2.pose.orientation.w=i

        # print("orientation.x ")
        # print(obstacle_pose.pose.orientation.x)
        # obstacle_pose.pose.orientation.y= 1
        # obstacle_pose.pose.orientation.z= 0

        obstacles[i].history_path.poses.append(obstacle_pose)
        obstacles[i].history_path.header.stamp = rospy.Time.now()

        maker_msg=Marker()
        maker_msg.header=obstacle_pose.header
        maker_msg.type=Marker.SPHERE
        maker_msg.action=Marker.ADD
        maker_msg.scale.x=5.0
        maker_msg.scale.y=5.0
        maker_msg.scale.z=1.0
        maker_msg.color.a=1.0
        maker_msg.color.r=1.0
        maker_msg.color.g=0.0
        maker_msg.color.b=0.0
        maker_msg.pose=obstacle_pose.pose
        maker_msg.pose.orientation.x= 1.0


        # 发布消息
        publisher_path[i].publish(obstacles[i].history_path)
        publisher_fupath[i].publish(obstacles[i].future_trajpath)
        publisher[i].publish(obstacle_pose)
        publisher_maker[i].publish(maker_msg)
        all_pose_path.poses.append(obstacle_pose_2)
    publisher_all_pose.publish(all_pose_path)
    
        # rate.sleep()


def main():
    rospy.init_node('random_map_sensing', anonymous=True)
    # num_obstacles = 15 # 指定障碍物数量
    # publisher=[]
    # publisher_path=[]
    # publisher_maker=[]
    # publisher_fupath=[]
    # topic_name_all_pose="/dy_obs_posepath"
    # all_pose_path=Path()
    # publisher_all_pose=rospy.Publisher(topic_name_all_pose,Path, queue_size=10)
    # for i in range(0,num_obstacles):
    #     topic_name=f"/obstacle_{i}"
    #     topic_namepath=f"/obstacle_path{i}"
    #     topic_namemaker=f"/obstacle_maker{i}"
    #     topic_namepathfu=f"/obstacle_fu_path{i}"
        
        
    #     publisher_maker.append(rospy.Publisher(topic_namemaker,Marker, queue_size=10))
    #     publisher.append(rospy.Publisher(topic_name,PoseStamped, queue_size=10))
    #     publisher_path.append(rospy.Publisher(topic_namepath,Path, queue_size=10))
    #     publisher_fupath.append(rospy.Publisher(topic_namepathfu,Path, queue_size=10))

    # obstacless = [generate_random_obstacle() for _ in range(num_obstacles)]
    map_maker = map_generator()
    map_maker.make_map()
    rate = rospy.Rate(10)  # 1 Hz，根据需要调整发布频率

    while not rospy.is_shutdown():
        map_maker.publish_map()
        map_maker.rate.sleep()
        # for i in range(0,num_obstacles):
        # publish_obstacle_positions(obstacless,publisher,publisher_path,publisher_maker,publisher_fupath,publisher_all_pose,all_pose_path)
        

if __name__ == '__main__':
    main()

