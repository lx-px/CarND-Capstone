#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50#200  # Number of waypoints we will publish. You can change this number
RATE = 2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.bw_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.base_waypoints = None
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.pose = None
        self.stop_wp_idx = -1
        self.decel_limit = rospy.get_param('~decel_limit', -5)

        self.loop()

    def loop(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_2d and self.waypoint_tree:
                closest_coord_idx = self.get_closest_coord()
                self.publish_waypoints(closest_coord_idx)
            rate.sleep()

    def get_closest_coord(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        curr_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, curr_vect - cl_vect)
        if val > 0:
            return (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx + 1

    def publish_waypoints(self, closest_idx):
        #generate required waypoints and publish        
        lane = self.generate_lane(closest_idx)
        self.final_waypoints_pub.publish(lane)
        
    def generate_lane(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        end_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:end_idx]
        if end_idx > len(self.base_waypoints.waypoints):
            print('done with all base waypoints')
        
        if self.stop_wp_idx == -1 or self.stop_wp_idx > end_idx:
            lane.waypoints = self.accelerate_waypoints(base_waypoints)# base_waypoints#
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane
    
    def accelerate_waypoints(self, waypoints_front):
        temp = []
        
        
        for i, wp in enumerate(waypoints_front):
            p = Waypoint()
            p.pose = wp.pose
            p.twist = wp.twist
            p.twist.twist.linear.x = max(5, p.twist.twist.linear.x/1.375)#max(p.twist.twist.linear.x, 0.1)#
            p.twist.twist.angular.x = 0
            p.twist.twist.angular.y = 0
            p.twist.twist.angular.z = 0
            temp.append(p)
            
        return temp
    
    def decelerate_waypoints(self, waypoints_front, closest_idx):
        temp = []
        
        
        stop_idx = max(self.stop_wp_idx - closest_idx -2, 0)
        for i, wp in enumerate(waypoints_front):
            p = Waypoint()
            p.pose = wp.pose
            d = self.distance(waypoints_front, i, stop_idx)
            vel = 0.4*d
            if vel < 2.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            p.twist.twist.angular.z = 0
            temp.append(p)
            if i > stop_idx:
                break
        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if not self.waypoints_2d and not self.waypoint_tree:
            self.base_waypoints = waypoints
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            self.bw_sub.unregister()
            

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')