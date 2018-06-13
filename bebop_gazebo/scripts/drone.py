#!/usr/bin/env python
from __future__ import print_function

#import python libraries
import sys
import math

#import ros libraries
import roslib
roslib.load_manifest('navigator')
import rospy

#import ros messages and functions
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

cur = Pose()
odometry_cur = {'x': 0,
				'y': 0,
				'heading': 0}
cur_heading = 0

def pose_msg(x, y, heading):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    
    quaternion = quaternion_from_euler(0,0,heading) #r,p,y
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    pose.pose.position.x =  x
    pose.pose.position.y =  y
    pose.pose.position.z = 1
    return pose

# Callback for odometry
def updateOdometry(odometry):

	global cur_heading
	# Update the value of self.odometry
	quaternion = []
	quaternion.append(odometry.pose.pose.orientation.x)
	quaternion.append(odometry.pose.pose.orientation.y)
	quaternion.append(odometry.pose.pose.orientation.z)
	quaternion.append(odometry.pose.pose.orientation.w)
	cur_heading = euler_from_quaternion(quaternion)[2]

	global cur
	cur.position.x = odometry.pose.pose.position.x
	cur.position.y = odometry.pose.pose.position.y
	cur.orientation = odometry.pose.pose.orientation

	global odometry_cur
	odometry_cur = {
		'x': odometry.pose.pose.position.x,
		'y': odometry.pose.pose.position.y,
		'heading': cur_heading
	}

def check_reach(waypoint):
    
    global cur_heading

    quaternion = (
    waypoint.pose.orientation.x,
    waypoint.pose.orientation.y,
    waypoint.pose.orientation.z,
    waypoint.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    waypoint_heading = euler[2]

    if(((waypoint.pose.position.x - 0.2) <= cur.position.x <= (waypoint.pose.position.x + 0.2)) and ((waypoint.pose.position.y - 0.2) <= cur.position.y <= (waypoint.pose.position.y + 0.2)) and ((math.degrees(waypoint_heading) - 5) <= math.degrees(cur_heading) <= math.degrees(waypoint_heading) + 5)):
		#print("Check Reach -- x: ", waypoint.pose.position.x, "Y:", waypoint.pose.position.y, "\n")
		return True
    else:
		return False

class Drone(object):
	"""docstring for [object Object]."""
	def __init__(self,
				 forward_speed,
				 yaw_cmd_value,
				 angular_speed,
				 pose_cmd_pub,
		 		 dubin_omega,
				 ):
		self.forward_speed = forward_speed
		self.yaw_cmd_value = math.pi/6
		self.angular_speed = angular_speed
		self.pose_cmd_pub = pose_cmd_pub
		self.depth_compensation = forward_speed * 0.9 / angular_speed
		self.dubin_omega = 1
		self.kill_thread = False
		self.done = False

	def dubinsMoveDrone(self, px, py, pyaw):
		waypoints = []
    	# px, py, pyaw, mode, clen, pathlength = dubins_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
		for i in range(len(px)):
			waypoints.append(pose_msg(px[i], py[i], pyaw[i]))

		for i in range(len(waypoints)):
			waypoint_reached = False
			while(not waypoint_reached and not self.kill_thread):
				self.pose_cmd_pub.publish(waypoints[i])
				waypoint_reached = check_reach(waypoints[i])
				# time.sleep(0.1)

		print("waypoint intermediate Reached :D")
		self.done = True