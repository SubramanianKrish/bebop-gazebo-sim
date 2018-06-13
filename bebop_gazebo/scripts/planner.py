#!/usr/bin/env python

# Import system modules
import time, threading


# Import python modules
import numpy as np
from matplotlib import pyplot as plt
import cv2

# Import ROS modeules
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

# Import custom modules
from dubins import dubins_path_planning
from drone import *

class Planner():
	"""
		- Generate a grid with all detected tags on it.
		- Detect free space on the grid and get waypoint in x wrt to drone.
		- If waypoint is 0 continue.
		- Else replan.
	"""
	def __init__(self, 
				min_safe_distance,
				window_size,
				grid_size,
				obstacle_radius,
				goal,
				error_margin,
				drone,
				land
				):
		self.min_safe_distance = min_safe_distance
		self.goal = goal
		self.error_margin = error_margin
		self.drone = drone
		self.window_size = window_size
		self.land = land
		self.grid = Grid(np.array(grid_size), obstacle_radius, min_safe_distance)
		self.control_point = 0
		self.next_waypoint = None
		self.move = False
		self.initialized = False
		self.initial_yaw = None

	def resetSearch(self):
		self.control_point = 0

	# Plan dubins curve and move drone along it
	def run(self):
		global odometry_cur
		global cur_heading
		global cur
		if not self.initialized:
			# Plan initial trajectory
			px, py, pyaw, mode, _, pathlength = dubins_path_planning(
				sy = 0,
				sx = 0,
				syaw = math.radians(0),
				ey = self.goal[1],
				ex = self.goal[0],
				eyaw = self.goal[2],
				c = 1
				)
			self.last_plan = time.time()
			self.move = True
			self.initialized = True
			print('Initialized')
			self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [px, py, pyaw])
			self.drone.thread.start()
		while self.move:
			if self.goalReached():
				self.move = False
				self.drone.kill_thread = True
			if self.next_waypoint:
				print('Replanning on next waypoint')
				# Plan a dubins curve
				quaternion = (  cur.orientation.x,
							    cur.orientation.y,
							    cur.orientation.z,
							    cur.orientation.w)
				newheading_obs = euler_from_quaternion(quaternion)[2]

				nextway_x = math.cos(newheading_obs)*self.min_safe_distance + math.sin(newheading_obs)*(self.next_waypoint) + cur.position.x
				nextway_y = -math.sin(newheading_obs)*self.min_safe_distance + math.cos(newheading_obs)*self.next_waypoint + cur.position.y
				print("next waypoint(global):", nextway_x, nextway_y)

				px, py, pyaw, mode, _, pathlength = dubins_path_planning(
					sy = cur.position.y,
					sx = cur.position.x,
					syaw = newheading_obs,
					ey = nextway_y,
					ex = nextway_x,
					eyaw = newheading_obs,
					c = 1
					)

				self.last_plan = time.time() + 3

				# Kill previous moveDrone thread
				self.drone.kill_thread = True
				self.next_waypoint = None
				
				# Start a new thread for moveDrone (use while self.kill)
				self.drone.thread.join()
				self.drone.done = False
				self.drone.kill_thread = False
				
				self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [px, py, pyaw])
				self.drone.thread.start()
				time.sleep(2.0)

			# Replan to goal if waypoint reached
			if self.drone.done or (time.time() - self.last_plan) > 3:
				print('Replanning')
				quaternion = (  cur.orientation.x,
							    cur.orientation.y,
							    cur.orientation.z,
							    cur.orientation.w)
				newheading = euler_from_quaternion(quaternion)[2]

				# Replan
				px, py, pyaw, mode, _, pathlength = dubins_path_planning(
					sy = cur.position.y,
					sx = cur.position.x,
					syaw = newheading,
					ey = self.goal[1],
					ex = self.goal[0],
					eyaw = self.goal[2],
					c = 1
					)

				self.last_plan = time.time()
				# Kill previous moveDrone thread
				self.drone.kill_thread = True
				self.next_waypoint = None
				# Start a new thread for moveDrone (use while self.kill)
				self.drone.thread.join()
				self.drone.kill_thread = False
				self.drone.done = False
				self.drone.thread = threading.Thread(target = self.drone.dubinsMoveDrone, args = [px, py, pyaw])
				self.drone.thread.start()

			# Limit the loop rate
			time.sleep(1/10)

		print('Reached Goal!')
		time.sleep(1)
		self.land.publish(Empty())

	# Check if the goal is reached
	def goalReached(self):
		margin_x = 1 #self.goal[0] * self.error_margin
		margin_y = 1 #self.goal[1] * self.error_margin
		reached_x = abs(self.goal[0] - cur.position.x) < margin_x
		reached_y = abs(self.goal[1] - cur.position.y) < margin_y
		quaternion = (  cur.orientation.x,
					    cur.orientation.y,
					    cur.orientation.z,
					    cur.orientation.w)
		headingnow = euler_from_quaternion(quaternion)[2]
		reached_heading = abs(self.goal[2] - headingnow) < math.pi/12
		if reached_x and reached_y and reached_heading:
			print(odometry_cur)
			return True

	# Callback for apriltags subscriber
	def detectAprilTags(self, tags):
		# Put all tags on a grid
		self.grid.populate(tags.detections)

		try:
			self.next_waypoint = self.getWaypoint()
			if self.waypoint:
				print(self.next_waypoint)
		except:
			pass
		# Reset the grid
		self.grid.reset()

	# Helper function to calculate waypoint
	def getWaypoint(self):
		# Slide window and check for freespace
		freespace = False
		self.resetSearch()
		while not freespace and self.control_point < self.grid.size[0]*5:
			# Select the window region
			sub_space_left, sub_space_right = self.crop()
			# Check for obstacles
			obstacles_left = np.count_nonzero(sub_space_left)
			obstacles_right = np.count_nonzero(sub_space_right)
			# Move window if there are obstacles
			if obstacles_left > 0 and obstacles_right > 0:
				self.control_point += 1
			# Return None if window was not moved, else distance of waypoint in metres.
			else:
				self.control_point *= -1 if obstacles_left > 0 else 1
				if self.control_point:
					self.control_point += -20 if obstacles_left > 0 else 20
					return self.control_point/10
				else:
					return None

		# Halt if no free space is detected
		self.move = False
		print('No free space detected. Halting!')

	def crop(self):
		y_top = int(self.grid.origin['y'] - self.window_size[1] * 10)
		y_bottom = int(self.grid.origin['y'] + self.window_size[1] * 10)
		wleft_x_left = int(self.grid.origin['x'] - self.window_size[0] * 10 - self.control_point)
		wleft_x_right = int(self.grid.origin['x'] + self.window_size[0] * 10 - self.control_point)
		wright_x_left = int(self.grid.origin['x'] - self.window_size[0] * 10 + self.control_point)
		wright_x_right = int(self.grid.origin['x'] + self.window_size[0] * 10 + self.control_point)

		left_window = self.grid.grid[y_top:y_bottom, wleft_x_left:wleft_x_right]
		right_window = self.grid.grid[y_top:y_bottom, wright_x_left:wright_x_right]

		return left_window, right_window


class Grid(object):
	def __init__(self, grid_size, obstacle_radius, min_safe_distance):
		self.obstacle_radius = obstacle_radius
		self.size = grid_size
		self.min_safe_distance = min_safe_distance
		self.origin = {
			'x': round(grid_size[0]*10/2),
			'y': round(grid_size[1]*10/2)
		}
		print(10*grid_size)
		self.grid = np.zeros(10 * grid_size)
	
	def reset(self):
		self.grid = np.zeros(10*self.size)

	def populate(self, tags):
		for tag in tags:
			
			if tag.pose.pose.position.z < self.min_safe_distance :
				
				x = round(tag.pose.pose.position.x*10) + self.origin['x']
				y = round(-tag.pose.pose.position.y*10) + self.origin['y']
				
				try:
					self.grid[int(y-self.obstacle_radius*10):int(y+self.obstacle_radius*10), int(x-self.obstacle_radius*10):int(x+self.obstacle_radius*10)] = 1
				except Exception as e:
					print('Obstacle omitted dues to padding.')
