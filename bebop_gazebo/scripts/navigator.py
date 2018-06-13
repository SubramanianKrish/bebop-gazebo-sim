#!/usr/bin/env python
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray

# Load custom modules
from planner import *

# Main function
def main(args):
	# Intiate the ROS node
	rospy.init_node('navigator', anonymous=True)
	
	# Drone control topics
	pose_cmd_pub = rospy.Publisher('/bebop2/command/pose', PoseStamped, queue_size=2)
	land = rospy.Publisher('/bebop/land', Empty, queue_size=3)
	
	# Load drone model
	drone = Drone(
					forward_speed = 1,
					yaw_cmd_value = 1,
					angular_speed = 1,
					pose_cmd_pub = pose_cmd_pub,
					dubin_omega = 1)

	# Planner parameters (All in meters)
	planner = Planner(
		min_safe_distance = 5,
		window_size = (0.5, 0.5), # (x, y)
		grid_size = (10, 10), # (x, y)
		obstacle_radius = 2,
		goal = (20, 0, math.radians(0)), # (x, y, heading)
		error_margin = 0.1,
		drone = drone,
		land = land
		)

	# Subscribe to required topics
	tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, planner.detectAprilTags)
	odometry_sub = rospy.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, updateOdometry)

	# Thread to run the planner
	print('Starting planner in 2 seconds...')
	time.sleep(2)
	planner_thread = threading.Thread(target = planner.run)
	planner_thread.start()

	try:
		rospy.spin()
	except Exception as e:
		print('Shutting Down!')

if __name__ == '__main__':
	main(sys.argv)
