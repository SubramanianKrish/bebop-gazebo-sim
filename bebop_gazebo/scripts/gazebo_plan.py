#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from math import degrees, radians
from dubins import *

cur = Pose()

def pose_msg(x, y, heading):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    
    quaternion = quaternion_from_euler(0,0,heading) #r,p,y
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 1
    return pose

def callback(pose):
    global cur
    cur.position.x = pose.position.x
    cur.position.y = pose.position.y
    cur.orientation = pose.orientation

def check_reach(waypoint):
    
    quaternion = (
    cur.orientation.x,
    cur.orientation.y,
    cur.orientation.z,
    cur.orientation.w)
    euler = euler_from_quaternion(quaternion)
    cur_heading = euler[2]

    quaternion = (
    waypoint.pose.orientation.x,
    waypoint.pose.orientation.y,
    waypoint.pose.orientation.z,
    waypoint.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    waypoint_heading = euler[2]

    if(((waypoint.pose.position.x - 0.2) <= cur.position.x <= (waypoint.pose.position.x + 0.2)) and ((waypoint.pose.position.y - 0.2) <= cur.position.y <= (waypoint.pose.position.y + 0.2)) and ((degrees(waypoint_heading) - 5) <= degrees(cur_heading) <= degrees(waypoint_heading) + 5)):
        print "current: ", degrees(cur_heading), "target: ", degrees(waypoint_heading)
        return True
    else:
        return False
    
def start_motion():
    pub = rospy.Publisher('/bebop2/command/pose', PoseStamped, queue_size=10)
    rospy.init_node('gazebo_planner', anonymous=True)
    rospy.Subscriber("/bebop2/odometry_sensor1/pose", Pose, callback)
    rate = rospy.Rate(10) # 10hz
   
    time.sleep(0.1)

    waypoints = []
    # px, py, pyaw, mode, clen, pathlength = dubins_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    px, py, pyaw, mode, clen, pathlength = dubins_path_planning(0, 0, radians(0), 0, 5, radians(0), 1)
    for i in range(len(px)):
        waypoints.append(pose_msg(px[i],py[i],pyaw[i]))

    for i in range(len(waypoints)):
        waypoint_reached = False
        while(not waypoint_reached):
            pub.publish(waypoints[i])
            waypoint_reached = check_reach(waypoints[i])
            # time.sleep(0.1)

    print "Goal Reached :D"
            
if __name__ == '__main__':
    try:
        start_motion()
    except rospy.ROSInterruptException:
        pass