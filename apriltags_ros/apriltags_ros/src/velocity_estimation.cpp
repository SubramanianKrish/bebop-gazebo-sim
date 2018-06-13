#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <geometry_msgs/Twist.h>

double prev_position[3] = {0};
ros::Publisher vel_pub;
double prevtime=0;
double prevvely=0;
double prevvelx=0;
double prevvelz=0;
void getVelocities(const apriltags_ros::AprilTagDetectionArray msg)
{
    double currtime =ros::Time::now().toSec();
    geometry_msgs::Twist computed_velocity;
    if(!msg.detections.empty())
    {
        computed_velocity.linear.x =(msg.detections[0].pose.pose.position.x - prev_position[0])/(currtime-prevtime);
        computed_velocity.linear.y =(msg.detections[0].pose.pose.position.y - prev_position[1])/(currtime-prevtime);
        computed_velocity.linear.z = (msg.detections[0].pose.pose.position.z - prev_position[2])/(currtime-prevtime);
        // prevvelx=computed_velocity.linear.x;
        // prevvely=computed_velocity.linear.y;
        // prevvelz=computed_velocity.linear.z;
      
        prev_position[0] = msg.detections[0].pose.pose.position.x;
        prev_position[1] = msg.detections[0].pose.pose.position.y;
        prev_position[2] = msg.detections[0].pose.pose.position.z;
        vel_pub.publish(computed_velocity);
        prevtime=currtime;
    }
    // catch(const std::exception &e) {

    // }
   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltags");
    ros::NodeHandle nh;
    prevtime=ros::Time::now().toSec();
    ros::Subscriber tag_sub = nh.subscribe("tag_detections", 10, getVelocities);
    // Velocity publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/computed_velocity",1000);
    ros::spin();
    ros::spin();
    return 0;
}