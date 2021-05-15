#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& request,
    ball_chaser::DriveToTarget::Response& response)
{
    // ROS_INFO("Drive request received: (%1.2fx, %1.2fz)", (float)request.linear_x, (float)request.angular_z);

    // Create the motor_command object
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = request.linear_x;
    motor_command.angular.z = request.angular_z;

    // Publish the angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    response.message_feedback = "Velocity set - x: " + std::to_string(motor_command.linear.x) + ", z: " + std::to_string(motor_command.angular.z);
    
    ROS_INFO_STREAM(response.message_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle node;

    // Inform ROS master that we will be publishing a message of type
    // geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_Robot service with a handle_drive_request callback
    ros::ServiceServer server = node.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send joint commands to /ball_chaser/command_robot");

    // Handle ROS communication events
    ros::spin();

    return 0;
}