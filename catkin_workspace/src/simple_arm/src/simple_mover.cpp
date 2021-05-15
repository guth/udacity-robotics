#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    // Initialize the arm_mover node
    ros::init(argc, argv, "arm_mover");

    // Create a handle to the arm_mover node
    ros::NodeHandle node;

    // Create a publisher that can publish a std_msgs::Float64 message
    // on the /simple_arm/joint_1_position_controller/command topic
    ros::Publisher joint1Publisher = node.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);

    // Create a publisher that can publish a std_msgs::Float64 message
    // on the /simple_arm/joint_2_position_controller/command topic
    ros::Publisher joint2Publisher = node.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Set loop frequency of 10Hz
    ros::Rate loopRate(10);

    int startTime;
    int elapsed;

    // Get ROS start time
    while (!startTime)
    {
        startTime = ros::Time::now().toSec();
    }

    while (ros::ok())
    {
        // Get ROS elapsed time
        elapsed = ros::Time::now().toSec() - startTime;

        // Set the arm joint angles
        std_msgs::Float64 joint1_angle, joint2_angle;
        joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

        // Publish the arm joint angles
        joint1Publisher.publish(joint1_angle);
        joint2Publisher.publish(joint2_angle);

        // Sleep for the time remaining until 10Hz is reached
        loopRate.sleep();
    }

    return 0;
}