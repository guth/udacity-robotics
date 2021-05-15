#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

// Global joint publishers
ros::Publisher joint1_publisher, joint2_publisher;

// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_joint1, float requested_joint2)
{
    // Define clamped joint angles and assign them to the requested ones
    float clamped_joint1 = requested_joint1;
    float clamped_joint2 = requested_joint2;

    // Get min and max joint parameters
    float min_joint1, max_joint1;
    float min_joint2, max_joint2;

    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;

    // Get node name
    std::string node_name = ros::this_node::getName();

    // Get joints min and max parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_joint1);
    n2.getParam(node_name + "/max_joint_1_angle", max_joint1);
    n2.getParam(node_name + "/min_joint_2_angle", min_joint2);
    n2.getParam(node_name + "/max_joint_2_angle", max_joint2);

    // Check if joint 1 falls in the safe zone. If not, clamp it.
    if (requested_joint1 < min_joint1 || requested_joint1 > max_joint1) {
        clamped_joint1 = std::max(requested_joint1, min_joint1);
        clamped_joint1 = std::min(clamped_joint1, max_joint1);
        ROS_WARN("joint1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f",
            min_joint1, max_joint1, clamped_joint1);
    }
    
    // Check if joint 2 falls in the safe zone. If not, clamp it.
    if (requested_joint2 < min_joint2 || requested_joint2 > max_joint2) {
        clamped_joint2 = std::max(requested_joint2, min_joint2);
        clamped_joint2 = std::min(clamped_joint2, max_joint2);
        ROS_WARN("joint2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f",
            min_joint2, max_joint2, clamped_joint2);
    }

    // Store clamped joint angles in a vector
    std::vector<float> clamped_data = { clamped_joint1, clamped_joint2 };

    return clamped_data;
}

// Callback function that executes whenever a safe_move service is requested
bool handle_safe_move_request(simple_arm::GoToPosition::Request& request,
    simple_arm::GoToPosition::Response& response)
{
    ROS_INFO("GoToPosition Request received - j1:%1.2f, j2:%1.2f",
        (float)request.joint_1, (float)request.joint_2);

    // Clamp angles if they are not within min/max values
    std::vector<float> joints_angles = clamp_at_boundaries(request.joint_1, request.joint_2);

    // Publish clamped angles to the arm
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_publisher.publish(joint1_angle);
    joint2_publisher.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    response.message_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + ", j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(response.message_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle node;

    // Define two publishers to publish std_msgs::Float64 to their respective topics
    joint1_publisher = node.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_publisher = node.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a "safe_move" service with a handle_safe_move_request callback function.
    ros::ServiceServer server = node.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands to safe_move");

    // Handle ROS communication events
    ros::spin();

    return 0;
}