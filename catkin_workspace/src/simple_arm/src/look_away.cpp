#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

// Define global vector of joints last position, moving state of the arm,
// and the client that can request services std::vector<double> joints_last_position{ 0, 0 };
std::vector<double> joints_last_position{0, 0};
bool moving_state = false;
ros::ServiceClient client;

// Function to call safe_move to safely move the arm to the center position
void move_arm_to_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles [1.57, 1.57]
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call the safe_move service");
    }
}

// This callback continuously executes and reads the arm joint angles position
void joint_states_callback(const sensor_msgs::JointState jointState)
{
    // Get joints current position
    std::vector<double> joints_current_position = jointState.position;

    // Define a tolerance threshhold for comparing doubles
    double tolerance = 0.0005;

    // Check if arm is moving by comparing its current joints position to its latest
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance
        &&  fabs(joints_current_position[1] - joints_last_position[1]) < tolerance)
    {
        moving_state = false;
    }
    else
    {
        moving_state = true;
        joints_last_position = joints_current_position;
    }
}

// This callback function continuously reads the image data
void look_away_callback(const sensor_msgs::Image image)
{
    bool uniform_image = true;

    // Check each pixel to see if they're all the same
    for(int i=0; i < image.height * image.step; i++)
    {
        if (image.data[i] != image.data[0])
        {
            uniform_image = false;
            break;
        }
    }

    // If the image is uniform and the arm is not moving,
    // then move the arm to the center
    if (uniform_image && !moving_state)
    {
        move_arm_to_center();
    }
}

int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it.
    ros::init(argc, argv, "look_away");
    ros::NodeHandle node;

    // Define a client service to call safe_move
    client = node.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    // Subscribe to /simple_arm/joint_states topic to read the arm joints position
    ros::Subscriber sub1 = node.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

    // Subscribe to rgb_camera/image_raw to read the image data
    ros::Subscriber sub2 = node.subscribe("rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}