#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

// Define a global client that can request services.
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float linear_x, float angular_z)
{
    // Request a service and pass the velocities to it to drive the robot

    // Make srv object
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // ROS_INFO("Calling with linear_x: %1.2f, angular_z: %1.2f", linear_x, angular_z);

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call command_robot service in process_image.");
    }
}

// Callback function to continuously read the image data.
void process_image_callback(const sensor_msgs::Image image)
{
    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera'
    bool found = false;
    int h = -1;
    int w = -1;
    int i = -1;

    // Get the left-most, right-most, top-most, and bottom-most location
    // for white pixels. Take the average to find the "center" of the visible
    // portion of the ball.
    int h_max = INT_MIN;
    int h_min = INT_MAX;
    int w_max = INT_MIN;
    int w_min = INT_MAX;
    for (h = 0; h < image.height; h++)
    {
        for (w = 0; w < image.width; w++)
        {
            i = (h * image.step) + (w * 3);
            if (image.data[i] == white_pixel
                && image.data[i+1] == white_pixel
                && image.data[i+2] == white_pixel)
            {
                h_max = std::max(h, h_max);
                h_min = std::min(h, h_min);
                w_max = std::max(w, w_max);
                w_min = std::min(w, w_min);
                found = true;
            }
        }
    }

    // We found the ball
    if (found)
    {
        // ROS_INFO("Height: %d, Width: %d, Step: %d", image.height, image.width, image.step);
        // ROS_INFO("White pixel found at h=%d, w=%d, i=%d", h, w, i);
        // ROS_INFO("h_max: %d, h_min: %d, w_max: %d, w_min: %d", h_max, h_min, w_max, w_min);
        // ros::Duration(5).sleep();

        int w_avg = (w_max + w_min) / 2;
        int left = image.width / 3;
        int right = left * 2;

        // Figure out which direction we go.
        float linear_x = 0.0;
        float angular_z = 0.0;
        if (w_avg <= left)
        {
            linear_x = 0.20;
            angular_z = 0.20;
        }
        else if(w_avg >= left && w_avg <= right)
        {
            linear_x = 0.20;
            angular_z = 0.0;
        }
        else // w_avg > right
        {
            linear_x = 0.20;
            angular_z = -0.20;
        }

        drive_robot(linear_x, angular_z);
    }
    else
    {
        // Not found, so stop if we're moving.
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it.
    ros::init(argc, argv, "process_image");
    ros::NodeHandle node;

    // Define a client service capable of requesting services from command_robot
    client = node.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw
    ros::Subscriber cameraSubscriber = node.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}