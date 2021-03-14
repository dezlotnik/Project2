#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#include "process_image.h"

ProcessImage::ProcessImage(ros::NodeHandle &n) : n_(n) {
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::Callback, this);
}

void ProcessImage::Callback(const sensor_msgs::Image img) {
    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int size = img.height * img.step;
    int row_size = img.step;
    int n_left = 0;
    int n_center = 0;
    int n_right = 0;
    bool ball_found = false;
    for (int i = 0; i < size; i+=3) {
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            ball_found = true;
            if (i % row_size <= row_size/3) {
                n_left++;
            } else if (i % row_size <= 2*row_size/3) {
                n_center++;
            } else {
                n_right++;
            }
        }
    }

    if (ball_found) {
        if (n_center >= n_left && n_center >= n_right) {
            DriveRobot(0.25,0.0);
        } else if (n_left >= n_right ) {
            DriveRobot(0.0,0.25);
        } else {
            DriveRobot(0.0,-0.25);
        }
    } else {
        DriveRobot(0.0,0.0);
    }
}

void ProcessImage::DriveRobot(float lin_x, float ang_z) {
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    client_.call(srv);
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ProcessImage process_image(n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}