#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
public:
    ProcessImage(ros::NodeHandle &n);
    void Callback(const sensor_msgs::Image img);
    void DriveRobot(float lin_x, float ang_z);
private:
    ros::ServiceClient client_;
    ros::NodeHandle n_;
    ros::Subscriber sub_;

};

