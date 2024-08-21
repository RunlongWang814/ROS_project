#include <ros/ros.h>
#include <std_msgs/String.h>
#include <link_service/BluePoint.h> // include .srv flie
#include <sstream>
#include <string>

// Global variables to store the latest blue point coordinates
float latest_x = 0.0;
float latest_y = 0.0;
float latest_z = 0.0;

// Callback function to receive blue point coordinates messages
void bluePointCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string coord_str = msg->data;
    ROS_INFO("Received raw coordinate string: %s", coord_str.c_str());

    std::stringstream ss(coord_str);
    std::string temp;

    // Parsing the X coordinate
    ss >> temp >> latest_x;
    // Removing the comma and space
    ss.ignore(2);

    // Parsing the Y coordinate
    ss >> temp >> latest_y;
    // Removing the comma and space
    ss.ignore(2);

    // Parsing the Z coordinate
    ss >> temp >> latest_z;

    ROS_INFO("Parsed coordinates: X: %f, Y: %f, Z: %f", latest_x, latest_y, latest_z);
}

// Service callback function to respond to service requests
bool getBluePoint(link_service::BluePoint::Request &req,
                  link_service::BluePoint::Response &res)
{
    // Set the response coordinates to the latest received coordinates
    res.X = latest_x;
    res.Y = latest_y;
    res.Z = latest_z;

    ROS_INFO("Service request received. Sending coordinates: X: %f, Y: %f, Z: %f", latest_x, latest_y, latest_z);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blue_point_service_server");
    ros::NodeHandle nh;

    // Subscribe to the blue point coordinates topic
    ros::Subscriber sub = nh.subscribe("blue_point_coordinates", 10, bluePointCallback);

    // Advertise the service named "get_blue_point"
    ros::ServiceServer service = nh.advertiseService("get_blue_point", getBluePoint);

    ROS_INFO("Blue point service server is ready.");

    ros::spin();  // Keep the node running and processing callbacks

    return 0;
}
