#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Global publisher object
ros::Publisher coord_pub;

// define a callback function to process subcribed image
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        // convert image to opencv format
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // convert RGB to HSV
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // define blueberry HSV range
        cv::Scalar lower_blue(90, 50, 50);
        cv::Scalar upper_blue(150, 255, 255);

        // create mask
        cv::Mat mask;
        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

        // find contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            ROS_WARN("Object not found due to poor lighting conditions or no object present.");
            return;
        }

        // find the largest contour
        size_t largest_contour_index = 0;
        double largest_area = 0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;
            }
        }

        // calculte the center of the blueberry
        cv::Moments m = cv::moments(contours[largest_contour_index]);
        cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);

        // define FOV
        float HFOV = 62.2 * CV_PI / 180.0; // horizontal angle to radius
        float VFOV = 48.8 * CV_PI / 180.0; // vertical angle to radius
        float D = 0.20; // the distant between cam and ground unit meter

        // get the width and length of the cam view
        float W = 2 * D * tan(HFOV / 2.0);
        float H = 2 * D * tan(VFOV / 2.0);

        // convert the pixel coordianate to pysical coordianate
        float X = ((center.x - (image.cols / 2.0)) / image.cols) * W;
        float Y = ((center.y - (image.rows / 2.0)) / image.rows) * H;
        float Z = D; // Z always be the same

        // print the coordiantes
        ROS_INFO("Blue point position in the plane: [X: %f, Y: %f, Z: %f]", X, Y, Z);

        // publish the coordiantes
        std_msgs::String coord_msg;
        std::stringstream ss;
        ss << "X: " << X << ", Y: " << Y << ", Z: " << Z;
        coord_msg.data = ss.str();

        coord_pub.publish(coord_msg);

        // draw red box
        cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);
        cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);

        // show the coordiannates on opencv windows
        std::stringstream coord_text;
        coord_text << "X: " << X*100 << ", Y: " << Y*100; //cm
        cv::putText(image, coord_text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        // show image 
        cv::imshow("Blue Object Detection", image);

        // show mask image
        cv::imshow("Blue Object Mask", mask);
        
        cv::waitKey(1); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blue_point_detector");
    ros::NodeHandle nh;

    // Create the publisher outside the callback function

    ros::Subscriber image_sub = nh.subscribe("/raspicam_node/image/compressed", 10, imageCallback);
    
    coord_pub = nh.advertise<std_msgs::String>("blue_point_coordinates", 10);

    // create the opencv windows
    cv::namedWindow("Blue Object Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Blue Object Mask", cv::WINDOW_AUTOSIZE);

    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
