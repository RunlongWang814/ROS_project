#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// Global publisher object
ros::Publisher coord_pub;

// 定义一个回调函数，用于处理订阅到的图像
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try
    {
        // 将压缩图像消息转换为OpenCV图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        // 转换为HSV色彩空间
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义蓝色的HSV范围
        cv::Scalar lower_blue(90, 50, 50);
        cv::Scalar upper_blue(150, 255, 255);

        // 创建掩码
        cv::Mat mask;
        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

        // 查找轮廓
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            // 如果没有找到任何轮廓，打印对象未找到的消息
            ROS_WARN("Object not found due to poor lighting conditions or no object present.");
            return;
        }

        // 找到最大的轮廓
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

        // 计算蓝点的质心
        cv::Moments m = cv::moments(contours[largest_contour_index]);
        cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);

        // 定义视场角和距离（以米为单位）
        float HFOV = 62.2 * CV_PI / 180.0; // 水平视场角，转换为弧度
        float VFOV = 48.8 * CV_PI / 180.0; // 垂直视场角，转换为弧度
        float D = 0.20; // 摄像头距离平面的距离，单位：米

        // 计算图像平面的宽度和高度
        float W = 2 * D * tan(HFOV / 2.0);
        float H = 2 * D * tan(VFOV / 2.0);

        // 将像素坐标转换为物理平面坐标
        float X = ((center.x - (image.cols / 2.0)) / image.cols) * W;
        float Y = ((center.y - (image.rows / 2.0)) / image.rows) * H;
        float Z = D; // Z 保持为摄像头与平面的距离

        // 打印蓝点的平面坐标
        ROS_INFO("Blue point position in the plane: [X: %f, Y: %f, Z: %f]", X, Y, Z);

        // 发布蓝点坐标
        std_msgs::String coord_msg;
        std::stringstream ss;
        ss << "X: " << X << ", Y: " << Y << ", Z: " << Z;
        coord_msg.data = ss.str();

        coord_pub.publish(coord_msg);

        // 绘制包围蓝点的红色矩形框
        cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);
        cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2);

        // 在图像上显示X和Y值
        std::stringstream coord_text;
        coord_text << "X: " << X*100 << ", Y: " << Y*100; //cm
        cv::putText(image, coord_text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        // 显示带有红色矩形框和坐标值的图像
        cv::imshow("Blue Object Detection", image);

        // 显示掩码图像
        cv::imshow("Blue Object Mask", mask);
        
        cv::waitKey(1); // 添加一个短暂的延迟以显示图像
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

    // 创建显示窗口
    cv::namedWindow("Blue Object Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Blue Object Mask", cv::WINDOW_AUTOSIZE);

    ros::spin();

    // 关闭窗口
    cv::destroyAllWindows();

    return 0;
}
