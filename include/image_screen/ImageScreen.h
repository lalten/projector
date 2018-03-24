#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

class ImageScreen
{
public:
  
  ImageScreen();
  
  void set_black();
  void show_image(const cv::Mat& img);
  void close_image_window();
  
private:
  ros::NodeHandle nh_private_;
  ros::Subscriber img_sub_;
  ros::Subscriber toggle_sub_;
  ros::Subscriber black_sub_;
  std::string display_name_;
  bool window_is_open_;
  cv::Size img_size_;
  cv::Mat black_img_;
  
  void black_cb_(const std_msgs::EmptyConstPtr& msg);
  void toggle_cb_(const std_msgs::BoolConstPtr& msg);
  void open_image_window();
  void img_cb_(const sensor_msgs::ImageConstPtr& msg);
};
