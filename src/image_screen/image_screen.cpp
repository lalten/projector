#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "image_screen/ImageScreen.h"


ImageScreen::ImageScreen():
nh_private_("~"),
display_name_("projection_window"),
window_is_open_(false)
{
  nh_private_.param<int>("/projector_width", img_size_.width, 1920);
  nh_private_.param<int>("/projector_height", img_size_.height, 1200);
  
  ROS_INFO("Expecting projector size of  %i %i", img_size_.width, img_size_.height);
  
  black_img_ = cv::Mat(img_size_.height, img_size_.width, CV_8UC1);
  black_img_.setTo(0);
  
  img_sub_ = nh_private_.subscribe("/game_window", 1, &ImageScreen::img_cb_, this);
  toggle_sub_ = nh_private_.subscribe("activate", 1, &ImageScreen::toggle_cb_, this);
  toggle_sub_ = nh_private_.subscribe("black", 1, &ImageScreen::black_cb_, this);
  
  open_image_window();
  cv::imshow(display_name_, black_img_);
  cv::waitKey(1);
}


void ImageScreen::black_cb_(const std_msgs::EmptyConstPtr& msg)
{
  ROS_INFO("Going dark");
  set_black();
}

void ImageScreen::toggle_cb_(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
  {
    open_image_window();
  }else
  {
    close_image_window();
  }
}

void ImageScreen::open_image_window()
{
  if (window_is_open_)
    return;
  window_is_open_ = true;
  
  ROS_INFO("Opening window");
  cv::namedWindow(display_name_, CV_WINDOW_NORMAL); // CV_WINDOW_AUTOSIZE);
  cv::moveWindow(display_name_, 1920*1.5, 100);
  cv::setWindowProperty(display_name_,CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  cv::waitKey(10);
}

void ImageScreen::close_image_window()
{
  if (!window_is_open_)
    return;
  
  window_is_open_ = false;
  ROS_INFO("Closing image");
  cv::destroyWindow(display_name_);
}

void ImageScreen::show_image(const cv::Mat& img)
{
  open_image_window();
  cv::imshow(display_name_, img);
  cv::waitKey(2);
}

void ImageScreen::set_black()
{
  show_image(black_img_);
}


void ImageScreen::img_cb_(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    //        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  show_image(cv_ptr->image);
  // cv::imwrite("/tmp/received.jpg", cv_ptr->image);
  // cv::imshow(display_name_, cv_ptr->image);
  // cv::waitKey(2);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_screen");

  ImageScreen screen;
  ros::spin();
  return EXIT_SUCCESS;
}
