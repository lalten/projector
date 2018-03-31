#ifndef CLOUD_GATEWAY_H
#define CLOUD_GATEWAY_H


#include <actionlib/server/simple_action_server.h>
#include <advanced_actionlib/advanced_action_server.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/Trigger.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/octree/octree.h>
#include <bin_picking_tools/bin_picking_tools.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CloudGateway {
public:
    CloudGateway();


    ros::NodeHandle nh_private_;
    image_transport::ImageTransport it;
    image_transport::Publisher it_pub;
    cv::Mat img;
    int w, h;

    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;

    Eigen::Affine3d eigen_t;



    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);


private:

};


#endif
