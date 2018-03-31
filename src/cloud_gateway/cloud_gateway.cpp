//
// Created by engelhard on 3/24/18.
//

#include <cloud_gateway/cloud_gateway.h>

#include <pcl_conversions/pcl_conversions.h>


CloudGateway::CloudGateway():
    nh_private_("~"),
    it(nh_private_)
{
    std::string cloud_topic = "/camera/depth/points";
    sub_cloud = nh_private_.subscribe(cloud_topic, 1, &CloudGateway::cloud_cb, this);
    pub_cloud = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ> >("rotated_cloud", 10);

    it_pub = it.advertise("flat/image", 1);

    w = 1920/4;
    h = 1200/4;
    img = cv::Mat(h, w, CV_8UC3);

    tf2_ros::BufferClient buffer_client("/tf2_buffer_server", 100);

    geometry_msgs::TransformStamped tfs;
    try
    {
        if (!buffer_client.waitForServer(ros::Duration(1)))
        {
            ROS_WARN("No server at /tf2_buffer_server (needed for bin_picking_tools/CloudTransformer");
        }
        tfs = buffer_client.lookupTransform("ASD", "camera_depth_optical_frame", ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException& e)
    {
        ROS_WARN("Could not transform: %s", e.what());
    }

    tf::transformMsgToEigen(tfs.transform, eigen_t);

//    cloud.header.frame_id = target_frame;
};

void CloudGateway::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ROS_INFO_THROTTLE(1, "GOT CLOUD");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::string in_frame = cloud.header.frame_id;
//    bin_picking_tools::CloudTransformer ctf;
//    ctf.transform_to_frame(cloud, "ASD");
    pcl::transformPointCloud(cloud, cloud, eigen_t);

    cloud.header.frame_id = in_frame;

    img.setTo(0);
    double scale;
    nh_private_.param<double>("scale", scale, 4.0);
    float m2px = w/scale;
//    ROS_INFO("scale %f", scale);


    double min_z;
    nh_private_.param<double>("min_z", min_z, 2.0);

    double max_z;
    nh_private_.param<double>("max_z", max_z, 4.0);

    ROS_INFO("Z range: %.2f to %.2f", min_z, max_z);

    for (size_t i=0; i<cloud.size(); i += 1)
    {
        const pcl::PointXYZ& p = cloud[i];
        int x_px = w - (p.x*m2px + w/2);
        int y_px = p.y*m2px + w/2;


        if (x_px < 0 || y_px < 0 || x_px >= w || y_px >= h)
        {
            continue;
        }

        // img.at<cv::Vec3b>(y_px, x_px) = cv::Vec3b(50, 200, 0);

        cv::Scalar col = cv::Scalar(200, 200, 200);

        if (p.z < min_z || p.z > max_z)
        {
            col = cv::Scalar(50, 50, 50);
        }

        cv::circle(img, cv::Point(x_px, y_px), 2, col, -1);
    }

//    cv::imwrite("/tmp/asd.png", img);

    sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    it_pub.publish(msg_out);
    // pub_cloud.publish(cloud);
}
