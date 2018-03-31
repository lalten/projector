
#include <cloud_gateway/cloud_gateway.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CloudGateWay");

    CloudGateway cgw;
    ros::spin();
}
