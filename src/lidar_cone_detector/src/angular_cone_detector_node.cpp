#include "lidar_cone_detector/angular_cone_detector.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "angular_cone_detector_node");
    ros::NodeHandle nh;
    float rate;
    nh.param<float>("angular_cone_detector_node/rate", rate, 10);

    AngularConeDetector cone_detector(nh, rate);
    ros::spin();

    // ros::Rate r(rate);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     r.sleep();
    // }
    return 0;
}