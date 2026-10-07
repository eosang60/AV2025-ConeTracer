#include "midpoint_tracer/midpoint_tracer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "midpoint_tracer_node");
    ros::NodeHandle nh;

    MidpointTracer midpoint_tracer(nh);
    ros::spin();

    return 0;
}
