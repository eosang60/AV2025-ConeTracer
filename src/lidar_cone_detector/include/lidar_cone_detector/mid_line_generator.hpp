#ifndef MID_LINE_GENERATOR
#define MID_LINE_GENERATOR

#include <ros/ros.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class LineGenerator
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_left_cone;
    ros::Subscriber sub_right_cone;
    ros::Publisher pub_mid_line;

    visualization_msgs::MarkerArrayConstPtr left_cone;
    visualization_msgs::MarkerArrayConstPtr right_cone;
    visualization_msgs::Marker mid_line;

    std::string sub_left_topic;
    std::string sub_right_topic;
    std::string pub_mid_topic;

    std_msgs::Header header;

    float rate;
    float cone_distance;
    float cone_distance_threshold;
    float past_dx;
    float past_dy;
    float dx;
    float dy;

public:
    bool sub_left;
    bool sub_right;

    LineGenerator(const ros::NodeHandle &nh_, const float r);
    void LeftConeCallBack(const visualization_msgs::MarkerArrayConstPtr &msg);
    void RightConeCallBack(const visualization_msgs::MarkerArrayConstPtr &msg);
    void CalculateMidLine();
    void CalculateByLeft();
    void CalculateByRight();
    geometry_msgs::Point SetMarker(Eigen::Vector2f p);
    // void PublishMidLine();
    void ResetData();
};
#endif // MID_LINE_GENERATOR