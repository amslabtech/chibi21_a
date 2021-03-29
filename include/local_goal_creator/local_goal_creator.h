#ifndef CHIBI_20_LOCAL_GOAL_CREATOR
#define CHIBI_20_LOCAL_GOAL_CREATOR
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
struct Coordinate
{
    int x;
    int y;
};

class Local_goal_creator
{
public:
    Local_goal_creator();
    void process();

private:
    //メンバ変数
    int hz = 0;
    bool receive_global_path = false;
    bool receive_current_pose = true;
    float local_path_index = 0;
    int ahead_value = 0;
    float limit_distance = 0;
    float distance = 0;
    Coordinate current;
    nav_msgs::Path global_path;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped local_goal;
    //メソッド
    void check_point_callback(const nav_msgs::Path::ConstPtr &);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &);
    void calc_distance(const float& gx,const float& gy,const float& cx,const float& cy);
    void calc_local_goal();

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_local_goal;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_path;
};

#endif
