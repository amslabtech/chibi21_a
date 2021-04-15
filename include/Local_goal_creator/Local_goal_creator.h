#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class Local_Goal_Creator{
    public:
        Local_Goal_Creator();
        void process();
    private:
        void global_path_callback(const nav_msgs::Path::ConstPtr&);
        void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
        void mesure_dis(const float& x1,const float& y1,const float& x2,const float& y2);
        void set_the_next_goal();

        int hz;
        float reselection_dis;
        int reselection_add_val;
        float distance = 0;
        float goal_number = 0;
        int min_index = 0.0;
        int count = 0.0;
        float max_distance = 0.0;
        bool got_global_path = false;
        bool got_current_pose = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Publisher pub_local_goal;
        ros::Subscriber sub_global_path;
        ros::Subscriber sub_current_pose;
        nav_msgs::Path global_path;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped local_goal;
};
#endif
