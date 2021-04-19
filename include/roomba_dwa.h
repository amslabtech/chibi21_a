#ifndef ROOMBADWA_H
#define ROOMBADWA_H

#include<ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"

class DWA
{
    public:
    DWA();
    void process();

private:
    void roomba();
    //void ranges_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    //void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void point_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void path_callback(const nav_msgs::Path::ConstPtr &msg);
    void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void set_map();
    //std::vector<std::vector<int>> Raycast();

    int hz;
    float dt;
    float mass=100;
    float mass_reso=0.05; //[m]

    std::vector<std::vector<int>> map;

    float range_min=180;
    float range_max=900;
    float range_reso=20;
    float sensor_theta;
    int obstacle_x;
    int obstacle_y;
    int safemass_x;
    int safemass_y;
    float old_pose_x;
    float old_pose_y;
    float now_pose_x;
    float now_pose_y;
    float now_speed;
    float old_angle;
    float now_angle;
    float now_yawrate;
    float v2_max;
    float v2_min;
    float omega2_max;
    float omega2_min;
    float theta=0;
    float max_speed = 1.0; //[m/s]
    float min_speed = 0; //[m/s]
    float max_accel = 1;
    float v_reso = 0.05;
    float predict_time =3.0;
    float max_yawrate = 40.0*M_PI/180; //<=1
    float max_dyawrate = 40.0*M_PI/180; //<=1
    float omega_reso =2*M_PI/180;
    float heading_theta = 0;
    float heading=0;
    float dist=0;
    float velocity=0;
    float to_goal_gain = 1.0; //α
    float robot_distance_gain = 1.0; //β
    float speed_gain = 1.0; //γ
    int x; //次の時刻におけるｘ座標//ひげの先端位置
    int y; //次の時刻におけるｙ座標
    float x2;
    float y2;
    float fin_v=0;
    float fin_omega=0;
    float fin_y=0;
    float fin_x=0;
    float evaluation;
    float max_evaluation=0;
    float obstacle;
    float min_obstacle=14.14;
    float safe_space=10;
    //float max_heading=0;
    //float max_dist=0;
    //float max_velocity=0;

    double roll=0;
    double pitch=0;
    double yaw=0;
    int row;
    int column;
    float resolution = 0;
    bool receive_local_map = false;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_point;
    ros::Subscriber sub_path;
    ros::Subscriber sub_local_goal;
    //ros::Subscriber sub_range;
    ros::Subscriber sub_local_map;
    ros::Subscriber sub_estimated_pose;
    ros::Subscriber sub_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_hige;
    ros::Publisher pub_goal_point;
    ros::Publisher pub_best_predict_path;
    ros::Publisher pub_ob_check;
    ros::Publisher pub_predict_path;
    //ros::Publisher pub_local_map;
    nav_msgs::Path path;
    nav_msgs::OccupancyGrid local_map;
    nav_msgs::Odometry point;
    geometry_msgs::PoseStamped local_goal;
    geometry_msgs::PoseStamped estimated_pose;
    geometry_msgs::PointStamped goal_point;
    sensor_msgs::LaserScan ranges;
    geometry_msgs::PoseStamped pose;
};
#endif


