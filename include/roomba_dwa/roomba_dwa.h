#ifndef ROOMBADWA_H
#define ROOMBADWA_H

#include<ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tf.h"

struct Roomba_state
{
    float x;
    float y;
    float yaw;
    float v;
    float omega;
};

struct Coodinate
{
    int x;
    int y;
};

struct Move
{
    float v;
    float omega;
};

class DWA
{
    public:
    DWA();
    void process();

private:
    void roomba();
    //void ranges_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void set_map();
    void calc_dynamic_window(const Roomba_state &state);
    void calc_trajectory(const float &v ,const float &y);
    float calc_to_goal_cost();
    float calc_obstacle_cost();
    void calc_final_input();
    void dwa_control();

    int hz;
    int x=0;
    int y = 0;
    float dt;
    float mass=100;
    float mass_reso=0.05; //[m]
    std::vector<std::vector<int>> map;
    int safemass_x;
    int safemass_y;

    float max_speed = 1.0; //[m/s]
    float min_speed = 0; //-0.5[m/s]
    float max_accel = 0.4;
    float v_reso = 0.01;
    float predict_time = 0.0;
    float max_yawrate = 40.0*M_PI/180; //<=1
    float max_dyawrate = 10.0*M_PI/180; //<=1
    float omega_reso = 0.1*M_PI/180;
    float to_goal_gain = 1.0; //α
    float robot_distance_gain = 1.0; //β
    float speed_gain = 1.0; //γ
    float speed_cost = 0.0;
    float safe_space=20;
    float safe_space3=10;
    float min_cost = 0;
    float final_cost = 0;
    float minr = 0;
    float distance = 0;
    float to_goal_cost = 0.0;
    float ob_cost = 0.0;
    float goal_magnitude = 0.0;
    float traj_magnitude = 0.0;
    float inner_product = 0.0;
    float inner_product_angle = 0.0;

    //float max_heading=0;
    //float max_dist=0;
    //float max_velocity=0;

    double roll=0;
    double pitch=0;
    double yaw=0;
    int row = 0;
    int column = 0;
    float resolution = 0;
    float roomba_radius = 0.3;
    float theta = 0.0;
    float cost = 0.0;
    bool receive_local_map = false;
    std::vector<float> dw;
    Roomba_state state = {0.0,0.0,0.0,0.0,0.0};
    Move move;
    Move min_m;
    std::vector<Roomba_state> traj,best_traj;
    std::vector<float> goal;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_local_goal;
    ros::Subscriber sub_local_map;
    ros::Subscriber sub_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_predict_path;
    nav_msgs::Path path;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::Path all_predict_path;
    nav_msgs::Path predict_path;
    geometry_msgs::PoseStamped pose;
};
#endif


