#ifndef _LOCALIZER_H
#define _LOCALIZER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <random>
#include <math.h>
#include <vector>

class Particle
{
public:
    Particle();
    void process();
    geometry_msgs::PoseStamped pose;
    double weight;

private:
    //method
    void p_init(double,double,double,double,double,double);
    void p_motion_update(geometry_msgs::PoseStamped,geometry_msgs::PoseStamped);
    void p_measurement_update();
    void p_move(double,double,double);

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr&);

    int index(double,double);                       //indexの計算
    int grid_data(double,double);                   //Griddataの取得
    double get_Yaw(geometry_msgs::Quaternion);      //Yaw取得
    double angle_diff(double,double);               //角度差の算出
    double get_Range(double,double,double);         //Rangeの更新

    void p_spread(double*,double*,double*);         //particleを散布し,分散を再設定
    void move_process();                            //particleの移動処理
    void measurement_process(int*);                 //particleの尤度処理
    void resampling_process(int);                   //resampling処理
    void estimated_pose_process();                  //推定位置の算出
    void create_new_cov(double*,double*,double*);   //新たな分散を算出

    //parameter
    int N;
    double INIT_X;
    double INIT_Y;
    double INIT_YAW;
    double INIT_X_COV;
    double INIT_Y_COV;
    double INIT_YAW_COV;
    double MAX_RANGE;
    int RANGE_STEP;
    double X_COV_TH;
    double Y_COV_TH;
    double YAW_COV_TH;
    double ALPHA_1;
    double ALPHA_2;
    double ALPHA_3;
    double ALPHA_4;
    double ALPHA_SLOW;
    double ALPHA_FAST;
    double HIT_COV;
    double LAMBDA_SHORT;
    double Z_HIT;
    double Z_SHORT;
    double Z_MAX;
    double Z_RAND;
    double JUDGE_DISTANCE_VALUE;
    double JUDGE_ANGLE_VALUE;
    double SELECTION_RATIO;

    int Hz;
    double x_cov;
    double y_cov;
    double yaw_cov;
    double weight_slow;
    double weight_fast;

    bool get_map = false;      //mapを取得したかどうかの判定
    bool update_flag = false;     //更新するかどうかの判定

    std::vector<Particle> particles;
    std::vector<Particle> sort_particles;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber map_sub;
    ros::Subscriber lsr_sub;
    ros::Subscriber odo_sub;

    ros::Publisher estimated_pose_pub;
    ros::Publisher estimated_poses_pub;

    geometry_msgs::PoseStamped estimated_pose;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped previous_pose;
    geometry_msgs::PoseArray poses;
    nav_msgs::Odometry odometry;
    nav_msgs::OccupancyGrid map;
    sensor_msgs::LaserScan laser;

};

#endif  //LOCALIZER_H
