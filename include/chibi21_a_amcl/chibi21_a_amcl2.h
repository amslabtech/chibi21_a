#ifndef CHIBI21_A_AMCL2_H
#define CHIBI21_A_AMCL2_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <geometry_msgs/PoseArray.h>
//#include <std_srvs/Empty.h>
//#include <nav_msgs/SetMap.h>
//#include <nav_msgs/GetMap.h>

#include <math.h>
#include <random>
#include <vector>
#include <iostream>

class AMCL
{
    public:
        AMCL();
        void process();

    private:
        class Particle
        {
            public:
                Particle(int N);
                geometry_msgs::PoseStamped pose;
                tf::Quaternion init_orientation;  //ばらまいた直後のorientation、動作更新の際の進行方向に用いる
                double weight;

                // AMCL* amcl;

                // void p_init(double x, double y, double yaw, double cov_x, double cov_y, double cov_yaw);
                // void p_move(nav_msgs::Odometry current_odo, nav_msgs::Odometry previous_odo);
                // void p_calc_weight(sensor_msgs::LaserScan laserscan, nav_msgs::OccupancyGrid map);
                // double get_wall_range(double laser_angle, nav_msgs::OccupancyGrid map);
                // bool is_wall(double x, double y, nav_msgs::OccupancyGrid map);
        };

        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

         void p_init(AMCL::Particle &p);
        void p_motion_update();
         void p_move(AMCL::Particle &p);
        void p_measurement_update();
         void p_calc_weight(AMCL::Particle &p);
         double get_wall_range(double laser_angle, AMCL::Particle p);
         bool is_wall(double x, double y);
        void weight_normalize();
        void resampling_process();
        double calc_ess();
        void resampling();

        void p_disp_update();   //rviz表示用

        //parameter
        int N;
        double INIT_X;          //パーティクル初期中心位置
        double INIT_Y;          //
        double INIT_YAW;        //
        double INIT_X_COV;      //パーティクルのばらつき
        double INIT_Y_COV;      //
        double INIT_YAW_COV;    //
        double ANGLE_INC;       //レーザーの値を何本間隔で参照するか
        double MAX_RANGE;       //壁までの距離を測る際の限界距離
        double CHECK_INTERVAL;  //壁を探す際の距離の間隔
        double M_WEIGHT;        //尤度計算に使う．現状意味なし
        double M_COV;           //
        double ESS_LIMEN;       //ESSのしきい値

        int hz;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_odometry;
        ros::Subscriber sub_laserscan;
        ros::Subscriber sub_map;

        ros::Publisher pub_estimated_pose;
        ros::Publisher pub_p_poses;

        geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::PoseStamped previous_pose;
        geometry_msgs::PoseArray poses;

        nav_msgs::Odometry current_odo;
        nav_msgs::Odometry previous_odo;

        std::vector<Particle> particles;
        // std::vector<Particle> init_particles;

        nav_msgs::Odometry odometry;
        sensor_msgs::LaserScan laserscan;
        nav_msgs::OccupancyGrid map;

        bool odo_get;
        bool odo_move;     //動いていたらtrue
        bool scan_get;
        bool map_get;

        bool first_update;  //最初の更新の前だけprevious_odo=current_odoとするためのフラグ

        double max_weight;
};

#endif
