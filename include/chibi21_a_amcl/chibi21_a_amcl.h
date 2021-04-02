#ifndef CHIBI21_A_AMCL_H
#define CHIBI21_A_AMCL_H

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

class Particle
{
    public:
        Particle();
        void process();
        geometry_msgs::PoseStamped pose;
        double weight;

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

        void p_init(double x, double y, double yaw, double cov_x, double cov_y, double cov_yaw);
        void p_motion_update();
        void p_move();

        //parameter
        int N;
        double INIT_X;
        double INIT_Y;
        double INIT_YAW;
        double INIT_X_COV;
        double INIT_Y_COV;
        double INIT_YAW_COV;

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

        nav_msgs::Odometry odometry;
        sensor_msgs::LaserScan laserscan;
        nav_msgs::OccupancyGrid map;
};

#endif
