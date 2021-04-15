#ifndef LOCAL_MAP
#define LOCAL_MAP
#include <ros/ros.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"

class Local_map
{
public:
    Local_map();
    void process();

private:
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
    void create_local_map();
    void set_grid_map(const int& set_value,const float& radius);
    int verify_index(const int&) ;
    int hz = 0;
    int set_value = 0;
    int world = 0;
    int row = 0;
    int column = 0;
    int base_angle = 0;
    int max_index = 0;
    int min_index = 0;
    int alliance = 0;
    int index = 0;
    float roomba_radius = 0.3;//ルンバの半径 30cm
    float observation_radius = 0;
    float degree = 0;
    float resolution = 0.02;
    float average = 0.0;
    float radius = 0;
    float theta = 0;
    float max_radius = 0.0;
    float estimate_radius = 0.0;
    float angle_min =0;
    float angle_max = 0;
    int angle_increment = 0;
    std::vector< std::vector<int>> grid_map;
    ros::Publisher pub_local_map;
    ros::Subscriber sub_scan;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    sensor_msgs::LaserScan laserscan;
    nav_msgs::Path local_map;
};

#endif
