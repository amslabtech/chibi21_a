#ifndef MAP_MANAGER
#define MAP_MANAGER
#include <ros/ros.h>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

struct Coodinate
{
    float i;
    float j;
};

class Map_Manager
{
public:
    Map_Manager();
    void modify_map();
    void set_grid_map_parametor();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);

    int hz;
    int parallel_y;
    int parallel_x;
    int u;
    int v;
    float row;
    float column;

    std::vector< std::vector<int> > grid_map;
    Coodinate medium_value;
    Coodinate parallel;
    ros::Publisher pub_modify_map;
    ros::Subscriber sub_map;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    nav_msgs/OccupancyGrid original_map;

};

#endif

