#ifndef MAP_MANAGER
#define MAP_MANAGER
#include <ros/ros.h>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

struct Coodinate
{
    float x;
    float y;
};
struct Map_number
{
    int min_height;
    int max_height;
    int min_width;
    int max_width;
};

class Map_Manager
{
public:
    Map_Manager();
    void conversion_modify_map();
    void set_grid_map_parametor();
    void process();
    void calc_motion(const int& i,const int& j);
    void calc_limit();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);

    int hz = 0;
    int parallel_height = 0;
    int parallel_width = 0;
    int u = 0;
    int v = 0;
    float row = 0;
    float column = 0;
    float resolution = 0;
    float degree = 0;
    float theta = 0;
    bool recieved_original_map = false;
    std::vector< std::vector<int> > grid_map;
    std::vector< std::vector<int> > copy_map;
    Coodinate medium_value;
    Map_number map_number;
    ros::Publisher pub_modify_map;
    ros::Subscriber sub_map;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    nav_msgs::OccupancyGrid original_map;
    nav_msgs::OccupancyGrid modify_map;

};

#endif

