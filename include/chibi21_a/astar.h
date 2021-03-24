#ifndef CHIBI_20_A_ASTAR
#define CHIBI_20_A_ASTAR
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

struct Node
{
    float g;
    float f;
    int parent_x;
    int parent_y;
};

struct Coordinate
{
    int x;
    int y;
};

struct Map
{
    int min_width;
    int max_width;
    int min_height;
    int max_height;
};

class Astar
{
public:
    Astar();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);
    void set_parameter();
    void calc_final_path();
    float clac_heuristic(const int&,const int&);
    void clear_node();

    nav_msgs::Path path;
    geometry_msgs::PoseStamped roomba_pose;
    nav_msgs::OccupancyGrid map;

    std::vector<Node> open_set,close_set;
    std::vector<float> rx ,ry;
    std::vector< std::vector<float> > motion = {
        {1,0,1},
        {0,1,1},
        {-1,0,1},
        {0,-1,1},
        {-1,-1,sqrt(2)},
        {-1,1,sqrt(2)},
        {1,-1,sqrt(2)},
        {1,1,sqrt(2)}
    };
    std::vector< std::vector<int> > grid_map;
    Node node;
    Node start_node;
    Node goal_node;
    Node init_node;
    Node next_node;
    Node current_node;
    Coordinate landmark[] = {{}} //未定
    Coordinate current;
    Map map;
    int hz;
    int x = 0;
    int y = 0;
    int g = 0;
    int row = 0;
    int wall_border = 0;
    int column = 0;
    int w = 0;
    float pos = 0;
    float now_f = 0;
    float old_f = 0;
    float next_g = 0;
    float next_f = 0;
    ros::NodeHandle nh;

#endif
