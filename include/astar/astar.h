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
    int x;
    int y;
    int parent_x;
    int parent_y;
};

struct Coordinate
{
    int x;
    int y;
};

struct Map_number
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
    void test(); //パスを表示するための関数　テスト用
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);
    void set_map_parameter();
    void calc_final_path();
    float clac_heuristic(const int&,const int&);
    void clear_node();
    void clear_close_set(const int& x,const int& y);
    void check_goal_node();
    void add_path_point(const int& x,const int& y);
    void update_current_node();
    void update_close_set();
    void update_open_set(const int& x,const int& y);
    void open_node();
    void define_start_node();
    void define_goal_node();
    void calc_limit();
    void add_wall(const int& i,const int& j);
    void node_set();
    void trace_path();
    void checkpoint_path_creator();
    void set_landmark();
    void planning();
    float calc_heuristic(const int&,const int&);
    bool set_map_checker = false;
    bool reach_goal = false;
    bool complete = false;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped roomba_pose;
    nav_msgs::OccupancyGrid prior_map;
    nav_msgs::Path checkpoint_path;
    nav_msgs::Path global_path;

    std::vector< std::vector<Node> > open_set,close_set;
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
    Node start_node;
    Node goal_node;
    Node init_node;
    Node next_node;
    Node current_node;
    std::vector<Coordinate> landmark;
    Coordinate landmark_point;
    Coordinate current;
    Coordinate reminder;
    Coordinate tracing_node;
    Coordinate medium_value;
    Map_number map_number;
    int hz;
    int x = 0;
    int y = 0;
    int g = 0;
    int row = 0;
    int wall_border = 0;
    int column = 0;
    float w = 0;
    int wall_thickness = 0;
    float resolution = 0;
    float pos = 0;
    float min = 1e+10;
    float next_g = 0;
    float next_f = 0;
    float wall_cost =0;
    float f = 0;
    int landmark_set_param = 0;
    int checkpoint_index = 0;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_path;
    ros::Subscriber sub_map;
};

#endif
