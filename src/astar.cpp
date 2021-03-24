#include "astar/astar.h"
#include <vector>
#include <math.h>
#include <iostream>

Astar::Astar():private_nh("~")
{
    private_nh.param("hz",hz,{1});

    sub_pose = nh.subscribe("map",10,&Astar::pose_callback,this);
    pub_path = nh.advertise<nav_msgs::Path>("global_path",1);

}

void Astar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    prior_map = *msg;
    set_map_parameter();
}

void Astar::set_parameter()
{
    row = prior_map.info.height;
    column = prior_map.info.weight;
    init_node = {wall_cost,wall_cost,-1,-1};

    for(int i = 0;i<row;i++)
    {
        for(int j = 0;j<column;j++)
        {
            grid_map[i][j] = prior_map.data[j*row+i];
        }
    }
}

void Astar::calc_final_path(goal_node,closed_set)
{
    parent_index = goal_node.parent_index;
    while(parent_index != -1)
    {
        n = closed_set.back();
        rx.push_back(calc_grid_position(n.x,min_x));
        ry.push_back(calc_grid_position(n,y,min_y));
        parent_index = n.parent_index;
    }
}

float Astar::calc_heuristic(int& x, int& y)
{
    f = w * sqrt(pow(current.x + x - goal_node.x) + pow(current.y + y - goal_node.y)); //wは重み
    return f;
}

void Astar::calc_limit()
{
    map.min_width = row;
    map.max_width = 0;
    map.min_height = column;
    map.max_height = 0;
    for(int i = 0;i<row;i++)
    {

}

//チェックポイントまで探索したら初期化
void Astar::clear_node()
{
    for(int i = map.min_width; i<map.max_width; i++)
    {
        for(int j = map.min_height; i<map.max_height; j++)
        {
            close_set[i][j] = init_node;
            open_node[i][j] = init_node;
        }
    }
}

void Astar::define_start_node()
{
    serching_node.x = landmark[waypoint].x;
    serching_node.y = landmark[waypoint].y;
}

void Astar::define_goal_node()
{
    goal_node.x = landmark[waypoint+1].x;
    goal_node.y = landmark[waypoint+1].y;
}

void Astar::open_node()
{
    for(int i=0;i<9;i++)
    {
        x = motion[i][0];
        y = motion[i][1];
        g = motion[i][2];

        next_g =open_set[current.x + x][current.y + y].g + g;
        next_f = next_g + calc_heuristc(current.x + x,current.y + y);

        //ノードが適切か調べる
        if(open_set[current.x + x][current.y + y] < wall_cost)
        {
            if(open_set[current.x+ x][current.y+ y].f > next_f)
            {
                update_open_set(x,y);
            }
        }

        else if(close_set[current.x + x][current.y + y] < wall_cost)
        {
            if(close_set[current.x + x][current.y + y] < next_f)
            {
                update_open_set(x,y);
                clear_close_set(x,y);
            }
        }
        else
        {
            update_open_set(x,y);
        }
    }
}

void Astar::update_open_set(const int& x,const int& y)
{
    if(grid_map[current.x + x][current.y + y] > wall_border || grid_map[current.x + x][current.y] == -1)
    {
        return;
    }
    open_set[current.x + x][current.y + y].g = next_g;
    open_set[current.x + x][current.y + y].f = next_f;
    open_set[current.x + x][current.y + y].parent_x = current.x;
    open_set[current.x + x][current.y + y].parent_y = current.y;
}

void Astar::clear_close_set(const int& x,const int& y)
{
    close_set[current.x + x][current.y + y] = init_node;
}

void Astar::update_close_set()
{
    /*
    close_set[current.x][current.y].g = open_set[current.x][current.y].g;
    close_set[current.x][current.y].f = open_set[current.x][current.y].f;
    close_set[current.x][current.y].parent_x = current.x;
    close_set[current.x][current.y].parent_y = current.y;*/

    close_set[current.x][current.y] = current_node;
    //ノードの初期化
    open_set[current.x][current.y] = init_node;
}

//open_setの中から最小のコストのnodeを選ぶ
void Astar::update_current_node()
{
    next_node = init_node;

    for(int i = map.min_width; i<map.max_width; i++)
    {
        for(int j = map.min_height; j<map.max_height; j++)
        {
            if(open_set[i][j].f < next_node.f)
            {
                next_node.g = open_set[i][j].g;
                next_node.f = open_set[i][j].f;
                current.x = i;
                current.y = j;
                /*next_node.parent_x = i;
                next_node.parent_y = j;*/
            }
        }
    }
}

void Astar::check_goal_node()
{
    if(current.x == goal_node.x && current.y == goal_node.y)
    {
        std::cout << "reach check point" << std::endl;
        goal_node.parent_x = current.x;
        goal_node.parent_y = current.x;
        goal_node.cost = current.cost;
    }
}

//チェックポイントまでの最適パスを探す
void Astar::cheakpoint_path_creater()
{
    clear_node();//nodeの初期化
    define_start_node();
    define_goal_node();

    //探索
    while(!reach_goal)
    {
        if(open_set.size() == 0)
        {
            std::cout << "Open set is empty .." << std::endl;
        }

        //open_setの中から最もコストが小さいnodeを選ぶ
        for(auto& node : open_set )
        {
            now_f = node.f + calc_heuristc(node.x,node.y);
            if(now_f < old_f)
            {
                current_node = node;
                //close_set.push_back(node);
            }

            old_f = now_f;
        }

        update_close_set();
        //次のnodeを展開する
        open_node();
        update_current_node();
        check_goal_node();
    }
}

void Astar::planning()
{
    node_update();


    open_set.push_back(start_node);
    start_node.node_index = calc_grid_index(strat_node);//nodeのindexを代入
    auto itr = open_set.begin();

    while (true)
    {
        if(open_set.size() == 0)
        {
            std::cout << "Open set is empty.." << std::endl;
        }
        /*if(itr == open_set.end())
        {
            break;
        }*/
   }

    calc_final_path(goal_node,closed_set);

    pub_path.publish(...);//Pathをpublishする
}



void Astar::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        planning();
        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main(int argc,char** argv)
{
    ros::init(argv,"astar");
    Astar astar;
    astar.process();
}

