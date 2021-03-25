#include "astar/astar.h"
#include <vector>
#include <math.h>
#include <iostream>

Astar::Astar():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("wall_cost",wall_cost,{1e+10});
    private_nh.param("wall_border",wall_border,{50});
    private_nh.param("wall_thickness",wall_thickness,{4});
    sub_map= nh.subscribe("map",10,&Astar::map_callback,this);
    pub_path = nh.advertise<nav_msgs::Path>("global_path",1);

}

void Astar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

    prior_map = *msg;
    if(prior_map.data.size() != 0)
    {
        std::cout << prior_map.info.width << std::endl;
    }
    if(!set_map_checker)
    {
        std::cout << "aa" << std::endl;
        set_map_parameter();
        set_map_checker = true;
    }
}

void Astar::set_map_parameter()
{

    row = prior_map.info.height;
    column = prior_map.info.width;
    grid_map.resize(row,std::vector<int>(column));
    open_set.resize(row,std::vector<Node>(column));
    close_set.resize(row,std::vector<Node>(column));
    init_node = {wall_cost,wall_cost,-1,-1};
    medium_value.x = row/2;
    medium_value.y = column/2;
    resolution = prior_map.info.resolution;
    global_path.header.frame_id ="map";
    for(int i = 0;i<row;i++)
    {
        for(int j = 0;j<column;j++)
        {
            grid_map[i][j] = prior_map.data[j*row+i];
        }
    }
}

float Astar::calc_heuristic(const int& x, const int& y)
{
    f = w * sqrt((current.x + x - goal_node.x)*(current.x + x - goal_node.x) + (current.y + y - goal_node.y)*(current.y + y - goal_node.y)); //wは重み
    return f;
}

void Astar::node_set()
{
    for(int i = map_number.min_width;i<map_number.max_width;i++)
    {
        for(int j = map_number.min_height; j<map_number.max_height; j++)
        {
            if(grid_map[i][j] == 100)
            {
                add_wall();
            }
        }
    }
}

//障害物の周囲のコストを上げる
void Astar::add_wall()
{
    for(int i = map_number.min_width; i< map_number.max_width;i++)
    {
        for(int j = map_number.min_height; i<map_number.max_width;j++)
        {
            if(grid_map[i][j] != -1 && grid_map[i][j] != 100)
            {
                grid_map[i][j] = 99;
            }
        }
    }
}

//観測した範囲の最大値と最小を求める
void Astar::calc_limit()
{
    map_number.min_width = row;
    map_number.max_width = 0;
    map_number.min_height = column;
    map_number.max_height = 0;
    for(int i = 0;i<row;i++)
    {
        for(int j = 0; j<column;j++)
        {
            if(grid_map[i][j] != -1)
            {
                if(i<map_number.min_width)
                {
                    map_number.min_width = i;
                }
                if(j<map_number.min_height)
                {
                    map_number.min_height = j;
                }
                if(i>map_number.max_width)
                {
                    map_number.max_width = i;
                }
                if(j>map_number.max_height)
                {
                    map_number.max_height = j;
                }
            }
        }
    }
}

//チェックポイントまで探索したら初期化
void Astar::clear_node()
{
    std::cout<< "bb"<< std::endl;
    for(int i = map_number.min_width; i<map_number.max_width; i++)
    {
        for(int j = map_number.min_height; i<map_number.max_height; j++)
        {
            close_set[i][j] = init_node;
            open_set[i][j] = init_node;
        }
    }
}

void Astar::define_start_node()
{
    current.x = landmark[checkpoint_index].x;
    current.y = landmark[checkpoint_index].y;
}

void Astar::define_goal_node()
{
    goal_node.x = landmark[checkpoint_index+1].x;
    goal_node.y = landmark[checkpoint_index+1].y;
}

void Astar::open_node()
{
    for(int i=0;i<9;i++)
    {
        x = (int)motion[i][0];
        y = (int)motion[i][1];
        g = motion[i][2];

        next_g =open_set[current.x + x][current.y + y].g + g;
        next_f = next_g + calc_heuristic(current.x + x,current.y + y);

        //ノードが適切か調べる
        if(open_set[current.x + x][current.y + y].f < wall_cost)
        {
            if(open_set[current.x+ x][current.y+ y].f > next_f)
            {
                update_open_set(x,y);
            }
        }

        else if(close_set[current.x + x][current.y + y].f < wall_cost)
        {
            if(close_set[current.x + x][current.y + y].f < next_f)
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

    for(int i = map_number.min_width; i<map_number.max_width; i++)
    {
        for(int j = map_number.min_height; j<map_number.max_height; j++)
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
        goal_node.f = current_node.f;
        reach_goal = true;
    }
}

void Astar::add_path_point(const int& x,const int& y)
{
    //ロボットの座標系の直す
    geometry_msgs::PoseStamped path_point;
    path_point.pose.position.x = ((double)((x - medium_value.x)*resolution));
    path_point.pose.position.y = ((double)((y - medium_value.y)*resolution));
    checkpoint_path.poses.push_back(path_point);
}

void Astar::trace_path()
{
    complete = false;
    add_path_point(goal_node.x,goal_node.y);
    tracing_node.x = open_set[goal_node.x][goal_node.y].parent_x;
    tracing_node.y = open_set[goal_node.x][goal_node.y].parent_y;
    while(!complete)
    {
        add_path_point(tracing_node.x,tracing_node.y);
        reminder.x = tracing_node.x;
        reminder.y = tracing_node.y;
        tracing_node.x = close_set[reminder.x][reminder.y].parent_x;
        tracing_node.y = close_set[reminder.x][reminder.y].parent_y;
        if(tracing_node.x == close_set[tracing_node.x][tracing_node.y].parent_x && tracing_node.y == close_set[tracing_node.x][tracing_node.y].parent_y)
        {
            add_path_point(tracing_node.x,tracing_node.y);
            complete = true;
        }
    }
    //ゴールから入れているので逆転させる
    std::reverse(checkpoint_path.poses.begin(),checkpoint_path.poses.end());
}
//チェックポイントまでの最適パスを探す
void Astar::checkpoint_path_creator()
{
    checkpoint_path.poses.clear();
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
        for(int i = map_number.min_width; i<map_number.max_width; i++)
        {
            for(int j = map_number.min_height; i<map_number.max_height; j++)
            {
                 now_f = open_set[i][j].f + calc_heuristic(open_set[i][j].x,open_set[i][j].y);
                if(now_f < old_f)
                {
                    current_node = open_set[i][j];
                    //close_set.push_back(node);
                }
                else
                {
                    old_f = now_f;
                }

            }
        }

        update_close_set();
        //次のnodeを展開する
        open_node();
        update_current_node();
        check_goal_node();
    }
    trace_path();
}

void Astar::planning()
{
    clear_node();
    //nodeの初期設定
    node_set();


    for(int i = 0; i<2; i++)
    {
        if(open_set.size() == 0)
        {
            std::cout << "Open set is empty.." << std::endl;
        }
        checkpoint_path_creator();
        //checkpoint_path_createrで作ったパスをglobal_pathにくっつける
        global_path.poses.insert(global_path.poses.end(),checkpoint_path.poses.begin(),checkpoint_path.poses.end());

        if(i == 2)
        {
            std::cout << "global_path  created!" << std::endl;
        }
    }
    pub_path.publish(global_path);//Pathをpublishする
}



void Astar::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        if(set_map_checker) planning();
        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"astar");
    Astar astar;
    astar.process();
}

