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
    private_nh.param("landmark_set_param",landmark_set_param,{2});
    private_nh.param("w",w,{1.0});
    sub_map= nh.subscribe("map",10,&Astar::map_callback,this);
    pub_path = nh.advertise<nav_msgs::Path>("global_path",1);//local goal creatorへ送る

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
    calc_limit();
    set_landmark();
}

void Astar::test()
{

}

void Astar::set_landmark()
{
    landmark_point.x = 2000;
    landmark_point.y = 2000;
    landmark.push_back(landmark_point);
    landmark_point.x = 1970;
    landmark_point.y =2040;
    landmark.push_back(landmark_point);
    landmark_point.x = 1950;
    landmark_point.y = 2060;
    std::cout << landmark[0].x << std::endl;

}

float Astar::calc_heuristic(const int& x, const int& y)
{
    f = w * sqrt((x - goal_node.x)*(x - goal_node.x) + (y - goal_node.y)*(y - goal_node.y)); //wは重み
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
                add_wall(i,j);
            }
        }
    }
}

//障害物の周囲のコストを上げる
void Astar::add_wall(const int& x ,const int& y)
{
    for(int i = x - wall_thickness; i <= x + wall_thickness;i++)
    {
        for(int j = y - wall_thickness; j <= y + wall_thickness;j++)
        {
            if(grid_map[i][j] != -1 && grid_map[i][j] != 100)
            {
                grid_map[i][j] = 99;
            }
        }
    }
}

//観測した範囲の最大値と最小値を求める
void Astar::calc_limit()
{
    map_number.min_width = row;
    map_number.max_width = 0;
    map_number.min_height = column;
    map_number.max_height = 0;
    std::cout << "calc_limit" << std::endl;
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
    for(int i = map_number.min_width; i < map_number.max_width; i++)
    {
        for(int j = map_number.min_height; j < map_number.max_height; j++)
        {
            close_set[i][j] = init_node;
            open_set[i][j] = init_node;
        }
    }
}

void Astar::define_start_node()
{
    std::cout << "define_start_node" << std::endl;
    current.x = landmark[0].x;
    current.y = landmark[0].y;
    start_node.g = 0;
    start_node.f = 0;
    start_node.parent_x = -1;
    start_node.parent_y = -1;
    open_set[current.x][current.y] = start_node;
}

void Astar::define_goal_node()
{
    goal_node.x = landmark[1].x;
    goal_node.y = landmark[1].y;
}

void Astar::open_node()
{
    for(int i=0;i<8;i++)
    {
        x = (int)motion[i][0];
        y = (int)motion[i][1];
        g = motion[i][2];
        next_g =open_set[current.x][current.y].g + g;
        next_f = next_g + calc_heuristic(current.x + x,current.y + y);

        //ノードが適切か調べる
        if(open_set[current.x + x][current.y + y].f < wall_cost)
        {
            if(open_set[current.x+ x][current.y+ y].f > next_f)
            {
                update_open_set(x,y);
            }
        }
        //close_setの更新はいらないかも
        /*else if(close_set[current.x + x][current.y + y].f < wall_cost)
        {
            if(close_set[current.x + x][current.y + y].f > next_f)
            {
                update_open_set(x,y);
                clear_close_set(x,y);
            }
        }*/
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

/*void Astar::clear_close_set(const int& x,const int& y)
{
    close_set[current.x + x][current.y + y] = init_node;
}*/

void Astar::update_close_set()
{
    /*
    close_set[current.x][current.y].g = open_set[current.x][current.y].g;
    close_set[current.x][current.y].f = open_set[current.x][current.y].f;
    close_set[current.x][current.y].parent_x = current.x;
    close_set[current.x][current.y].parent_y = current.y;*/
    close_set[current.x][current.y] = current_node;
std::cout << "close_set parent"<<current.x<<","<<current.y<<","<<close_set[current.x][current.y].parent_x << std::endl;
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
                next_node.f = open_set[i][j].f;
                current.x = i;
                current.y = j;
               //next_node.parent_y = j;
            }
        }
    }
}

void Astar::check_goal_node()
{
    if(current.x == goal_node.x && current.y == goal_node.y)
    {
        std::cout << "reach check point" << std::endl;
        goal_node.parent_x = current_node.parent_x;
        goal_node.parent_y = current_node.parent_y;
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
    std::cout << "path_x" << path_point.pose.position.x << std::endl;
    std::cout << "path_y" << path_point.pose.position.y << std::endl;
    checkpoint_path.poses.push_back(path_point);
}

void Astar::trace_path()
{
    //complete = false;//test用のためコメントアウト
    add_path_point(goal_node.x,goal_node.y);
    tracing_node.x = close_set[goal_node.x][goal_node.y].parent_x;
    tracing_node.y = close_set[goal_node.x][goal_node.y].parent_y;
    while(!complete){
        add_path_point(tracing_node.x,tracing_node.y);
        reminder.x = tracing_node.x;
        reminder.y = tracing_node.y;
        tracing_node.x = close_set[reminder.x][reminder.y].parent_x;
        tracing_node.y = close_set[reminder.x][reminder.y].parent_y;
        if(tracing_node.x == -1 && tracing_node.y == -1)
        {
            //add_path_point(tracing_node.x,tracing_node.y);
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
            for(int j = map_number.min_height; j<map_number.max_height; j++)
            {
                //minの初期化忘れでバク、注意!!
                if(open_set[i][j].f< min)
                {
                    min = open_set[i][j].f;
                    current_node.f = open_set[i][j].f;
                    current_node.g = open_set[i][j].g;
                    current_node.parent_x = open_set[i][j].parent_x;
                    current_node.parent_y = open_set[i][j].parent_y;
                    //std::cout << "current_node.parent_x" << i << std::endl;
                    //close_set.push_back(node);
                }
            }
        }
        min = wall_cost;
        open_node();//open_node()をupdate_current_node移行に実行しないで
        update_close_set();
        check_goal_node();
        update_current_node();
        //check_goal_node();
    }
    trace_path();
    clear_node();
}

void Astar::planning()
{
    clear_node();
    //nodeの初期設定
    node_set();
    for(int i = 0; i<landmark_set_param; i++)
    {
        if(open_set.size() == 0)
        {
            std::cout << "Open set is empty.." << std::endl;
        }
        checkpoint_path_creator();
        //checkpoint_path_createrで作ったパスをglobal_pathにくっつける
        global_path.poses.insert(global_path.poses.end(),checkpoint_path.poses.begin(),checkpoint_path.poses.end());
    }

    std::cout << "global_path  created!" << std::endl;
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

