#define _USE_MATH_DEFINES
#include "map_manager/map_manager.h"
#include <cmath>

Map_Manager::Map_Manager():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("parallel_height",parallel_height,{600});//値テキトウ、平行移動
    private_nh.param("parallel_width",parallel_width,{900});//値テキトウ
    private_nh.param("degree",degree,{170});//値テキトウ、回転移動
    sub_map = nh.subscribe("map",10,&Map_Manager::map_callback,this);

    pub_modify_map = nh.advertise<nav_msgs::OccupancyGrid>("modify_map",1);
}

void Map_Manager::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if(!recieved_original_map)
    {
        std::cout<<"recieved original map"<<std::endl;
        recieved_original_map = true;
        original_map = *msg;
        set_grid_map_parametor();
        std::cout << "set_gridの後" << std::endl;
        //オリジナルのマップを修正する
        conversion_modify_map();
        pub_modify_map.publish(modify_map);
    }
}

void Map_Manager::set_grid_map_parametor()
{
    row = original_map.info.height;
    column = original_map.info.width;
    medium_value.x = row/2;
    medium_value.y = column/2;
    resolution = original_map.info.resolution;
    theta = M_PI/180 * degree;
    //初期化
    grid_map.resize(row,std::vector<int>(column,-1));
    copy_map.resize(row,std::vector<int>(column,-1));
    modify_map.data.resize(row*column,-1);
    for(int i = 0;i < row;i++)
    {
        for(int j =0;j < column;j++)
        {
            grid_map[i][j] = original_map.data[j*row +i];
        }
    }
    calc_limit();
}
//astarと同じ
void Map_Manager::calc_limit()
{
    map_number.min_width = row;
    map_number.max_width = 0;
    map_number.min_height = column;
    map_number.max_height = 0;
    for(int i = 0;i<row;i++)
    {
        for(int j=0;j<column;j++)
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

//平行移動と回転移動とグリッドマップ座標系の変換を行う
void Map_Manager::calc_motion(const int& i,const int& j)
{
    v = i - parallel_height;
    u = j - parallel_width;
    /*y = (double)((grid_v - medium_value.y)*resolution);
    x = (double)((grid_u - medium_value.x)*resolution);*/
    u = (int)((u - medium_value.x) * std::cos(theta) + (v - medium_value.y) * std::sin(theta));
    v = (int)((-u +medium_value.x) * std::sin(theta) + (v - medium_value.y) * std::cos(theta));
    /*u = u/resolution +medium_value.x;
    v = v/resolution + medium_value.y;*/
}

void Map_Manager::conversion_modify_map()
{
    for(int i = map_number.min_height;i <= map_number.max_height;i++)
    {
        for(int j =map_number.min_width;j <= map_number.max_width;j++)
        {
            if(grid_map[i][j] != -1)
            {
                calc_motion(i,j);
                modify_map.data[u*row + v] = grid_map[i][j];
            }
        }
    }
    pub_modify_map.publish(modify_map);
    /*for(int i = map_number.min_height;i <= map_number.max_height;i++)
    {
        for(int j =map_number.min_width;j <= map_number.max_width;j++)
        {
            if(grid_map[i][j] != -1)
            {
                modify_map[i][j] = copy_map[i][j];
            }
        }
    }*/


}
void Map_Manager::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc,char **argv)
{
    ros::init(argc,argv,"map_maneger");
    Map_Manager map_manager;
    map_manager.process();
    return 0;
}

