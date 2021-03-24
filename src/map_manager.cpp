#include "map_manager/map_manager.h"


Map_Manager::Map_Manager():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("parallel_y",parallel_height,{600});//値テキトウ、平行移動
    private_nh.param("parallel_x",parallel_weight,{900});//値テキトウ
    private_nh.param("degree",degree,{170});//値テキトウ、回転移動
    sub_map = nh.subscribe("map",10,&Map_Manager::map::callback,this);

    pub_modify_map = nh.advertise<nav_msgs::OccupancyGrid>("modify_map",1);
}

void Map_Manager::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("recieved_original_map");
    original_map = *msg;
    set_girid_map_parametor();
    //オリジナルのマップを修正する
    modify_map();
    pub_modify_map.publish(modify_map);

}

void Map_Manager::set_gird_map_parametor()
{
    row = original_map.info.height;
    column = original_map.info.width;
    medium_value.x = row/2;
    medium_value.y = column/2;
    //初期化
    grid_map.resize(row,std::vector<int>(column,-1));
    for(int i = 0;i < column;i++)
    {
        for(int j =0;j < row;j++)
        {
            grid_map[j][i] = original_map.data[i*column +j];
        }
    }
}

//平行移動と回転移動とグリッドマップ座標系の変換を行う
void Map_Manager::calc_motion()
{
    u = i +
}

void Map_Manager::modify_map()
{
    for(int i = 0;i < row;i++)
    {
        for(int j =0;j < column;j++)
        {
            if(grid_map[j][i] != -1)
            {
                calc_motion();
                modified_map[u][v] = grid_map[i][j];
            }
        }
    }

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

