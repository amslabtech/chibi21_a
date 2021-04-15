#define _USE_MATH_DEFINES
#include "local_map/local_map.h"
#include  <vector>
#include <iostream>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"

Local_map::Local_map():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("world",world,{5});
    sub_scan = nh.subscribe("scan",10,&Local_map::scan_callback,this);
    pub_local_map = nh.advertise<nav_msgs::Path>("local_map",1);
}

void Local_map::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laserscan = *msg;
    create_local_map();
}

void Local_map::create_local_map()
{
    row = (int)(world/resoution);
    column = (int)(world/resoution);
    grid_map.resize(row,std::vector<int>(column,-1));

    angle_min = laserscan.angle_min;
    angle_max = laserscan.angle_max;
    angle_increment = (int)(degree * M_PI/180 / laserscan.angle_increment);
    base_angle = 540;
    max_index = (int)(((M_PI/2)/laserscan.angle_increment) + 540);
    min_index = (int)(((-M_PI/2)/laserscan.angle_increment) + 540);

    for(int i=min_index; i<=max_index; i+=angle_increment)
    {
        observation_radius = laserscan.ranges[i];
        theta = (i-540)*angle_increment;
        grid_map[row/2 + (int)(observation_radius*std::sin(theta)/resoution)][column/2 + (int)(observation_radius*std::cos(theta)/resoution)] = 100;

        for(float j=0 ; j<observation_radius; j+=resoution)
        {
            /*if(j <= roomba_radius) grid_map[row/2 +(int)((j*std::sin(theta))/resoution)][column/2 + (int)((j*std::cos(theta))/resoution)] = -1;*/

            grid_map[row/2 +(int)((j*std::sin(theta))/resoution)][column/2 + (int)((j*std::cos(theta))/resoution)] = 0;
        }
    }
    nav_msgs::OccupancyGrid local_map;
    local_map.header.frame_id ="map";
    for(int i = 0; i<row ;i++)
    {
        for(int j=0; j<column; j++)
        {
            local_map.data[j*row + i] = grid_map[i][j];
        }
    }

    pub_local_map.publish(local_map);
}

void Local_map::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"local_map");
    Local_map local_map;
    local_map.process();
    return 0;
}
