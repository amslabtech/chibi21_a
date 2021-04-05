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
    private_nh.param("degree",degree,{5});
    private_nh.param("alliance",alliance,{5});
    sub_scan = nh.subscribe("scan",10,&Local_map::scan_callback,this);
    pub_local_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map",1);
}

void Local_map::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    laserscan = *msg;
    if(laserscan.ranges.size() != 0) create_local_map();
}

void Local_map::set_grid_map(const int& set_value,const float& radius)
{
    grid_map[row/2 + (int)(radius*std::sin(theta)/resolution)][column/2 + (int)(radius*std::cos(theta)/resolution)] = set_value;
    for(float j=0 ;j<radius; j+=resolution)
    {
        /*if(j <= roomba_radius) grid_map[row/2 +(int)((j*std::sin(theta))/resolution)][column/2 + (int)((j*std::cos(theta))/resolution)] = -1;*/
        grid_map[row/2 +(int)((j*std::sin(theta))/resolution)][column/2 + (int)((j*std::cos(theta))/resolution)] = 0;
    }
    for(float x=world/2;  x>radius+0.1 ; x-=resolution)
    {
        grid_map[row/2 +(int)((x*std::sin(theta))/resolution)][column/2 + (int)((x*std::cos(theta))/resolution)] = -1;
    }

}
int Local_map::verify_index(const int& i)
{
    if(i >= 348 && i <= 409)
    {
         average =  (348+409)/2;
         if(average < i)
         {
            return 2;
         }
         else
         {
             return 3;
         }
    }
    if(i >= 700 && i <= 762)
    {
        average = (700+762)/2;
        if(average < i)
        {
            return 4;
        }
        else
        {
            return 5;
        }
    }
    else
    {
        return 1;
    }
}

void Local_map::create_local_map()
{

    row = (int)(world/resolution);
    column = (int)(world/resolution);
    grid_map.clear();
    grid_map.resize(row,std::vector<int>(column,-1));

    angle_min = laserscan.angle_min;
    angle_max = laserscan.angle_max;
    angle_increment = (int)(degree * M_PI/180 / laserscan.angle_increment);
    base_angle = 540;
    max_index = (int)(((M_PI/2)/laserscan.angle_increment) + 540);
    min_index = (int)(((-M_PI/2)/laserscan.angle_increment) + 540);
    for(int i=min_index; i<=max_index; i++)
    {
        observation_radius = laserscan.ranges[i];
        theta = (i-540)*laserscan.angle_increment;

        if(observation_radius < world/2)
        {
            switch(verify_index(i))
            {
                case 1:
                    set_value = 100;
                    set_grid_map(set_value,observation_radius);
                    break;
                case 2:
                    index = 348 -alliance;
                    estimate_radius = laserscan.ranges[index];
                    if(estimate_radius < world/2)
                    {
                        set_value = 100;
                        set_grid_map(set_value,estimate_radius);
                    }
                    else
                    {
                        set_value = 100;
                        set_grid_map(set_value,world/2);
                    }
                    break;

                case 3:
                    index = 409 + alliance;
                    estimate_radius = laserscan.ranges[index];
                    if(estimate_radius < world/2)
                    {
                        set_value = 100;
                        set_grid_map(set_value,estimate_radius);
                    }
                    else
                    {
                        set_value = 100;
                        set_grid_map(set_value,world/2);
                    }

                    break;

                case 4:
                    index = 700 - alliance;
                    estimate_radius = laserscan.ranges[index];
                    if(estimate_radius < world/2)
                    {
                        set_value = 100;
                        set_grid_map(set_value,estimate_radius);
                    }
                    else
                    {
                        set_value = 100;
                        set_grid_map(set_value,world/2);
                    }
                    break;

                case 5:
                    index =  762 + alliance;
                    estimate_radius = laserscan.ranges[index];
                    if(estimate_radius < world/2)
                    {
                        set_value = 100;
                        set_grid_map(set_value,estimate_radius);
                    }
                    else
                    {
                        set_value = 100;
                        set_grid_map(set_value,world/2);
                    }
                    break;


                default:
                    break;
            }
        }
        else
        {
            set_value = 0;
            max_radius = world/2;
            set_grid_map(set_value,max_radius);
        }

    }

    nav_msgs::OccupancyGrid local_map;
    local_map.info.resolution = resolution;
    local_map.info.width = column;
    local_map.info.height = row;
    local_map.data.resize(row*column);
    local_map.header.frame_id ="map";
    for(int i = 0; i<row ;i++)
    {
        for(int j=0; j<column; j++)
        {
            local_map.data[j*row + i] = grid_map[i][j];
            //local_map.data[j*row + i] = 0;
        }
    }

    pub_local_map.publish(local_map);
}

void Local_map::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        //create_local_map();
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
