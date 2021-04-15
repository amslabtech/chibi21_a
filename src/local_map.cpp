#define _USE_MATH_DEFINES
#include "local_map/local_map.h"
#include  <vector>
#include <iostream>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_datatypes.h>

Local_map::Local_map():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("world",world,{5});
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
    grid_map[row/2 + round(radius*std::sin(theta)/resolution)][column/2 + round(radius*std::cos(theta)/resolution)] = set_value;
    for(float j=0 ;j<radius; j+=resolution)
    {
        /*if(j <= roomba_radius) grid_map[row/2 +(int)((j*std::sin(theta))/resolution)][column/2 + (int)((j*std::cos(theta))/resolution)] = -1;*/
        grid_map[row/2 +round((j*std::sin(theta))/resolution)][column/2 + round((j*std::cos(theta))/resolution)] = 0;
    }
    for(float x=world/2;  x>radius+0.1 ; x-=resolution)
    {
        grid_map[row/2 +round((x*std::sin(theta))/resolution)][column/2 + round((x*std::cos(theta))/resolution)] = -1;
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
    //grid_map.resize(row,std::vector<int>(column,0));//テスト用
    //テスト用
    /*for(int i = 0; i<row; i++)
    {
        for(int j = 0; j<column; j++)
        {
            if(i*j % 12000 == 0)
            {
                grid_map[i][j] = 100;
            }
        }
    }*/

    for(int i=0; i<=1080; i++)
    {
        observation_radius = laserscan.ranges[i];
        theta = (i-540)*laserscan.angle_increment;

        if(observation_radius < world/2)
        {
            if(observation_radius <roomba_radius)
            {
                set_value = 0;
                set_grid_map(set_value,world/2);
            }
            else
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
                            set_value = 0;
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
                            set_value = 0;
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
                            set_value = 0;
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
                            set_value = 0;
                            set_grid_map(set_value,world/2);
                        }
                        break;


                    default:
                        break;
                }
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
    local_map.info.origin.position.x = -2.5;
    local_map.info.origin.position.y = -2.5;

    /*tf::Quaternion quaternion=tf::createQuaternionFromRPY(0,0,0);
    //std::cout << "quaternion" << quaternion.x << std::endl;
    local_map.info.origin.orientation.x = quaternion[0];
    local_map.info.origin.orientation.y = quaternion[1];
    local_map.info.origin.orientation.z = quaternion[2];
    local_map.info.origin.orientation.w = quaternion[3];*/

    local_map.header.frame_id ="base_link";
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


