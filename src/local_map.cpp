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
    private_nh.param("resolution",resolution,{0.05});
    private_nh.param("pillar1",pillar1,{300});
    private_nh.param("pillar2",pillar2,{460});
    private_nh.param("pillar3",pillar3,{600});
    private_nh.param("pillar4",pillar4,{760});
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
    int x  =(int)(row/2 + radius*std::sin(theta)/resolution);
    int y = (int)(column/2 + radius*std::cos(theta)/resolution);
    if(x >= 100)
    {
        x = 99;
    }
    if(y >= 100)
    {
        y = 99;
    }

    grid_map[x][y] = set_value;

    for(float j=0.2 ;j<radius; j+=resolution)
    {

        int ix = (int)(row/2 +(j*std::sin(theta))/resolution);
        int iy = (int)(column/2 + (j*std::cos(theta))/resolution);
        if(ix >= 100)
        {
            ix = 99;
        }
        if(iy >= 100)
        {
            iy = 99;
        }
       // std::cout << "x,y:" << x <<"," << y << std::endl;
        /*if(j <= roomba_radius) grid_map[row/2 +(int)((j*std::sin(theta))/resolution)][column/2 + (int)((j*std::cos(theta))/resolution)] = -1;*/
        grid_map[ix][iy] = 0;
    }
    for(float x=world/2;  x>radius+0.2 ; x-=resolution)
    {
        int ix = (int)(row/2 +(x*std::sin(theta))/resolution);
        int iy = (int)(column/2 + (x*std::cos(theta))/resolution);
        if(ix >= 100)
        {
            ix = 99;
        }
        if(iy >= 100)
        {
            iy = 99;
        }
       // std::cout<< "ix,iy:" << ix << "," << iy << std::endl;
        grid_map[ix][iy] = -1;

    }

}

void Local_map::set_ignore_grid(const int& set_value,const float& radius)
{
    int x  =(int)(row/2 + radius*std::sin(theta)/resolution);
    int y = (int)(column/2 + radius*std::cos(theta)/resolution);
    if(x >= 100)
    {
        x = 99;
    }
    if(y >= 100)
    {
        y = 99;
    }

    grid_map[x][y] = set_value;

    for(float j=0 ;j<radius; j+=resolution)
    {

        int ix = row/2 +round((j*std::sin(theta))/resolution);
        int iy = column/2 + round((j*std::cos(theta))/resolution);
        if(ix >= 100)
        {
            ix = 99;
        }
        if(iy >= 100)
        {
            iy = 99;
        }
        grid_map[ix][iy] = 0;
    }
}


int Local_map::verify_index(const int& i)
{
    if(i >=pillar1  && i <=pillar2)
    {
         average =  (pillar1+pillar2)/2;
         if(average > i)
         {
            return 2;
         }
         else
         {
             return 3;
         }
    }
    if(i >= pillar3 && i <= pillar4)
    {
        average = (pillar3+pillar4)/2;
        if(average > i)
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

    max_index = (int)(((M_PI/2)/laserscan.angle_increment)+540);
    min_index = (int)(((-M_PI/2)/laserscan.angle_increment)+540);

    for(int i=min_index; i<=max_index; i++)
    {

        observation_radius = laserscan.ranges[i];
        theta = (i-540)*laserscan.angle_increment;
        if(observation_radius < 0.3)
        {
           // std::cout <<"index" << i << std::endl;
        }
        if(observation_radius < world/2)
        {
            if(observation_radius < 0.01)
            {
               // std::cout << "distance small" << observation_radius << std::endl;
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
                        index = pillar1 -alliance;
                        estimate_radius = laserscan.ranges[index];
                       // std::cout << "radius 320:" << laserscan.ranges[index] << "\n"<<std::endl;;
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
                        index = pillar2 + alliance;
                        estimate_radius = laserscan.ranges[index];
                       // std::cout << "radius 425: " << laserscan.ranges[index] << "\n" << std::endl;
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
                        index = pillar3 - alliance;
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
                        index =  pillar4 + alliance;
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

    /*for(int i = row/2-min_index; i < row/2 + min_index ; i++)
    {
        for(int j = column/2-min_index;j <column/2 +min_index; j++)
        {

        }
    }*/

    //ルンバの半径以下はカット
    for(int i = min_index;i<max_index; i++)
    {
        set_value = 0;
        min_radius = 0.3;
        theta = (i-540)*laserscan.angle_increment;
        set_ignore_grid(set_value,min_radius);
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


