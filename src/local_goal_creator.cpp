#include "local_goal_creator/local_goal_creator.h"
#include <vector>
#include <iostream>

Local_goal_creator::Local_goal_creator():private_nh("~")
{
    private_nh.param("hz",hz,{1});
    private_nh.param("ahead_value",ahead_value,{50});
    private_nh.param("limit_distance",limit_distance,{1.0});
    sub_path = nh.subscribe("global_path",1,&Local_goal_creator::check_point_callback,this);
    //amclの人からもらう
    sub_pose = nh.subscribe("aaaa",1,&Local_goal_creator::pose_callback,this);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("local_goal",1);
}

void Local_goal_creator::check_point_callback(const nav_msgs::Path::ConstPtr &msg)
{
    if(!receive_global_path)
    {
        global_path = *msg;
        std::cout << "receive global_path" << std::endl;
        receive_global_path = true;
        local_path_index = 0;
    }
}

void Local_goal_creator::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(!receive_current_pose)
    {
        current_pose = *msg;
        std::cout << "receive current_pose" << std::endl;
        receive_current_pose = true;
    }
}

void Local_goal_creator::calc_distance(const float& gx,const float& gy,const float& cx,const float& cy)
{
    distance = sqrt((gx-cx)*(gx-cx) + (gy-cy)*(gy-cy));
}

void Local_goal_creator::calc_local_goal()
{
    current.x = current_pose.pose.position.x;
    current.y = current_pose.pose.position.y;

    calc_distance(local_goal.pose.position.x,local_goal.pose.position.y,current.x,current.y);
    std::cout << "distance" << distance << std::endl;

    if(distance < limit_distance) local_path_index += ahead_value;
    if(local_path_index < global_path.poses.size()) local_goal = global_path.poses[local_path_index];
    else
    {
        //ゴールに近くなったらちょっとずつ先のゴールを決める
        local_path_index = local_path_index - ahead_value + 10;
        if(local_path_index < global_path.poses.size())
        {
            local_goal = global_path.poses[local_path_index];
        }
        else
        {
            local_goal = global_path.poses[global_path.poses.size() -1];
        }
    }

    std::cout << "local goal x" << local_goal.pose.position.x << "," << "local goal y" << local_goal.pose.position.y << std::endl;
    local_goal.header.frame_id = "map";
    pub_local_goal.publish(local_goal);
        /*for(auto& position : global_path.poses)
        {

            distance = calc_distance(position.pose.position.x,position.pose.position.y,current_pose.x,current_pose.y);
            if(distance < limit)
            {
                limit = distance;
                limit_pose.x = position.pose.position.x;
                limit_pose.y = position.pose.position.y;
            }
        }*/


}

void Local_goal_creator::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        if(receive_global_path && receive_current_pose) calc_local_goal();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"local_goal_creator");
    Local_goal_creator local_goal_creator;
    local_goal_creator.process();
}




