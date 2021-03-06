#include "Local_goal_creator/Local_goal_creator.h"

Local_Goal_Creator::Local_Goal_Creator():private_nh("~"){
    private_nh.param("hz",hz,{10});
    private_nh.param("reselection_dis",reselection_dis,{2.0});
    private_nh.param("reselection_add_val",reselection_add_val,{50});
    sub_global_path = nh.subscribe("global_path",10,&Local_Goal_Creator::global_path_callback,this);
    sub_current_pose = nh.subscribe("estimated_pose",10,&Local_Goal_Creator::current_pose_callback,this);
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("local_goal",1);
    pub_final_goal = nh.advertise<geometry_msgs::PoseStamped>("final_goal",1);
}

void Local_Goal_Creator::global_path_callback(const nav_msgs::Path::ConstPtr& msg){
    if(!got_global_path){
        global_path = *msg;
        if(global_path.poses.size() != 0)
        {
            local_goal = global_path.poses[goal_number];
            got_global_path = true;
        }
    }
}
void Local_Goal_Creator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void Local_Goal_Creator::mesure_dis(const float& x1,const float& y1,const float& x2,const float& y2){
    distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

void Local_Goal_Creator::set_the_next_goal(){
    float cx = current_pose.pose.position.x;
    float cy = current_pose.pose.position.y;
    count = 0;
    max_distance = 1e5;

    geometry_msgs::PoseStamped temporay_goal;
    temporay_goal.pose.position.x = 0.0;
    temporay_goal.pose.position.y = 0.0;

    for(auto& point : global_path.poses)
    {
        float x = point.pose.position.x;
        float y = point.pose.position.y;

        mesure_dis(x,y,cx,cy);
        if(max_distance > distance)
        {
            max_distance = distance;
            min_index = count;
        }
        count++;
    }

    if(min_index+reselection_add_val > global_path.poses.size() -1)
    {
        reach_final_goal = true;
        local_goal = global_path.poses[global_path.poses.size() - 1];
        final_goal = global_path.poses[global_path.poses.size() - 1];
    }
    else
    {
        local_goal = global_path.poses[min_index+reselection_add_val];
        final_goal = temporay_goal;
    }



    local_goal.header.frame_id = "map";
    if(!reach_final_goal) pub_local_goal.publish(local_goal);
    else
    {
        local_goal = global_path.poses[global_path.poses.size() - 1];
        pub_local_goal.publish(local_goal);
        pub_final_goal.publish(final_goal);
    }
}

void Local_Goal_Creator::process(){
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(got_global_path){
            set_the_next_goal();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc,char **argv){
    ros::init(argc, argv, "Local_goal_creator");
    Local_Goal_Creator Local_goal_creator;
    Local_goal_creator.process();
}
