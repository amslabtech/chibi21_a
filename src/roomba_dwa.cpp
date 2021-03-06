#include<roomba_dwa/roomba_dwa.h>
#include<stdio.h>

DWA::DWA():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("dt",dt,{0.5});
    private_nh.param("to_goal_gain",to_goal_gain,{1.0});
    private_nh.param("robot_distance_gain",robot_distance_gain,{1.0});
    private_nh.param("speed_gain",speed_gain,{1.0});
    private_nh.param("v_reso",v_reso,{0.1});
    private_nh.param("omega_reso",omega_reso,{0.1});
    private_nh.param("max_dyawrate",max_dyawrate,{10});//degree
    private_nh.param("max_accel",max_accel,{0.4});
    private_nh.param("safemass_x",safemass_x,{10});
    private_nh.param("safemass_y",safemass_y,{10});
    private_nh.param("predict_time",predict_time,{3.0});

    max_dyawrate *= (M_PI/180);

    sub_local_goal = nh.subscribe("local_goal",10,&DWA::local_goal_callback,this);
    sub_pose = nh.subscribe("estimated_pose",10,&DWA::pose_callback,this);
    sub_local_map = nh.subscribe("local_map",10,&DWA::local_map_callback,this);//local_map
    sub_final_goal = nh.subscribe("final_goal",10,&DWA::final_goal_callback,this);

    pub_twist = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_predict_path = nh.advertise<nav_msgs::Path>("predict_path",1);//rvizにひげを配信するためのもの
    pub_goal_point = nh.advertise<geometry_msgs::PointStamped>("goal_point",1);//goalをrvizに配信
    pub_best_predict_path = nh.advertise<nav_msgs::Path>("best_predict_path",1);//ベストなパスをrvizに配信
}

void DWA::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_goal = *msg;
}

void DWA::final_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    final_goal = *msg;
}

void DWA::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    local_map = *msg;
    if(local_map.data.size() != 0) set_map();
}

void DWA::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose = *msg;
}

void DWA::set_map()
{
    column= local_map.info.width;
    row = local_map.info.height;
    resolution = local_map.info.resolution;
    map.resize(row,std::vector<int>(column));

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            map[i][j] = local_map.data[j*row+i];
        }
    }

    receive_local_map = true;
}

void DWA::calc_dynamic_window(const Roomba_state &state)
{
    std::vector<float> Vs = {min_speed,max_speed,-max_yawrate,max_yawrate};
    std::vector<float> Vd = {
        state.v - max_accel,
        state.v + max_accel,
        state.omega - max_dyawrate,
        state.omega + max_dyawrate,
    };
    dw = {
        std::max(Vs[0],Vd[0]),
        std::min(Vs[1],Vd[1]),
        std::max(Vs[2],Vd[2]),
        std::min(Vs[3],Vd[3]),
    };
}

void DWA::calc_trajectory(const float &v, const float &y)
{
    traj.clear();
    predict_path.poses.clear();
    state = {0,0,0.0,0.0,0.0};
    traj.push_back(state);
    for(float time =0; time <= predict_time; time+=dt)
    {
        state.yaw += y * dt;
        state.x += v * std::sin(state.yaw) * dt;
        state.y += v * std::cos(state.yaw) * dt;
        state.v = v;
        state.omega = y;
        traj.push_back(state);

        geometry_msgs::PoseStamped path_point;
        path_point.pose.position.x = state.x;
        path_point.pose.position.y = state.y;
        predict_path.poses.push_back(path_point);
    }

    predict_path.header.frame_id = "base_link";
    pub_predict_path.publish(predict_path);
}

float DWA::calc_to_goal_cost()
{
    theta = std::atan2(goal[0],goal[1]) - std::atan2(traj.back().x,traj.back().y);
    cost = fabs(std::atan2(sin(theta),cos(theta)));

    geometry_msgs::PointStamped goal_point;
    goal_point.header.frame_id = "base_link";
    goal_point.point.x = (double)goal[0];
    goal_point.point.y = (double)goal[1];
    pub_goal_point.publish(goal_point);

    return cost;
}

void DWA::calc_final_input()
{
    min_cost = 1e10;
    best_traj.clear();
    final_cost = 0.0;

    for(float v=dw[0]; v<=dw[1]; v+=v_reso)
    {
        for(float y=dw[2]; y<=dw[3]; y+=omega_reso)
        {
            calc_trajectory(v,y);
            to_goal_cost = calc_to_goal_cost();
            speed_cost = max_speed - traj.back().v;
            ob_cost = calc_obstacle_cost();
            final_cost = to_goal_gain*to_goal_cost + speed_gain*speed_cost +  robot_distance_gain*ob_cost;
            if(min_cost >= final_cost)
            {
                min_cost = final_cost;
                min_m.v = v;
                min_m.omega = y;
                best_traj = traj;
            }
            if(min_ob_cost >= ob_cost)
            {
                min_ob_cost = ob_cost;
            }

        }
    }
    get_best_traj();
}

void DWA::get_best_traj()
{
    best_predict_path.poses.clear();
    for(auto& state : best_traj)
    {
        geometry_msgs::PoseStamped best_path_point;
        best_path_point.pose.position.x = state.x;
        best_path_point.pose.position.y = state.y;
        best_predict_path.poses.push_back(best_path_point);
    }
    best_predict_path.header.frame_id = "base_link";
    pub_best_predict_path.publish(best_predict_path);

}

void DWA::dwa_control()
{
    state = {0,0,0.0,0.0,0.0};

    calc_dynamic_window(state);
    calc_final_input();
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    if(wait < 10)
    {
        cmd_vel.cntl.linear.x = 0.0;
        cmd_vel.mode = 11;
        pub_twist.publish(cmd_vel);
        wait++;
    }
    else
    {
        cmd_vel.cntl.linear.x = min_m.v;
        cmd_vel.cntl.angular.z = min_m.omega;
        cmd_vel.mode = 11;
        pub_twist.publish(cmd_vel);
    }
    std::cout << "final"  << final_goal.pose.position.x << std::endl;
    std::cout << "final y"  << final_goal.pose.position.y << std::endl;
    float final_distance_x = pose.pose.position.x - local_goal.pose.position.x;
    float final_distance_y = pose.pose.position.y - local_goal.pose.position.y;
    if(sqrt(final_distance_x*final_distance_x + final_distance_y*final_distance_y) <= 0.3)
    {
        std::cout << "goal" << std::endl;
        cmd_vel.cntl.linear.x = 0.0;
        cmd_vel.cntl.angular.z = 0.0;
        cmd_vel.mode = 11;
        pub_twist.publish(cmd_vel);

    }

    if(min_ob_cost == 1e10)
    {
        cmd_vel.cntl.linear.x = 0.0;
        cmd_vel.cntl.angular.z = 0.2;
        cmd_vel.mode = 11;
        pub_twist.publish(cmd_vel);
    }

}

float DWA::calc_obstacle_cost()
{
    minr = 1e5;
    max_cost = 0.0;

    for(auto& state : traj)
    {
        int x = (int)((state.x - (-world/2)) /resolution);
        int y = (int)((state.y - (-world/2)) /resolution);

        for(int i=x-safemass_x; i<x+safemass_x; i++)
        {
            for(int j=y-safemass_y; j<y+safemass_y; j++)
            {
                if(i > 0 && j > 0 && (map[i][j] == 100 /*|| map[i][j] == -1*/))
                {
                    a = float(-world/2+i*resolution);//ロボットから見た座標系
                    b = float(-world/2+ j*resolution);

                    distance = sqrt((state.x -a)*(state.x -a) + (state.y -b)*(state.y - b));
                    if(b < 0)
                    {
                        continue;
                    }

                    if(distance <= 0.2)
                    {
                        distance = sqrt((state.x -a)*(state.x -a) + (state.y -b)*(state.y - b));
                        max_cost = 1e10;
                    }
                    if(minr >= distance)
                    {
                        minr = distance;
                    }
                }
            }
        }
    }

    if(max_cost > minr) return max_cost;
    else return 1.0/minr;
}

void DWA::roomba()
{
    goal.resize(2);
    tf::Quaternion quat(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    conversion_theta = -1* float(yaw-M_PI/2);

    dwa_control();
    conversion_x = local_goal.pose.position.x - pose.pose.position.x;
    conversion_y = local_goal.pose.position.y - pose.pose.position.y;

    goal[0] =-1*( conversion_x*std::cos(conversion_theta) - conversion_y*std::sin(conversion_theta));

    goal[1] = conversion_x*std::sin(conversion_theta)+ conversion_y*std::cos(conversion_theta);

    receive_local_map = false;
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        if(receive_local_map) roomba();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roomba");
    DWA DWA;
    DWA.process();
    return 0;
}
