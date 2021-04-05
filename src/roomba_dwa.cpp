#include<roomba_dwa/roomba_dwa.h>
#include<stdio.h>
//#include<math.h>

DWA::DWA():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("dt",dt,{0.5});
    private_nh.param("to_goal_gain",to_goal_gain,{1.0});
    private_nh.param("robot_distance_gain",robot_distance_gain,{1.0});
    private_nh.param("speed_gain",speed_gain,{1.0});
    private_nh.param("predict_time",predict_time,{3.0});

    //sub_local_goal = nh.subscribe("",10,&DWA::local_goal_callback,this);
    //sub_pose = nh.subscribe("",10,&DWA::pose_callback,this);
    sub_local_map = nh.subscribe("local_map",10,&DWA::local_map_callback,this);//local_map

    pub_twist = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_predict_path = nh.advertise<nav_msgs::Path>("predict_path",1);//rvizにひげを配信するためのもの
}

void DWA::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //local_goal = *msg;
}

void DWA::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std::cout <<"local_map" <<std::endl;
    local_map = *msg;
    if(local_map.data.size() != 0) set_map();
}

void DWA::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //pose = *msg;
}

void DWA::set_map()
{
    std::cout << "received map" << std::endl;
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
    state = {0.0,0.0,0.0,0.0,0.0};
    for(float time =0; time <= predict_time; time+=dt)
    {
        state.yaw += y * dt;
        state.x += v * std::cos(state.yaw) * dt;
        state.y += v * std::sin(state.yaw) * dt;
        state.v = v;
        state.omega = y;
        traj.push_back(state);

        geometry_msgs::PoseStamped path_point;
        path_point.pose.position.x = state.x;
        path_point.pose.position.y = state.y;
        predict_path.poses.push_back(path_point);
    }

    predict_path.header.frame_id = "map";//test
    pub_predict_path.publish(predict_path);
}

float DWA::calc_to_goal_cost()
{
    //ゴールがロボット座標系かどうか分からん
    goal_magnitude = sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
    traj_magnitude = sqrt(traj.back().x*traj.back().x +traj.back().y*traj.back().y);
    inner_product = ((goal[0]*traj.back().x) + (goal[1]*traj.back().y))/(goal_magnitude*traj_magnitude);
    inner_product_angle = std::acos(inner_product);
    cost = to_goal_gain * inner_product_angle;
    return cost;
}

void DWA::calc_final_input()
{
    min_cost = 100000;

    for(float v=dw[0]; v<=dw[1]; v+=0.05)
    {
        for(float y=dw[2]; y<=dw[3]; y+=0.05)
        {
            calc_trajectory(v,y);
            to_goal_cost = calc_to_goal_cost();
            speed_cost = max_speed - traj.back().v;
            ob_cost = calc_obstacle_cost();
            final_cost = to_goal_gain * to_goal_cost + speed_gain * speed_cost + robot_distance_gain * ob_cost;
            if(min_cost >= final_cost)
            {
                min_cost = final_cost;
                min_m.v = v;
                min_m.omega = y;
                best_traj = traj;
            }

            //all_predict_path.poses.insert(all_predict_path.poses.end(),predict_path.poses.begin(),predict_path.poses.end());
        }
    }
    std::cout << "min_m,v,y" << min_m.v<<"," << min_m.omega << std::endl;

}

void DWA::dwa_control()
{
    /*tf::Quaternion quat(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    theta = float(yaw);
    state.x = pose.pose.position.x;
    state.y = pose.pose.position.y;
    state.yaw = yaw;*/
    state = {0.0,0.0,0.0,0.0,0.0};//テスト

    calc_dynamic_window(state);
    calc_final_input();
/*
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.linear.x = min_m.v;
    cmd_vel.cntl.angular.z = min_m.omega;
    cmd_vel.mode = 11;
    pub_twist.publish(cmd_vel);
    */
}

float DWA::calc_obstacle_cost()
{
    //ロボット座標系からグリッドマップ座標系に変換
    x = (int)(column/2 + traj.back().x*resolution);
    y = (int)(row/2 + traj.back().y*resolution);
    minr = std::numeric_limits<float>::infinity();

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            if(map[i][j] == 100 || map[i][j] == -1)
            {
                distance = sqrt((x-i)*(x-i) + (y -j)*(y - j));
                if(distance <= roomba_radius)
                {
                    std::cout << "collision!" << std::endl;
                    return minr;
                }
                if(minr >= distance)
                {
                    minr = distance;
                }
            }
        }
    }
    return 1.0/minr;

}

//処理の最後にreceive_local_mapをfalseにする
void DWA::roomba()
{
    goal.resize(2);
    //y座標が逆になってる
    goal[0] = 10;
    goal[1] = 0;//テスト
    std::cout<< "goal,x,y" << goal[0]<< "," << goal[1] << std::endl;
    dwa_control();
    if(sqrt((state.x - goal[0]*state.x)*(state.x - goal[0]) + (state.y - goal[1]) * (state.y - goal[1])) <= roomba_radius)
    {
        //最終地点
        std::cout << "goal" << std::endl;
    }
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        roomba();
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
