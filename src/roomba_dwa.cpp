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
    pub_goal_point = nh.advertise<geometry_msgs::PointStamped>("goal_point",1);//goalをrvizに配信
    pub_best_predict_path = nh.advertise<nav_msgs::Path>("best_predict_path",1);//ベストなパスをrvizに配信
    pub_ob_check = nh.advertise<geometry_msgs::PointStamped>("ob_check",1);//回避すべき障害物をrvizに配信
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
    state = {row/2*resolution,column/2*resolution,0.0,0.0,0.0};
    traj.push_back(state);
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
    traj.back().x = traj.back().x - row/2*resolution;
    traj.back().y = traj.back().y - column/2*resolution;

    traj_magnitude = sqrt(traj.back().x*traj.back().x +traj.back().y*traj.back().y);
    goal_magnitude = sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
    goal[0] = 3 + row/2*resolution;
    goal[1] = 0.3 + column/2*resolution;
    traj.back().x = traj.back().x + row/2*resolution;
    traj.back().y = traj.back().y + column/2*resolution;

    geometry_msgs::PointStamped goal_point;
    goal_point.header.frame_id = "map";
    goal_point.point.x = (double)goal[0];
    goal_point.point.y = (double)goal[1];
    pub_goal_point.publish(goal_point);

    inner_product = ((goal[0]*traj.back().x) + (goal[1]*traj.back().y))/(goal_magnitude*traj_magnitude);
    inner_product_angle = std::acos(inner_product);
    cost = to_goal_gain * inner_product_angle;
    return cost;
}

void DWA::calc_final_input()
{
    min_cost = 1e5;

    for(float v=dw[0]; v<=dw[1]; v+=0.1)
    {
        for(float y=dw[2]; y<=dw[3]; y+=0.1)
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
    get_best_traj();
}

void DWA::get_best_traj()
{
    best_predict_path.poses.clear();
    for(auto& state : best_traj)
    {
        geometry_msgs::PoseStamped best_path_point;
        best_path_point.pose.position.x = state.x + row/2 * resolution;
        best_path_point.pose.position.y = state.y + column/2 * resolution;
        best_predict_path.poses.push_back(best_path_point);
    }
    best_predict_path.header.frame_id = "map";
    pub_best_predict_path.publish(best_predict_path);

}

void DWA::dwa_control()
{
    /*tf::Quaternion quat(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    theta = float(yaw);
    state.x = pose.pose.position.x;
    state.y = pose.pose.position.y;
    state.yaw = yaw;*/
    state = {row/2*resolution,column/2*resolution,0.0,0.0,0.0};//テスト

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
    //ここアルゴリズムが違う。範囲for文ですべてのひげのx,y座標と障害物の距離を計算する。

    minr = std::numeric_limits<float>::infinity();
    std::cout << "minr" << minr << std::endl;

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            if(map[i][j] == 100 || map[i][j] == -1)
            {
                float a = i*resolution;
                float b = j*resolution;
                for(auto& state : traj)
                {
                    distance = sqrt((state.x-a)*(state.x-a) + (state.y -b)*(state.y - b));
                    //std::cout<<"state.x" <<state.x <<std::endl;
                    //std::cout<<"state.y" <<state.y <<std::endl;
                    if(distance <= roomba_radius)
                    {
                        geometry_msgs::PointStamped ob_check;
                        ob_check.point.x = a;
                        ob_check.point.y = b;

                        /*std::cout << "distance" << distance << std::endl;
                        std::cout << "collision!" << std::endl;
                        std::cout << "a,b" << a << "," << b << std::endl;*/

                        ob_check.header.frame_id = "map";
                        pub_ob_check.publish(ob_check);
                        return 1e10;
                    }
                    if(minr >= distance)
                    {
                        minr = distance;
                    }
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
    goal[0] = 5.0;
    goal[1] = 1.0;
    //std::cout<< "goal,x,y" << goal[0]<< "," << goal[1] << std::endl;
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
