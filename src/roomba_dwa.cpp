#include<roomba_dwa.h>
//#include<math.h>

DWA::DWA():private_nh("~"), map(mass, std::vector<int>(mass))
{
    private_nh.param("hz",hz,{10});
    private_nh.param("dt",dt,{0.1});
    // printf("gg\n");
    //sub_path = nh.subscribe("",10,&DWA::path_callback,this);
    //sub_range = nh.subscribe("/scan",10,&DWA::ranges_callback,this);
    sub_point = nh.subscribe("/roomba/odometry",10,&DWA::point_callback,this);
    sub_local_map = nh.subscribe("/local_map",10,&DWA::local_map_callback,this);//local_map
    sub_local_goal = nh.subscribe("/local_goal",10,&DWA::local_goal_callback,this);
    sub_estimated_pose = nh.subscribe("/estimated_pose",10,&DWA::estimated_pose_callback,this);
    pub_twist = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
    pub_goal_point = nh.advertise<geometry_msgs::PointStamped>("goal_point",1);
    pub_best_predict_path = nh.advertise<nav_msgs::Path>("best_predict_path",1);//ベストなパスをrvizに配信
    pub_ob_check = nh.advertise<geometry_msgs::PointStamped>("ob_check",1);//回避すべき障害物をrvizに配信
    pub_predict_path = nh.advertise<nav_msgs::Path>("predict_path",1);
    // std::cout << "start!!" << std::endl;
    // printf("gg\n");

}

void DWA::point_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    point = *msg;
}

void DWA::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

    local_map = *msg;
    if(local_map.data.size()!=0) set_map();
}

void DWA::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_goal = *msg;
}

void DWA::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    estimated_pose = *msg;
}

void DWA::set_map()
{
    column= local_map.info.width;
    row = local_map.info.height;
    resolution = local_map.info.resolution;
    // map.clear();
    map.resize(row,std::vector<int>(column));

// printf("ff\n");
    for(int i=0; i<row; i++) {
        for(int j=0; j<column; j++) {
            map[i][j] = local_map.data[j*row+i];
            // printf("map:%d\n", map[i][j]);
        }
    }
    receive_local_map = true;
}

/*
            map[safemass_y][safemass_x] = 0;//白
            map[obstacle_y][obstacle_x] = 100;//黒
*/

void DWA::roomba()
{
    nav_msgs::Path predict_path;
    roomba_500driver_meiji::RoombaCtrl v;
    old_pose_x = now_pose_x;
    old_pose_y = now_pose_y;
    // now_pose_x = estimated_pose.pose.position.x;
    // now_pose_y = estimated_pose.pose.position.y;
    // printf("x,y:%f,%f\n",now_pose_x,now_pose_y);
    now_pose_x = 0;
    now_pose_y = 0;
    now_speed = std::sqrt((now_pose_x - old_pose_x) * (now_pose_x - old_pose_x) + (now_pose_y - old_pose_y) * (now_pose_y - old_pose_y)) / dt;
    v2_max = now_speed + max_accel * dt;
    v2_min = now_speed - max_accel * dt;

    geometry_msgs::PointStamped goal_point;
    goal_point.header.frame_id="base_link";
    goal_point.point.x = local_goal.pose.position.x;
    goal_point.point.y = local_goal.pose.position.y;
    pub_goal_point.publish(goal_point);
    // printf("x,y:%f,%f\n",local_goal.pose.position.x,local_goal.pose.position.y);
    // printf("max,min:%f,%f\n", v2_max,v2_min);

    if(max_speed <= v2_max) v2_max = max_speed;
    if(min_speed >= v2_min) v2_min = min_speed;

    tf::Quaternion quat(estimated_pose.pose.orientation.x,estimated_pose.pose.orientation.y,estimated_pose.pose.orientation.z,estimated_pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    theta = float(yaw);

    old_angle = now_angle;
    // now_angle = theta;
    now_angle = 0;
    // printf("theta:%f\n",theta);
    now_yawrate = (old_angle - now_angle) /dt;
    omega2_max = now_yawrate + max_dyawrate * dt;
    omega2_min = now_yawrate - max_dyawrate * dt;

    // printf("angle:%f\n", now_angle);
    if(max_yawrate <= omega2_max) omega2_max = max_yawrate;
    if(max_yawrate * -1 >= omega2_min) omega2_min = max_yawrate * -1;
       // std::cout<<"v2_max,v2_min,v_reso:"<<v2_max<<","<<v2_min<<","<<v_reso<<std::endl;
    max_evaluation = 0;
    // min_obstacle = 20;
    for(float a=v2_min; a<=v2_max; a+=v_reso) {//次の時刻における速度v2
        // std::cout<<"omega2_max,omega2_min,omega_reso"<<omega2_max<<","<<omega2_min<<","<<omega_reso<<std::endl;
        for(float b=omega2_min; b<=omega2_max; b+=omega_reso) {
            // std::cout<<"loop2"<<std::endl;
            velocity = a/v2_max;
            min_obstacle = 14.14;
            obstacle = 14.14;
            predict_path.poses.clear();
            for(float time=0; time<=predict_time;time+=0.01) {
                x2 = a*time*std::sin(b*time);
                y2 = a*time*std::cos(b*time);
                geometry_msgs::PoseStamped path_point;
                path_point.pose.position.x = x2;
                path_point.pose.position.y = y2;
                predict_path.poses.push_back(path_point);
                x=(int)(x2/mass_reso)+column/2;
                y=(int)(y2/mass_reso)+row/2;
                // printf("map:%d\n",map[i][j]);
                // if(x<0 || y<0) continue;
                // printf("x,y:%d, %d\n", x,y);
                if(map[x][y] != 0) {
                    break;
                }
                if(time>=predict_time-0.01) {
                    heading_theta = std::atan2(local_goal.pose.position.y,local_goal.pose.position.x) - std::atan2(y2,x2);
                    heading = (std::cos(heading_theta)+1)/2;

                        //heading = std::atan2(((float)y2-local_goal.pose.positon.y),((float)x2-local_goal.pose.position.x));
                        // if(heading<0) heading *=-1;
                }
                    // printf("x2,y2:%d, %d\n", x2,y2);
                for(int i=x-safe_space; i<x+safe_space; ++i) {
                    for(int j=y-safe_space; j<y+safe_space; ++j){
                        if(j<=row/2+3 || i==50 || j>=80) continue;
                            // if(j<row/2) continue;
                            // printf("i,j,x,y:%d,%d,%d,%d",i,j,x,y);
                            // printf("map:%d\n",map[i][j]);
                            // if(map[i][j] == 0) obstacle=20;
                        if(map[i][j] == 100 || map[i][j] == -1) {
                               // if(j<=y) continue;
                               // printf("i,j,x,y,map:%d,%d,%d,%d,%d\n",i,j,x,y,map[i][j]);
                            obstacle = std::sqrt((i-x)*(i-x)+(j-y)*(j-y));
                                // if(obstacle==0) continue;
                                // printf("obstacle:%f\n",obstacle);
                        }
                            // printf("obstacle:%f\n",obstacle);
                        if(min_obstacle > obstacle) {
                            min_obstacle = obstacle;
                                // printf("obstacle:%f\n",obstacle);
                                // printf("i,j,x,y:%d,%d,%d,%d\n",i,j,x,y);
                        }
                    }
                }
            // dist = min_obstacle/(safe_space*1.42);
            // printf("min_obs:%f\n",min_obstacle);
            }
            predict_path.header.frame_id = "base_link";
            pub_predict_path.publish(predict_path);
            // printf("min_obs:%f\n",min_obstacle);
            dist = min_obstacle/(safe_space*1.414);
            // printf("dist:%f\n",dist);
            // printf("velocity,heading,dist:%f,%f,%f\n", velocity,heading,dist);
            evaluation = to_goal_gain * heading + robot_distance_gain * dist + speed_gain * velocity;
            // evaluation = speed_gain * velocity + to_goal_gain * heading;
            // evaluation = robot_distance_gain * dist;
            // printf("eva:%f\n",evaluation);
            if(max_evaluation < evaluation) {
                max_evaluation = evaluation;
                fin_v = a;
                fin_omega = b;
                // printf("max_evaluation:%f\n", max_evaluation);
            }
        }
    }
    v.cntl.linear.x = fin_v;
    v.cntl.angular.z = fin_omega;
    // printf("fin_v,fin_omega:%f,%f\n", fin_v,fin_omega);
    nav_msgs::Path best_predict_path;
    for(float time=0; time<=predict_time;time+=0.1) {
        fin_x = fin_v*time*std::sin(fin_omega*time);
        fin_y = fin_v*time*std::cos(fin_omega*time);
        geometry_msgs::PoseStamped best_path_point;
        best_path_point.pose.position.x = fin_x;
        best_path_point.pose.position.y = fin_y;
        best_predict_path.poses.push_back(best_path_point);
        // std::cout<<"fin_x,fin_y: "<<fin_x<<","<<fin_y<<std::endl;
    }
    v.mode = 11;
pub_twist.publish(v);
best_predict_path.header.frame_id = "base_link";
pub_best_predict_path.publish(best_predict_path);
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        //go_straight();
        // std::cout<<"process start\n";
        if(receive_local_map) roomba();
        // std::cout<<"roomba() end\n";
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    // std::cout<<"DWA NODE\n";
    ros::init(argc, argv, "roomba");
    // std::cout<<"dwa node\n";
    DWA DWA;
    // std::cout<<"started dwa\n";
    DWA.process();
    // wait(0.01);
    return 0;
}
