#include <chibi21_a_amcl/chibi21_a_amcl2.h>
//諦めてクラスを分割

//乱数生成
std::random_device seed;
std::mt19937 engine(seed());

AMCL::AMCL():private_nh("~")
{
    //param
    private_nh.getParam("N",N);
    private_nh.getParam("INIT_X",INIT_X);
    private_nh.getParam("INIT_Y",INIT_Y);
    private_nh.getParam("INIT_YAW",INIT_YAW);
    private_nh.getParam("INIT_X_COV",INIT_X_COV);
    private_nh.getParam("INIT_Y_COV",INIT_Y_COV);
    private_nh.getParam("INIT_YAW_COV",INIT_YAW_COV);
    private_nh.getParam("MOVE_DIST_COV",MOVE_DIST_COV);
    private_nh.getParam("MOVE_YAW_COV",MOVE_YAW_COV);
    private_nh.getParam("ANGLE_INC",ANGLE_INC);
    private_nh.getParam("MAX_RANGE",MAX_RANGE);
    private_nh.getParam("CHECK_INTERVAL",CHECK_INTERVAL);
    private_nh.getParam("M_WEIGHT",M_WEIGHT);
    private_nh.getParam("M_COV",M_COV);
    private_nh.getParam("ESS_LIMEN",ESS_LIMEN);
    private_nh.getParam("ALPHA_SLOW",ALPHA_SLOW);
    private_nh.getParam("ALPHA_FAST",ALPHA_FAST);

    private_nh.param("hz",hz,{10});

    //subscriber
    sub_odometry = nh.subscribe("/roomba/odometry",10,&AMCL::odometry_callback,this);
    sub_laserscan = nh.subscribe("/scan",10,&AMCL::laserscan_callback,this);
    sub_map = nh.subscribe("/map",10,&AMCL::map_callback,this);

    //publisher
    pub_estimated_pose = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose",1);
    pub_p_poses = nh.advertise<geometry_msgs::PoseArray>("p_poses",1);

    poses.header.frame_id = "map";
    estimated_pose.header.frame_id = "map";

    odo_get = false;
    map_get = false;
    scan_get = false;

    first_update = true;
}

void AMCL::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry = *msg;
    odo_get = true;
}

void AMCL::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laserscan = *msg;
    scan_get = true;
}

void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;

    ROS_INFO("Get map");
    map_get = true;

    std::vector<Particle> init_particles;
    for(int i = 0; i < N; i++)
    {
        Particle p(N);
        p_init(p);
        poses.poses.push_back(p.pose.pose);
        init_particles.push_back(p);
    }
    particles = init_particles;

    ROS_INFO("Particles init");
}

AMCL::Particle::Particle(int N)
{
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    weight = 1.0 / (double)N;
}

//Particle 初期化
void AMCL::p_init(AMCL::Particle &p)
{
    std::normal_distribution<> dist_x(INIT_X,INIT_X_COV);
    p.pose.pose.position.x = dist_x(engine);

    std::normal_distribution<> dist_y(INIT_Y,INIT_Y_COV);
    p.pose.pose.position.y = dist_y(engine);

    std::normal_distribution<> dist_yaw(INIT_YAW,INIT_YAW_COV);
    quaternionTFToMsg(tf::createQuaternionFromYaw(dist_yaw(engine)),p.pose.pose.orientation);

    quaternionMsgToTF(p.pose.pose.orientation, p.init_orientation);
}

void AMCL::p_motion_update()
{
    current_odo = odometry;
    if(first_update) previous_odo = current_odo;

    for(int i = 0; i < N; i++)
    {
       p_move(particles[i]);
       first_update = false;
    }
    previous_odo = current_odo;

    // ROS_INFO("motion_update");
}

void AMCL::p_move(AMCL::Particle &p)
{
    tf::Point p_position;
    tf::Point current_position;
    tf::Point previous_position;
    tf::Quaternion p_orientation;
    tf::Quaternion current_orientation;
    tf::Quaternion previous_orientation;

    //計算のためgeometry_msgsからtfへ変換
    pointMsgToTF(p.pose.pose.position, p_position);
    pointMsgToTF(current_odo.pose.pose.position, current_position);
    pointMsgToTF(previous_odo.pose.pose.position, previous_position);
    quaternionMsgToTF(p.pose.pose.orientation, p_orientation);
    quaternionMsgToTF(current_odo.pose.pose.orientation, current_orientation);
    quaternionMsgToTF(previous_odo.pose.pose.orientation, previous_orientation);

    if(current_orientation==previous_orientation && current_position==previous_position)
    {
        odo_move = false;
    }
    else odo_move = true;

    if(odo_move)
    {
        tf::Quaternion d_orientation = current_orientation * previous_orientation.inverse();
        tf::Point d_position = current_position - previous_position;

        tf::Point d_dist;
        d_dist.setValue(d_position.length(),0.0,0.0);   //大きさ=移動距離の正面向きベクトル

        std::normal_distribution<> move_dist_d(0.0,MOVE_DIST_COV);
        tf::Point dist_dist;
        dist_dist.setValue(move_dist_d(engine),0.0,0.0);   //移動距離をばらつかせるベクトル

        std::normal_distribution<> move_yaw_d(0.0,MOVE_YAW_COV);    //角度をばらつか(ry
        tf::Quaternion orientation_dist = tf::createQuaternionFromYaw(move_yaw_d(engine));

        pointTFToMsg(p_position + quatRotate(p_orientation, d_dist + dist_dist), p.pose.pose.position);
        quaternionTFToMsg(p_orientation*d_orientation*orientation_dist, p.pose.pose.orientation);
    }
}

void AMCL::p_measurement_update()
{
    for(int i = 0; i < N; i++)
    {
        p_calc_weight(particles[i]);
    }

    // std::cout<<"weight="<<particles[0].weight<<std::endl;

    // ROS_INFO("measurement_update");

    weight_normalize();

    // ROS_INFO("normalize");
}

void AMCL::weight_normalize()
{
    double w_sum = 0.0;

    for(int i = 0; i < N; i++)
    {
        w_sum += particles[i].weight;
    }

    max_weight = 0.0;
    for(int i = 0; i < N; i++)
    {
        particles[i].weight /= w_sum;
        //resampling_wheel用に重みの最大値を出しておく
        if(max_weight < particles[i].weight) max_weight = particles[i].weight;
    }
}

void AMCL::p_calc_weight(AMCL::Particle &p)
{
    p.weight = 0.0;
    double p_map_x = p.pose.pose.position.x - map.info.origin.position.x;
    double p_map_y = p.pose.pose.position.y - map.info.origin.position.y;

    if(!is_wall(p_map_x,p_map_y))
    {
        double wall_range = 0.0;
        double range_diff = 0.0;

        p.weight = 0.0;

        for(int i = 0; i < (int)laserscan.ranges.size(); i += ANGLE_INC)
        {
            wall_range = get_wall_range(laserscan.angle_min + i*laserscan.angle_increment, p);
            range_diff = wall_range - laserscan.ranges[i];

            p.weight += M_WEIGHT * exp((-range_diff*range_diff)/(2.0 * M_COV * M_COV));
        }
    }
    // ROS_INFO("calc_weight");
}

double AMCL::get_wall_range(double laser_angle, AMCL::Particle p)
{
    //作戦
    //セルのことは一旦無視してレーザーの方向に向かって一定間隔で点を移動させる
    //点の位置をセルの番号に変換して該当するセルが壁か否かを調べる
    //壁の角をかすった場合などはすり抜ける可能性もあるけど気にしない．

    double angle = laser_angle + tf::getYaw(p.pose.pose.orientation); //壁までの距離を測定する方向

    //マップ座標に変換
    //パーティクルの座標
    double p_x = p.pose.pose.position.x - map.info.origin.position.x;
    double p_y = p.pose.pose.position.y - map.info.origin.position.y;

    double check_x = p_x;
    double check_y = p_y;

    for(double range = 0.0; range < MAX_RANGE; range += CHECK_INTERVAL)
    {
        check_x = p_x + range*cos(angle);
        check_y = p_y + range*sin(angle);

        if(is_wall(check_x,check_y)) return range;
    }

    return MAX_RANGE;
}

bool AMCL::is_wall(double x, double y)
{
    int cx = (int)(x / map.info.resolution);
    int cy = (int)(y / map.info.resolution);

    // std::cout<<"cx="<<cx<<",cy="<<cy<<std::endl;

    if(cx > (int)map.info.width || cx < 0 ) return true;
    if(cy > (int)map.info.height || cy < 0 ) return true;

    if(map.data[cx + cy*map.info.width] != 0) return true;

    return false;
}

void AMCL::resampling_process()
{
    if(calc_ess() < ESS_LIMEN*(double)N)
    {
        resampling();
    }
    // ROS_INFO("resampling process");
    // std::cout<<"ESS="<<calc_ess()<<std::endl;
}

double AMCL::calc_ess()
{
    double sum = 0.0;

    for(int i = 0; i < N; i++)
    {
        sum += particles[i].weight*particles[i].weight;
    }

    return 1.0 / sum;
}

void AMCL::resampling()
{
    std::vector<Particle> next_particles;

    std::uniform_int_distribution<> int_dist(0,N-1);
    int index = int_dist(engine);
    std::uniform_real_distribution<> real_dist(0.0,max_weight*2.0);
    for(int i = 0; i < N; i++)
    {
       double beta = real_dist(engine);
       while(beta > particles[index].weight)
       {
           beta -= particles[index].weight;
           index++;
           index = index%N;
       }
       next_particles.push_back(particles[index]);
    }
    particles = next_particles;
    // ROS_INFO("resampling");
}

void AMCL::pose_estimate()
{
    tf::Point sum_pos;
    sum_pos.setZero();

    int max_w_num = 0;

    double sum_w = 0.0;

    for(int i = 0; i < (int)particles.size(); i++)
    {
        tf::Point p_position;
        pointMsgToTF(particles[i].pose.pose.position, p_position);
        sum_pos += p_position * particles[i].weight;

        if(particles[max_w_num].weight < particles[i].weight) max_w_num = i;
        sum_w += particles[i].weight;
    }

    tf::Point ave_pos = sum_pos / sum_w;
    pointTFToMsg(ave_pos, estimated_pose.pose.position);
    estimated_pose.pose.orientation = particles[max_w_num].pose.pose.orientation;   //最も尤度が高い向き

    // std::cout<<estimated_pose.pose<<std::endl;
}

double AMCL::fix_yaw(double yaw)
{
    if(yaw < 0.0) return yaw + 2*M_PI;

    return yaw;
}

void AMCL::p_disp_update()
{
    for(int i = 0; i < N; i++)
    {
        poses.poses[i] = particles[i].pose.pose;
    }
}

void AMCL::process()
{
    ros::Rate rate(hz);
    while(ros::ok())
    {
        if(map_get && odo_get && scan_get)
        {
            p_motion_update();
            if(odo_move)
            {
                p_measurement_update();
                pose_estimate();
                resampling_process();
            }
            p_disp_update();
        }

        pub_estimated_pose.publish(estimated_pose);
        pub_p_poses.publish(poses);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"chibi21_a_amcl");
    ROS_INFO("Start AMCL");
    AMCL amcl;
    amcl.process();
    return 0;
}
