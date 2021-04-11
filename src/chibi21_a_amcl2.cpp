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
    private_nh.getParam("ANGLE_INC",ANGLE_INC);
    private_nh.getParam("MAX_RANGE",MAX_RANGE);
    private_nh.getParam("CHECK_INTERVAL",CHECK_INTERVAL);
    private_nh.getParam("M_WEIGHT",M_WEIGHT);
    private_nh.getParam("M_COV",M_COV);

    private_nh.param("hz",hz,{10});

    //subscriber
    sub_odometry = nh.subscribe("/roomba/odometry",10,&AMCL::odometry_callback,this);
    sub_laserscan = nh.subscribe("/scan",10,&AMCL::laserscan_callback,this);
    sub_map = nh.subscribe("/map",10,&AMCL::map_callback,this);

    //publisher
    pub_estimated_pose = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose",1);
    pub_p_poses = nh.advertise<geometry_msgs::PoseArray>("p_poses",1);

    poses.header.frame_id = "map";

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

    // for(int i = 0; i < N; i++)
    // {
        // std::cout<<init_particles[i].pose<<std::endl;
    // }

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

    ROS_INFO("motion_update");
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

    tf::Quaternion d_orientation = current_orientation * previous_orientation.inverse();
    tf::Point d_position = current_position - previous_position;

    pointTFToMsg(p_position + quatRotate(p.init_orientation, d_position), p.pose.pose.position);
    quaternionTFToMsg(p_orientation*d_orientation, p.pose.pose.orientation);

    if(current_orientation==previous_orientation && current_position==previous_position)
    {
        odo_move = false;
    }
    else odo_move = true;
}

void AMCL::p_measurement_update()
{
    for(int i = 0; i < N; i++)
    {
        p_calc_weight(particles[i]);
    }

    std::cout<<"weight="<<particles[0].weight<<std::endl;

    ROS_INFO("measurement_update");

    weight_normalize();

    ROS_INFO("normalize");
}

void AMCL::weight_normalize()
{
    double w_sum = 0.0;

    for(int i = 0; i < N; i++)
    {
        w_sum += particles[i].weight;
    }

    for(int i = 0; i < N; i++)
    {
        particles[i].weight /= w_sum;
    }
}

void AMCL::p_calc_weight(AMCL::Particle &p)
{
    double wall_range = 0.0;
    double range_diff = 0.0;

    for(int i = 0; i < (int)laserscan.ranges.size(); i += ANGLE_INC)
    {
        // std::cout<<"ANGLE_INC="<<ANGLE_INC<<std::endl;
        wall_range = get_wall_range(laserscan.angle_min + i*laserscan.angle_increment, p);
        range_diff = wall_range - laserscan.ranges[i];

        p.weight = M_WEIGHT * exp((-range_diff*range_diff)/(2.0 * M_COV * M_COV));
    }
    ROS_INFO("calc_weight");
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

    if(cx > (int)map.info.width || cx < 0 ) return true;
    if(cy > (int)map.info.height || cy < 0 ) return true;

    if(map.data[cx + cy*map.info.width] != 0) return true;

    return false;
}

/*
double AMCL::Particle::get_wall_range(double laser_angle, nav_msgs::OccupancyGrid map)
{
    //次の横線との交点を算出
    //次の縦線との交点を算出
    //それぞれの交点までの距離を比較
    //距離の大小関係が逆転するまでグリッドを横または縦に移動

    double angle = laser_angle + getYaw(pose.pose.orientation); //壁までの距離を測定する方向

    //マップ座標に変換
    //パーティクルの座標
    double p_x = pose.pose.position.x - map.origin.position.x;
    double p_y = pose.pose.position.y - map.origin.position.y;
    //直線の端の座標
    double max_x = pose.pose.position.x + MAX_RANGE*cos(angle) - map.origin.position.x;
    double max_y = pose.pose.position.y + MAX_RANGE*sin(angle) - map.origin.position.y;

    //直線の傾きによって判別
    bool slope_judge;
    if(fabs(max_x-p_x) > fabs(max_y-p_y)) slope_judge = true;
    else slope_judge = false;

    //セルに変換
    int p_cell_x = (int)(p_x / map.resolution);
    int p_cell_y = (int)(p_y / map.resolution);
    int max_cell_x = (int)(max_x / map.resolution);
    int max_cell_y = (int)(max_y / map.resolution);

    if(map.data[p_cell_x + p_cell_y*map.width] != 0) return 0.0;  //かべのなかにいる

    //グリッドとの交点
    double inter_x = 0.0;
    double inter_y = 0.0;

    double ind_v;   //独立変数．こちらをresolutionずつ足していく
    double dep_v;

    if(slope_judge)
    {
    }
}

double AMCL::Particle::get_wall_range2(double laser_angle, nav_msgs::OccupancyGrid map)
{
    double angle = laser_angle + getYaw(pose.pose.orientation); //壁までの距離を測定する方向

    //マップ座標に変換
    //パーティクルの座標
    double p_x = pose.pose.position.x - map.origin.position.x;
    double p_y = pose.pose.position.y - map.origin.position.y;
    //直線の端の座標
    double max_x = pose.pose.position.x + MAX_RANGE*cos(angle) - map.origin.position.x;
    double max_y = pose.pose.position.y + MAX_RANGE*sin(angle) - map.origin.position.y;

    //セルに変換
    int p_cell_x = (int)(p_x / map.resolution);
    int p_cell_y = (int)(p_y / map.resolution);
    int max_cell_x = (int)(max_x / map.resolution);
    int max_cell_y = (int)(max_y / map.resolution);

    int dx = 0;
    int dy = 0;

    if(max_cell_x - p_cell_x > 0) dx = 1;
    else (max_cell_x - p_cell_x < 0) dx = -1;
    else (max_cell_x - p_cell_x = 0) dx = 0;

    if(max_cell_y - p_cell_y > 0) dy = 1;
    else (max_cell_y - p_cell_y < 0) dy = -1;
    else (max_cell_y - p_cell_y = 0) dy = 0;
}
*/

void AMCL::p_disp_update()
{
    // geometry_msgs::PoseArray disp_poses;
    for(int i = 0; i < N; i++)
    {
        poses.poses[i] = particles[i].pose.pose;
    }
    // poses.poses = disp_poses.poses;
}

void AMCL::process()
{
    ros::Rate rate(hz);
    while(ros::ok())
    {
        //pub_estimated_pose.publish(estimated_pose);
        // ROS_INFO("hoge");
        pub_p_poses.publish(poses);
        // ROS_INFO("fuga");

        if(map_get && odo_get && scan_get)
        {
            p_motion_update();
            if(odo_move) p_measurement_update();
            p_disp_update();
        }

        ros::spinOnce();
        // ROS_INFO("foo");
        rate.sleep();
        // ROS_INFO("foo foo");
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
