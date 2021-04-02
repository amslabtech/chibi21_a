#include <chibi21_a_amcl/chibi21_a_amcl.h>

//乱数生成
std::random_device seed;
std::mt19937 engine(seed());

Particle::Particle():private_nh("~")
{
    //param
    private_nh.getParam("N",N);
    private_nh.getParam("INIT_X",INIT_X);
    private_nh.getParam("INIT_Y",INIT_Y);
    private_nh.getParam("INIT_YAW",INIT_YAW);
    private_nh.getParam("INIT_X_COV",INIT_X_COV);
    private_nh.getParam("INIT_Y_COV",INIT_Y_COV);
    private_nh.getParam("INIT_YAW_COV",INIT_YAW_COV);

    private_nh.param("hz",hz,{10});

    //subscriber
    sub_odometry = nh.subscribe("/roomba/odometry",100,&Particle::odometry_callback,this);
    sub_laserscan = nh.subscribe("/scan",100,&Particle::laserscan_callback,this);
    sub_map = nh.subscribe("/map",100,&Particle::map_callback,this);

    //publisher
    pub_estimated_pose = nh.advertise<geometry_msgs::PoseStamped>("estimated_pose",1);
    pub_p_poses = nh.advertise<geometry_msgs::PoseArray>("p_poses",1);

    poses.header.frame_id = "map";

    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0),pose.pose.orientation);
    weight = 1.0 / (double)N;
}

void Particle::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odometry = *msg;
}

void Particle::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laserscan = *msg;
}

void Particle::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;

    ROS_INFO("Get map");

    std::vector<Particle> init_particles;
    for(int i = 0; i < N; i++)
    {
        Particle p;
        p.p_init(INIT_X,INIT_Y,INIT_YAW,INIT_X_COV,INIT_Y_COV,INIT_YAW_COV);
        poses.poses.push_back(p.pose.pose);
        init_particles.push_back(p);
    }
    particles = init_particles;
}

//Particle 初期化
void Particle::p_init(double x, double y, double yaw, double cov_x, double cov_y, double cov_yaw)
{
    std::normal_distribution<> dist_x(x,cov_x);
    pose.pose.position.x = dist_x(engine);

    std::normal_distribution<> dist_y(y,cov_y);
    pose.pose.position.y = dist_y(engine);

    std::normal_distribution<> dist_yaw(yaw,cov_yaw);
    quaternionTFToMsg(tf::createQuaternionFromYaw(dist_yaw(engine)),pose.pose.orientation);
}

void Particle::p_motion_update()
{
    for(int i = 0; i < N; i++)
    {
        particles[i].p_move();
    }
    previous_odo = current_odo;
}

void Particle::p_move()
{
    tf::Point p_position;
    tf::Point current_position;
    tf::Point previous_position;
    tf::Quaternion p_orientation;
    tf::Quaternion current_orientation;
    tf::Quaternion previous_orientation;

    //計算のためgeometry_msgsからtfへ変換
    pointMsgToTF(pose.pose.position, p_position);
    pointMsgToTF(current_odo.pose.pose.position, current_position);
    pointMsgToTF(previous_odo.pose.pose.position, previous_position);
    quaternionMsgToTF(pose.pose.orientation, p_orientation);
    quaternionMsgToTF(current_odo.pose.pose.orientation, current_orientation);
    quaternionMsgToTF(previous_odo.pose.pose.orientation, previous_orientation);

    tf::Quaternion d_orientation = current_orientation * previous_orientation.inverse();

    pointTFToMsg(p_position+current_position-previous_position, pose.pose.position);
    quaternionTFToMsg(p_orientation*d_orientation, pose.pose.orientation);
}


void Particle::process()
{
    ros::Rate rate(hz);
    while(ros::ok())
    {
        //pub_estimated_pose.publish(estimated_pose);
        pub_p_poses.publish(poses);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"chibi21_a_amcl");
    ROS_INFO("Start AMCL");
    Particle particle;
    particle.process();
    return 0;
}
