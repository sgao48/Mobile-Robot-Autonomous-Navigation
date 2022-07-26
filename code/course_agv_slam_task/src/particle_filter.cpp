#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>
#include <chrono>
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <ctime>
#include <cstring>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace Eigen;

#define particle_num 500
typedef struct particle {
    int id;
    float x;
    float y;
    float theta;
    float weight;
} particle;

class particle_filter{

public:
    particle_filter(ros::NodeHandle &n);
    ~particle_filter();
    ros::NodeHandle& n;

    // some params
    float init_x;
    float init_y;
    float init_theta;
    float init_rand_xy;
    float init_rand_theta;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher particles_pub;
    tf::TransformBroadcaster tf_broadcaster;

    // particles 
    particle particles[particle_num];

    // global state
    Eigen::Vector3d state;
    // map
    nav_msgs::OccupancyGrid global_map;
    bool isMapSet = false;

    // set map
    void setMap(nav_msgs::OccupancyGrid input);
    // init the state and particles 
    void init();
    // do Motion from ICP odom
    void doMotion(nav_msgs::Odometry input);
    // do Observation to weight each particle
    void doObservation(sensor_msgs::LaserScan input);
    // publish the particles and the estimated state
    void publishAll();
    // angle normalization
    double angleNorm(double angle);
    // weight normalization
    void weightNorm();
    // re-sample the particle according to the weights
    void resampling();
    // get the final pose 
    void getFinalPose();
    // gen new particle 
    particle genNewParticle();

    int mot_num = 0, obv_num = 0;
    double N_th;
    double z_hit, std_dev;
    double ratio;
    Vector3d icp_pose;
    nav_msgs::OccupancyGrid lh_map;
    ros::Publisher lhmap_pub;
    ros::Publisher error_pub;
    tf::TransformListener listener;

    // pose to transform matrix
    Matrix3d staToMatrix(Vector3d sta);
    // transform matrix to pose
    Vector3d getPose(Matrix3d T);

    ros::Publisher pose_pub;
};

particle_filter::~particle_filter()
{}

particle_filter::particle_filter(ros::NodeHandle& n):
    n(n)
{
    n.getParam("/particle_filter/init_x", init_x);
    n.getParam("/particle_filter/init_y", init_y);
    n.getParam("/particle_filter/init_theta", init_theta);

    n.getParam("/particle_filter/init_rand_xy", init_rand_xy);
    n.getParam("/particle_filter/init_rand_theta", init_rand_theta);

    n.getParam("/particle_filter/N_th", N_th);
    n.getParam("/particle_filter/z_hit", z_hit);
    n.getParam("/particle_filter/std_dev", std_dev);
    n.getParam("/particle_filter/ratio", ratio);

    this->init();

    particles_pub = n.advertise<visualization_msgs::MarkerArray>("particles", 0, true);
    map_sub = n.subscribe("/map", 1, &particle_filter::setMap, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &particle_filter::doObservation, this);
    odom_sub = n.subscribe("/icp_odom", 1, &particle_filter::doMotion, this);

    lhmap_pub = n.advertise<nav_msgs::OccupancyGrid>("lh_map", 0, true);
    error_pub = n.advertise<std_msgs::Float64>("particle_error", 1);

    pose_pub = n.advertise<geometry_msgs::Pose>("particle_loc", 1);
}

void particle_filter::setMap(nav_msgs::OccupancyGrid input)
{
    // set once at first time
    if(!isMapSet)
    {   
        cout << "init the global occupancy grid map" << endl;
        this->global_map = input;
        isMapSet = true;

        int height = global_map.info.height;
        int width = global_map.info.width;
        for(int i = 0; i < height*width; i++){
            if(global_map.data[i] == -1)
                global_map.data[i] = 100;
        }

        lh_map = this->global_map;
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                int lh_ind = j*width + i;

                if(lh_map.data[lh_ind] == 0){
                    double x, y, distance = 999999;
                    x = lh_map.info.origin.position.x + (i+1/2)*lh_map.info.resolution;
                    y = lh_map.info.origin.position.y + (j+1/2)*lh_map.info.resolution;
                    for(int n = 0; n < height; n++){
                        for(int m = 0; m < width; m++){
                            int global_ind = n*width + m;

                            if(global_map.data[global_ind] != 100)
                                continue;

                            double temp_x, temp_y, temp_dis;
                            temp_x = global_map.info.origin.position.x + (m+1/2)*global_map.info.resolution;
                            temp_y = global_map.info.origin.position.y + (n+1/2)*global_map.info.resolution;

                            temp_dis = sqrt(pow(x-temp_x, 2) + pow(y-temp_y, 2));
                            if(temp_dis < distance)
                                distance = temp_dis;
                        }
                    }

                    double f = 1/(sqrt(2*M_PI)*std_dev) * exp(-pow(distance, 2)/(2*pow(std_dev, 2)));
                    lh_map.data[lh_ind] = 100 * z_hit * f;
                }
            }
        }
    }

    lhmap_pub.publish(lh_map);
}

void particle_filter::init()
{   
    srand((int)time(0));

    // set state
    state << 0, 0, 0;
    icp_pose << 0, 0, 0;

    for(int i=0; i<particle_num; i++)
    {
        particles[i].id = i;
        particles[i].x = init_x + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].y = init_y + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].theta = init_theta + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_theta - init_rand_theta;
        particles[i].weight = float(1/float(particle_num)); // same weight
    }
}

void particle_filter::doMotion(nav_msgs::Odometry input)
{   
    if(mot_num > obv_num)
        return;

    // get yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(input.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // calculate pose
    Vector3d global_pose(input.pose.pose.position.x, input.pose.pose.position.y, angleNorm(yaw));
       
    // icp
    Vector3d last_pose(icp_pose(0), icp_pose(1), icp_pose(2));
    icp_pose = global_pose;

    // get local pose
    Matrix3d last = staToMatrix(last_pose), now = staToMatrix(global_pose);
    Matrix3d nowTolast = last.inverse()*now;
    Vector3d local_pose = getPose(nowTolast);
                
    // incremental status
    Vector3d delta_motion;
    delta_motion << cos(last_pose(2))*local_pose(0) - sin(last_pose(2))*local_pose(1),
                    sin(last_pose(2))*local_pose(0) + cos(last_pose(2))*local_pose(1),
                    local_pose(2);

    for(int i = 0; i < particle_num; i++)
    {
        const double mean = 0.0;
        const double stdxy = 1;
        const double stdt = 0.1;

        unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);

        std::normal_distribution<double> noisexy(mean, stdxy);
        std::normal_distribution<double> noiset(mean, stdt);

        // update
        particles[i].x += (1 + 3*noisexy(generator)) * delta_motion(0);
        particles[i].y += (1 + 3*noisexy(generator)) * delta_motion(1);
        particles[i].theta += (1 + 3*noiset(generator)) * delta_motion(2);
    }

    mot_num++;
}

void particle_filter::doObservation(sensor_msgs::LaserScan input)
{
    if(obv_num >= mot_num)
        return;

    double sum_w = 0.0;
    int laser_num = (input.angle_max-input.angle_min)/input.angle_increment + 1;
    for(int i = 0; i < particle_num; i++){
        particles[i].weight = 0.0;
        for(int j = 0; j < laser_num; j++){
            if(input.ranges[j] >= (input.range_max-0.01))
                continue;

            double x, y, angle;
            angle = input.angle_min + j*input.angle_increment;
            x = particles[i].x + input.ranges[j] * cos(angle+particles[i].theta);
            y = particles[i].y + input.ranges[j] * sin(angle+particles[i].theta);

            int x_ind, y_ind;
            x_ind = (x-lh_map.info.origin.position.x) / lh_map.info.resolution;
            y_ind = (y-lh_map.info.origin.position.y) / lh_map.info.resolution;

            int index = y_ind*lh_map.info.width + x_ind;
            particles[i].weight += lh_map.data[index];
        }

        sum_w += particles[i].weight;
    }

    // normalization
    for(int i = 0; i < particle_num; i++)
        particles[i].weight /= sum_w;

    double mean_w = 1.0/particle_num, sum_wsqr = 0;
    for(int i = 0; i < particle_num; i++)
        sum_wsqr += pow(particles[i].weight-mean_w, 2);

    this->getFinalPose();

    double N_eff = 1.0/sum_wsqr;
    cout << N_eff << endl;
    if(N_eff < N_th){
        this->resampling();
        for(int i = 0; i < particle_num; i++)
            particles[i].weight = 1.0/particle_num;
    }

    this->publishAll();
    obv_num++;
}

void particle_filter::resampling()
{
    int N = ratio*particle_num;

    vector<double> sum_w;
    sum_w.push_back(particles[0].weight);
    for(int i = 1; i < particle_num; i++)
        sum_w.push_back(sum_w[i-1] + particles[i].weight);
        
    particle temp_p[N];
    for(int i = 0; i < N; i++){
        double rand_num = rand()/(double)RAND_MAX;
        
        int j;
        for(j = 0; j < sum_w.size(); j++){
            if(rand_num <= sum_w[j])
                break;            
        }
        
        temp_p[i] = particles[j];
    }

    for(int i = 0; i < particle_num; i++){
        if(i < N)
            particles[i] = temp_p[i];
        else
            particles[i] = genNewParticle();
    }
}

void particle_filter::getFinalPose()
{   
    Vector3d res(0, 0, 0);
    for(int i = 0; i < particle_num; i++)
    {
        res(0) += particles[i].x * particles[i].weight;
        res(1) += particles[i].y * particles[i].weight;
        res(2) += particles[i].theta * particles[i].weight;
    }
    state = res;

    tf::StampedTransform transform;

    try{
		listener.lookupTransform("map", "robot_base", ros::Time(0), transform);

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();

        double dx = abs(x-state(0));
        double dy = abs(y-state(1));

        std_msgs::Float64 err;
        err.data = sqrt(dx*dx + dy*dy);
        error_pub.publish(err);
    }
    catch (exception &ex) {
		ROS_ERROR("%s",ex.what());
    }
}

particle particle_filter::genNewParticle()
{
    const double mean = 0.0;
    const double stdxy = 3;
    const double stdt = 0.1;

    unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> noisexy(mean, stdxy);
    std::normal_distribution<double> noiset(mean, stdt);
 
    particle temp;
    do{
        temp.x = state(0) + noisexy(generator);
        temp.y = state(1) + noisexy(generator);
        temp.theta = state(2) + noiset(generator);
    }while(abs(temp.x)>=9 || abs(temp.y)>=9);

    return temp;
}

double particle_filter::angleNorm(double angle)
{
    // -180 ~ 180
    while(angle > M_PI)
        angle = angle - 2*M_PI;
    while(angle < -M_PI)
        angle = angle + 2*M_PI;
    return angle;
}

Matrix3d particle_filter::staToMatrix(Vector3d sta)
{
    Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)), sta(1),
          0, 0, 1;
    return RT;
}

Vector3d particle_filter::getPose(Matrix3d T)
{
    Vector3d pose;
    pose(0) = T(0, 2);
    pose(1) = T(1, 2);
    pose(2) = angleNorm(atan2(T(1, 0), T(0, 0)));
    return pose;
}

void particle_filter::publishAll()
{
    visualization_msgs::MarkerArray particle_markers_msg;
    particle_markers_msg.markers.resize(particle_num);

    for(int i=0; i<particle_num; i++)
    {
        particle_markers_msg.markers[i].header.frame_id = "map";
        particle_markers_msg.markers[i].header.stamp = ros::Time::now();
        particle_markers_msg.markers[i].ns = "particle";
        particle_markers_msg.markers[i].id = i;
        particle_markers_msg.markers[i].type = visualization_msgs::Marker::ARROW;
        particle_markers_msg.markers[i].action = visualization_msgs::Marker::ADD;
        particle_markers_msg.markers[i].pose.position.x = particles[i].x;
        particle_markers_msg.markers[i].pose.position.y = particles[i].y;
        particle_markers_msg.markers[i].pose.position.z = 0; // add height for viz ?
        particle_markers_msg.markers[i].pose.orientation.x = 0.0;
        particle_markers_msg.markers[i].pose.orientation.y = 0.0;
        particle_markers_msg.markers[i].pose.orientation.z = sin(particles[i].theta/2);
        particle_markers_msg.markers[i].pose.orientation.w = cos(particles[i].theta/2);
        particle_markers_msg.markers[i].scale.x = 0.1;
        particle_markers_msg.markers[i].scale.y = 0.02;
        particle_markers_msg.markers[i].scale.z = 0.05;
        // particle_markers_msg.markers[i].color.a = particles[i].weight * particle_num / 2; // Don't forget to set the alpha!
        particle_markers_msg.markers[i].color.a = 0.5;
        particle_markers_msg.markers[i].color.r = 1.0;
        particle_markers_msg.markers[i].color.g = 0.0;
        particle_markers_msg.markers[i].color.b = 0.0;
    }
    particles_pub.publish(particle_markers_msg);

    // tf
    geometry_msgs::Quaternion quat_ = tf::createQuaternionMsgFromYaw(state(2));

    geometry_msgs::TransformStamped pf_trans;
    pf_trans.header.stamp = ros::Time::now();
    pf_trans.header.frame_id = "map";
    pf_trans.child_frame_id = "pf_loc";

    pf_trans.transform.translation.x = state(0);
    pf_trans.transform.translation.y = state(1);
    pf_trans.transform.translation.z = 0.0;
    pf_trans.transform.rotation = quat_;
    tf_broadcaster.sendTransform(pf_trans);

    // odom
    geometry_msgs::Pose odom;

    odom.position.x = state(0);
    odom.position.y = state(1);
    odom.position.z = 0.0;
    odom.orientation = quat_;

    pose_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    particle_filter particle_filter_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}