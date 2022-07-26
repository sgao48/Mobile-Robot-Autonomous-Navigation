#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace Eigen;

typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp_lm{

public:

    icp_lm(ros::NodeHandle &n);
    ~icp_lm();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // sensor states = robot_x_y_theta
    Vector3d sensor_sta;

    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;
    // min match_cnt
    int min_match;

    // main process
    void process(visualization_msgs::MarkerArray input);
    // landMarks to Eigen::Matrix
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;

    // odometry related param
    double leftv = 0.0;
    double rightv = 0.0;
    double timelast;
    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;
    double theta = 0.0;
    bool flag = false;

    double rx = 1.0/0.08;
    double rw = (0.1+0.08/6)/0.08;

    void calcinitRT(Eigen::Matrix2d &R, Eigen::Vector2d &T);
    void leftv_callback(const std_msgs::Float64::ConstPtr &msg);
    void rightv_callback(const std_msgs::Float64::ConstPtr &msg);
    ros::Subscriber leftv_sub;
    ros::Subscriber rightv_sub;

    // pose in gazebo
    void showPose(const geometry_msgs::Pose p);
    ros::Subscriber pose_sub;

    int skip_num = 0, jump_flag = 0, skip_thresh = 3;
};

icp_lm::~icp_lm()
{}

icp_lm::icp_lm(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/icp_lm/robot_x", robot_x);
	n.getParam("/icp_lm/robot_y", robot_y);
	n.getParam("/icp_lm/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

    n.getParam("/icp_lm/max_iter", max_iter);
	n.getParam("/icp_lm/tolerance", tolerance);
	n.getParam("/icp_lm/dis_th", dis_th);
    n.getParam("/icp_lm/min_match", min_match);

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &icp_lm::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);

    leftv_sub = n.subscribe("/course_agv/left_wheel_velocity_controller/command", 1, &icp_lm::leftv_callback, this);
    rightv_sub = n.subscribe("/course_agv/right_wheel_velocity_controller/command", 1, &icp_lm::rightv_callback, this);

    //pose_sub = n.subscribe("/gazebo/course_agv__robot_base", 1, &icp_lm::showPose, this);
}
// show robot real pose
void icp_lm::showPose(const geometry_msgs::Pose p){
    cout << "robot real position: " << p.position.x << "  " << p.position.y << endl;
}
// get left_wheel velocity
void icp_lm::leftv_callback(const std_msgs::Float64::ConstPtr& msg){
    leftv = msg->data;
}
// get right_wheel velocity and update odometry
void icp_lm::rightv_callback(const std_msgs::Float64::ConstPtr& msg){
    double t0 = (double)ros::Time::now().toSec();
    double dt = t0 - timelast;

    rightv = msg->data;
    if(flag){
        dx = 0;
        dy = 0;
        dtheta = 0;
        flag = false;
    }

    double vx = (leftv+rightv) / (2.0*rx);
    double vw = (rightv-leftv) / (2.0*rw);

    dx += vx*dt*cos(sensor_sta(2)+dtheta);
    dy += vx*dt*sin(sensor_sta(2)+dtheta);
    dtheta += vw*dt;
    timelast = t0;
}
// use odometry to initilaize RT
void icp_lm::calcinitRT(Eigen::Matrix2d &R, Eigen::Vector2d &T){
    Vector3d sta;
    sta << dx, dy, dtheta;

    Matrix3d RT = staToMatrix(sta);
    R = RT.block(0,0,2,2);
    T = RT.block(0,2,2,1);
    theta += dtheta;
    flag = true;
}

void icp_lm::process(visualization_msgs::MarkerArray input)
{   
    //cout<<"------Time:  "<<input.markers[0].header.stamp<<endl;
    
    double time_0 = (double)ros::Time::now().toSec();

    if(isFirstScan)
    {
        tar_pc = this->landMarksToMatrix(input);
        isFirstScan =false;
        return;
    }
    // source landmarks
    src_pc = landMarksToMatrix(input);
    MatrixXd src_copy = src_pc;

    // # of landmarks is not enough
    if(src_pc.cols() < min_match){
        skip_num++;
        if(skip_num >= skip_thresh)
            jump_flag = 1;
        return;
    }
    // need to jump over these frames
    if(jump_flag){
        tar_pc = this->landMarksToMatrix(input);

        this->timelast = (double)ros::Time::now().toSec();
        this->flag = true;

        // cout << "JUMP!!!" << endl;
        jump_flag = 0;
        return;
    }

    // if # of landmarks is enough for navigation
    if(src_pc.cols() >= min_match){
        double time_0 = (double)ros::Time::now().toSec();

        // init some variables
        Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);

        // main LOOP
        Matrix2d R_init = Transform_acc.block(0, 0, 2, 2);
        Vector2d t_init = Transform_acc.block(0, 2, 2, 1);
#define ODOMETRY // use odometry or not
#ifdef ODOMETRY
        calcinitRT(R_init, t_init);
        Transform_acc.block(0,0,2,2) = R_init;
        Transform_acc.block(0,2,2,1) = t_init;
#endif
        src_pc = Transform_acc*src_copy; // update source landmarks

        int i = 0;
        double error_last = 0.0;
        for(; i<max_iter; i++)
        {
            // TODO: please code by yourself
            // find the nearest point
            NeighBor neighbor = findNearest(src_pc, tar_pc);
            int N = neighbor.src_indices.size();

            // matched points are not enough
            if(N < min_match){
                skip_num++;
                if(skip_num >= skip_thresh)
                    jump_flag = 1;
                return;
            }
            skip_num = 0;

            MatrixXd src(3, N), tar(3, N);
            // matching
            for(int j = 0; j < N; j++){
                src.col(j) = src_pc.col(neighbor.src_indices[j]);
                tar.col(j) = tar_pc.col(neighbor.tar_indices[j]);
            }
            // calculate RT
            Matrix3d T = getTransform(src, tar);
            Matrix2d R = T.block(0, 0, 2, 2);
            Vector2d t = T.block(0, 2, 2, 1);
            // update
            t = R * t_init + t;
            R = R * R_init;
            R_init = R; t_init = t;
            // update source landmarks
            Transform_acc << R, t,
                             0, 0, 1;
            src_pc = Transform_acc * src_copy;
            // if the current RT satisfy the requirement
            double error = accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / N;
            if(error <= tolerance)
                break;
        }
        //cout << "iteration: " << i << endl;

        tar_pc = this->landMarksToMatrix(input);
        // make R be orthogonal
        JacobiSVD<Matrix2d> svd(Transform_acc.block(0, 0, 2, 2).transpose(), ComputeThinU | ComputeThinV);
        Matrix2d U = svd.matrixU();
        Matrix2d V = svd.matrixV();
        Transform_acc.block(0, 0, 2, 2) = V * U.transpose();
        this->publishResult(Transform_acc);

        double time_1 = (double)ros::Time::now().toSec();
        //cout<<"time_cost:  "<<time_1-time_0<<endl;
    }
}

Eigen::MatrixXd icp_lm::landMarksToMatrix(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();
    //cout<<markerSize<<" markers received !"<<endl;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

NeighBor icp_lm::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    vector<float> dist;
    vector<int> src_ind, tar_ind;
    NeighBor res;
    MatrixXd src_coord(2, src.cols()), tar_coord(2, tar.cols()), temp_coord(2, tar.cols());
    VectorXd distance(tar.cols());
    VectorXd::Index minInd;
    double minNum;

    src_coord = src.topRows(2);
    tar_coord = tar.topRows(2);

    for(int i = 0; i < src_coord.cols(); i++){
        temp_coord = tar_coord;
        temp_coord.colwise() -= src_coord.col(i);
        distance = temp_coord.array().square().colwise().sum();
        minNum = distance.minCoeff(&minInd); // find the nearest point

        if(minNum >= dis_th) // if minimum distance is to large
            continue;
        dist.push_back(sqrt(minNum));
        src_ind.push_back(i);
        tar_ind.push_back(minInd);
    }

    res.distances = dist;
    res.src_indices = src_ind;
    res.tar_indices = tar_ind;

    return res;
}

Eigen::Matrix3d icp_lm::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    Matrix2d W, U, V, R;
    MatrixXd src_coord(2, src.cols()), tar_coord(2, tar.cols());
    Vector2d src_mean, tar_mean, t;
    Matrix3d T;

    src_coord = src.topRows(2);
    tar_coord = tar.topRows(2);
    // centralization
    src_mean = src_coord.rowwise().mean();
    tar_mean = tar_coord.rowwise().mean();
    src_coord.colwise() -= src_mean;
    tar_coord.colwise() -= tar_mean;
    // SVD
    W = src_coord * tar_coord.transpose();
    JacobiSVD<Matrix2d> svd(W, ComputeThinU | ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();
    // calculate RT
    R = V * U.transpose();
    t = tar_mean - R*src_mean;
    T << R, t,
         0, 0, 1;

    return T;
}

float icp_lm::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
    float distance = sqrt((pta-ptb).array().square().sum());
    return distance;
}

Eigen::Matrix3d icp_lm::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp_lm::publishResult(Eigen::Matrix3d T)
{	
    float delta_yaw = atan2(T(1,0), T(0,0));
    //cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    // cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(sensor_sta(2));

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    //odom_trans.header.frame_id = "world_base";
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    //odom.header.frame_id = "world_base";
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_landMark");
    ros::NodeHandle n;

    icp_lm icp_lm_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}
