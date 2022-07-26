#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // match threshold;
    float match_th;
    // bool
    bool isFirstScan;
    // statusvisualization_msgs::MarkerArray
    VectorXd status;
    // covariance
    MatrixXd covariance;
    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
    // count the non-zero elements in status
    int nonZero_cnt;

    // init all 
    void initAll();
    // predict phase
    void predict(nav_msgs::Odometry odom);
    // update phase
    void update(visualization_msgs::MarkerArray input);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // get motion Jacobian
    MatrixXd getMotionJacobian(double theta, double x, double y);
    // get observation Jacobian
    MatrixXd getObservJacobian(Vector3d sta, double x, double y);
    // angle normalization
    double angleNorm(double angle);
    // find nearest map points
    int findNearestMap(Vector2d point);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
    tf::TransformBroadcaster ekf_broadcaster;
    void publishResult();
    void calcError();

    // noise M
    MatrixXd noise_M;
    // pose to transform matrix
    Matrix3d staToMatrix(Vector3d sta);
    // transform matrix to pose
    Vector3d getPose(Matrix3d T);
    // run times
    int pre_num, upd_num;
    // odom publisher
    ros::Publisher odom_pub;
    // landmark publisher
    ros::Publisher lm_pub;
    tf::TransformListener listener;
    ros::Publisher error_pub;
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    this->initAll();

    isFirstScan = true;
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this);
    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this);

    odom_pub = n.advertise<geometry_msgs::Pose>("ekf_slam", 1);
    lm_pub = n.advertise<visualization_msgs::MarkerArray>("ekf_landMarks", 1);
    error_pub = n.advertise<std_msgs::Float64>("ekf_error", 1);
}

void ekf::predict(nav_msgs::Odometry odom)
{
    // wait for finishing updating
    if(pre_num > upd_num)
        return;

    // TODO: Please complete the predict phase or motion model
    // cout << "----------PREDICT " << pre_num << "----------" << endl;
    // map 3 dims to 3N+3 dims
    MatrixXd F = MatrixXd::Zero(3, 3*(landMark_num+1));
    F.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
    // get yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // calculate pose
    Vector3d global_pose, last_pose;
    global_pose << odom.pose.pose.position.x, odom.pose.pose.position.y, angleNorm(yaw);
    last_pose << status(0), status(1), status(2);
    //cout << "icp: " << global_pose(0) << " " << global_pose(1) << " " << global_pose(2) << endl;
    // get local pose
    Matrix3d last = staToMatrix(last_pose);
    Matrix3d now = staToMatrix(global_pose);
    Matrix3d nowTolast = last.inverse() * now;
    Vector3d local_pose = getPose(nowTolast);
    // jacobian of motion model
    MatrixXd G = getMotionJacobian(last_pose(2), local_pose(0), local_pose(1));
    // incremental status
    Vector3d delta_motion;
    delta_motion << cos(last_pose(2))*local_pose(0) - sin(last_pose(2))*local_pose(1),
                    sin(last_pose(2))*local_pose(0) + cos(last_pose(2))*local_pose(1),
                    local_pose(2);
    // calculate noise matrix
    Matrix3d V;
    V << cos(last_pose(2)), -sin(last_pose(2)), 0,
         sin(last_pose(2)), cos(last_pose(2)), 0,
         0, 0, 1;
    noise_R = V * noise_M * V.transpose();
    // predict status & covariance
    status = status + F.transpose()*delta_motion;
    status(2) = angleNorm(status(2));
    covariance = G*covariance*G.transpose() + F.transpose()*noise_R*F;

    pre_num++;
}

void ekf::update(visualization_msgs::MarkerArray input)
{
    // wait for finishing predicting
    if(upd_num >= pre_num)
        return;

    cout << "----------UPDATE " << upd_num << "----------" << endl;
    double time_0 = (double)ros::Time::now().toSec();

    MatrixXd landMarkFeatures = this->landMarksToXY(input);

    // TODO: Please complete the update phase or observation model
    for(int i = 0; i < landMarkFeatures.cols(); i++){
        // global position
        Vector2d point;
        point << status(0) + cos(status(2))*landMarkFeatures(0, i) - sin(status(2))*landMarkFeatures(1, i),
                 status(1) + sin(status(2))*landMarkFeatures(0, i) + cos(status(2))*landMarkFeatures(1, i);
        // find the nearest feature
        int j = findNearestMap(point);
        // new feature
        if(j == -1){
            status(3*nonZero_cnt) = point(0);
            status(3*nonZero_cnt+1) = point(1);
            status(3*nonZero_cnt+2) = nonZero_cnt;
            j = nonZero_cnt++;  // (# of feature)++
        }
        // map 6 dims to 3N+3 dims
        MatrixXd F = MatrixXd::Zero(6, 3*(landMark_num+1));
        F.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
        F.block(3, 3*j, 3, 3) = MatrixXd::Identity(3, 3);
        // status of feature
        double xl = status(3*j), yl = status(3*j+1), s = status(3*j+2);
        // status of motion
        double xt = status(0), yt = status(1), theta = status(2);
        // observation & h(mu_bar)
        Vector3d z, z_hat;
        z << landMarkFeatures(0, i), landMarkFeatures(1, i), j;
        z_hat << xl*cos(theta) + yl*sin(theta) - xt*cos(theta) - yt*sin(theta),
                 -xl*sin(theta) + yl*cos(theta) + xt*sin(theta) - yt*cos(theta),
                 s;
        // jacobian of observation model
        Vector3d sta(xt, yt, theta);
        MatrixXd h = getObservJacobian(sta, xl, yl);
        MatrixXd H = h * F;
        // calculate K
        MatrixXd S = H*covariance*H.transpose() + noise_Q;
        MatrixXd K = covariance * H.transpose() * S.inverse();
        // update status & covariance
        status = status + K*(z-z_hat);
        status(2) = angleNorm(status(2));
        covariance = covariance - K*H*covariance;
    }
    
    this->calcError();
    this->publishResult();
    cout << "landmark_num: " << nonZero_cnt-1 << endl;
    cout << "update: " << status(0) << " " << status(1) << " " << status(2) << endl;

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;

    upd_num++;
}

void ekf::initAll()
{
    // TODO: You can initial here if you need
    // init status
    double yaw = angleNorm(robot_theta);
    status = VectorXd::Zero(3*(landMark_num+1));
    status(0) = robot_x;
    status(1) = robot_y;
    status(2) = yaw;
    // init covariance
    covariance = 999999 * MatrixXd::Identity(3*(landMark_num+1), 3*(landMark_num+1));
    covariance.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);
    // init noise
    noise_M = noise_motion * MatrixXd::Identity(3, 3);
    noise_R = noise_motion * MatrixXd::Identity(3, 3);
    noise_Q = noise_measure * MatrixXd::Identity(3, 3);

    nonZero_cnt = 1;    // init # of feature
    pre_num = 0; upd_num = 0;
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

int ekf::findNearestMap(Vector2d point)
{
    // TODO: Please complete the NN search
    // first scan
    if(nonZero_cnt == 1)
        return -1;
    // get positions of scanned features
    MatrixXd myMap(3*(nonZero_cnt-1), 1);
    myMap = status.segment(3, 3*(nonZero_cnt-1));
    myMap.resize(3, (nonZero_cnt-1));
    MatrixXd map_copy(2, (nonZero_cnt-1));
    map_copy = myMap.topRows(2);
    // find the nearest feature
    VectorXd distance((nonZero_cnt-1));
    VectorXd::Index minInd;
    map_copy.colwise() -= point;
    distance = map_copy.array().square().colwise().sum();
    double minNum = sqrt(distance.minCoeff(&minInd));

    minInd++;
    // adjust threshold with covariance
    double x_cov = 1.96*sqrt(covariance(3*minInd, 3*minInd));
    double y_cov = 1.96*sqrt(covariance(3*minInd+1, 3*minInd+1));
    double radius = sqrt(x_cov*x_cov + y_cov*y_cov);

    // existed feature or new
    if(minNum <= match_th+radius)
        return minInd;
    return -1;
}

Eigen::MatrixXd ekf::getMotionJacobian(double theta, double x, double y)
{
    // TODO: Please complete the Jocobian Calculation of Motion
    // map 3 dims to 3N+3 dims
    MatrixXd F = MatrixXd::Zero(3, 3*(landMark_num+1));
    F.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
    // jacobian
    Matrix3d J;
    J << 0, 0, -sin(theta)*x - cos(theta)*y,
         0, 0, cos(theta)*x - sin(theta)*y,
         0, 0, 0;
    // augmented jacobian
    MatrixXd G = MatrixXd::Identity(3*(landMark_num+1), 3*(landMark_num+1)) + F.transpose()*J*F;

    return G;
}

Eigen::MatrixXd ekf::getObservJacobian(Vector3d sta, double x, double y)
{
    // TODO: Please complete the Jocobian Calculation of Observation
    MatrixXd h(3, 6);
    double xt = sta(0), yt = sta(1), theta = sta(2);
    // jacobian
    h << -cos(theta), -sin(theta), sin(theta)*(-x+xt) + cos(theta)*(y-yt), cos(theta), sin(theta), 0,
         sin(theta), -cos(theta), sin(theta)*(-y+yt) + cos(theta)*(-x+xt), -sin(theta), cos(theta), 0,
         0, 0, 0, 0, 0, 1;

    return h;
}

void ekf::calcError()
{
    tf::StampedTransform transform;    
    
    try{
		listener.lookupTransform("map", "robot_base", ros::Time(0), transform);

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();

        double dx = abs(x-status(0));
        double dy = abs(y-status(1));

        std_msgs::Float64 err;
        err.data = sqrt(dx*dx + dy*dy);
        error_pub.publish(err);
    }
    catch (exception &ex) {
		ROS_ERROR("%s",ex.what());
    }
}

double ekf::angleNorm(double angle)
{
    // -180 ~ 180
    while(angle > M_PI)
        angle = angle - 2*M_PI;
    while(angle < -M_PI)
        angle = angle + 2*M_PI;
    return angle;
}

Matrix3d ekf::staToMatrix(Vector3d sta)
{
    Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)), sta(1),
          0, 0, 1;
    return RT;
}

Vector3d ekf::getPose(Matrix3d T)
{
    Vector3d pose;
    pose(0) = T(0, 2);
    pose(1) = T(1, 2);
    pose(2) = angleNorm(atan2(T(1, 0), T(0, 0)));
    return pose;
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    //ekf_trans.header.frame_id = "world_base";
    ekf_trans.header.frame_id = "map";
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0);
    ekf_trans.transform.translation.y = status(1);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    ekf_broadcaster.sendTransform(ekf_trans);

    // odom
    geometry_msgs::Pose odom;

    odom.position.x = status(0);
    odom.position.y = status(1);
    odom.position.z = 0.0;
    odom.orientation = odom_quat;

    odom_pub.publish(odom);

    // landmarks
    visualization_msgs::MarkerArray landMark_array_msg;

    landMark_array_msg.markers.resize(nonZero_cnt-1);
    for(int i=0; i<nonZero_cnt-1; i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "map";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "ekf_lm";
        landMark_array_msg.markers[i].id = i+1;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        landMark_array_msg.markers[i].pose.position.x = status(3*(i+1));
        landMark_array_msg.markers[i].pose.position.y = status(3*(i+1)+1);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.4;
        landMark_array_msg.markers[i].scale.y = 0.4;
        landMark_array_msg.markers[i].scale.z = 0.4;
        landMark_array_msg.markers[i].color.a = 0.5; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 1.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 0.0;
    }

    lm_pub.publish(landMark_array_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}
