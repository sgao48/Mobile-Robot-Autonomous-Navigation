#include "ros/ros.h"
#include "ros/console.h"
#include <iostream>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>

using namespace std;
using namespace Eigen;

// structure of the LandMark set, contians the landMarks 
typedef struct{
    std::vector<float> position_x;
    std::vector<float> position_y;
    std::vector<int> id;
} LandMarkSet;

class extraction{

public:
	extraction(ros::NodeHandle &n);
	~extraction();
    ros::NodeHandle& n;
 
    // get the clusters
    float range_threshold;
    // filter for landmarks extractions
    float radius_max_th;
    // filter for landmarks extractions
    int landMark_min_pt;

    // listen the ros::laserScan
    ros::Subscriber laser_sub;
    // publish the landMarks as ros::Markers
    ros::Publisher landMark_pub;

    // main process
    void process(sensor_msgs::LaserScan input);
    // filter & extraction process
    LandMarkSet extractLandMark(sensor_msgs::LaserScan input);
    // publish the landMarks
    void publishLandMark(LandMarkSet input);
    // 2D euclidean distance calculation
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
};

extraction::~extraction()
{}

extraction::extraction(ros::NodeHandle& n):
    n(n)
{   
    // get the params
    n.getParam("/extraction/range_threshold", range_threshold);
    n.getParam("/extraction/radius_max_th", radius_max_th);
    n.getParam("/extraction/landMark_min_pt", landMark_min_pt);
    
    landMark_pub = n.advertise<visualization_msgs::MarkerArray>("landMarks", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &extraction::process, this);
}

void extraction::process(sensor_msgs::LaserScan input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    int label = 0;
    sensor_msgs::LaserScan laser_extr = input;

    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    // init the previous/last point
    Vector2d last_xy(input.ranges[0] * std::cos(input.angle_min), input.ranges[0] * std::sin(input.angle_min));

    // set the intensities as labels
    Vector2d curr_xy;
    float angle, delta_dis;
    for(int i=0; i<total_num; i++)
    {   
        angle = input.angle_min + i * input.angle_increment;
        curr_xy << input.ranges[i] * std::cos(angle), input.ranges[i] * std::sin(angle);
        
        delta_dis = this->calc_dist(curr_xy, last_xy);

        if(delta_dis > range_threshold)
            label++;

        laser_extr.intensities[i] = label;

        last_xy = curr_xy;
    }

    //cout<<"Total original labels: "<<label<<endl;

    LandMarkSet landmarks_ = this->extractLandMark(laser_extr);

    this->publishLandMark(landmarks_);

    double time_1 = (double)ros::Time::now().toSec();
    //cout<<"time_cost:  "<<time_1-time_0<<endl;
}

LandMarkSet extraction::extractLandMark(sensor_msgs::LaserScan input)
{   
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    LandMarkSet landMarks;
    vector<float> x, y;
    vector<int> ind;

    // TODO: please code by yourself

    int flag = 0;
    for(int i = 0; i < total_num; i++){
        int cur = input.intensities[i], N = 0;
        Vector2d center = Vector2d::Zero();
        vector<double> ptx, pty;

        // find points with the same label
        for(; input.intensities[i] == cur; i++){
            float angle = input.angle_min + i * input.angle_increment;
            double cur_x = input.ranges[i] * cos(angle), cur_y = input.ranges[i] * sin(angle);

            // store points
            ptx.push_back(cur_x);
            pty.push_back(cur_y);
            // # of points
            N++;
        }

        // enough points for check
        if(N >= landMark_min_pt){
            // calculate the center
            center(0) = accumulate(ptx.begin(), ptx.end(), 0.0) / N;
            center(1) = accumulate(pty.begin(), pty.end(), 0.0) / N;

            // points with the same label
            MatrixXd points(2, N);
            for(int j = 0; j < N; j++){
                points(0, j) = ptx[j];
                points(1, j) = pty[j];
            }

            // calculate distance from center
            VectorXd distance(N);
            points.colwise() -= center;
            distance = points.array().square().colwise().sum();
            double maxNum = distance.maxCoeff();
            // if maximum distance satisfies cylinder's requirement
            if(sqrt(maxNum) < radius_max_th){
                int check = 1;
                for(int k = 0; k < total_num; k++){
                    Vector2d check_point;
                    float angle = input.angle_min + k * input.angle_increment;
                    check_point << input.ranges[k] * std::cos(angle), input.ranges[k] * std::sin(angle);
                    float check_dist = calc_dist(center, check_point);

                    if(input.intensities[k] == cur)
                        continue;
                    if(check_dist <= 1.0){
                        check = 0;
                        break;
                    }
                }

                if(check){
                    // update landMarkSet
                    x.push_back(center(0));
                    y.push_back(center(1));
                    ind.push_back(flag++);
                }
            }
        }

        i--;
    }

    landMarks.position_x = x;
    landMarks.position_y = y;
    landMarks.id = ind;

    return landMarks;
}

void extraction::publishLandMark(LandMarkSet input)
{
    if(input.id.size() <= 0)
        return;

    visualization_msgs::MarkerArray landMark_array_msg;

    landMark_array_msg.markers.resize(input.id.size());

    for(int i=0; i<input.id.size(); i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "course_agv__hokuyo__link";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "lm";
        landMark_array_msg.markers[i].id = i;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::MODIFY;
        landMark_array_msg.markers[i].pose.position.x = input.position_x.at(i);
        landMark_array_msg.markers[i].pose.position.y = input.position_y.at(i);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.2;
        landMark_array_msg.markers[i].scale.y = 0.2;
        landMark_array_msg.markers[i].scale.z = 0.2;
        landMark_array_msg.markers[i].color.a = 0.5; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 0.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 1.0;
    }

    landMark_pub.publish(landMark_array_msg);
}

float extraction::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
    float distance = sqrt((pta-ptb).array().square().sum());
    return distance;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extraction");
    ros::NodeHandle n;

    extraction extraction_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}