#ifndef PLANEDETECTOR_H
#define PLANEDETECTOR_H

#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "AR/DataConverter.h"
#include <opencv2/core/core.hpp>

class PlaneFunction{
public:
    float a,b,c,d;
    float norm2,norm;//for fast distance calculation
public:
    PlaneFunction();
    PlaneFunction(double _a, double _b, double _c, double _d);
    std::vector<cv::Point3f> generateFakePlaneData(double x_start=-10,double x_end=10,double y_start=-10,double y_end=10,double interval=1.0);
    friend std::ostream &operator<<(std::ostream &os,const PlaneFunction pf);
};

class Plane
{
public:
    Plane();
    Plane(const float nx, const float ny, const float nz, const float ox, const float oy, const float oz);
    Plane(cv::Mat _n, cv::Mat _o);
    PlaneFunction toPLaneFunction();
public:
    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //transform from world to plane
    cv::Mat Tpc;
};

class PlaneDetector{
private:
//    ros::Publisher normal_pub_;
//    ros::NodeHandle nh_;
//    tf::TransformBroadcaster broadcaster_;
public:
    PlaneDetector();
    Plane detectPlane(std::vector<cv::Point3f>& points,int iteration=50);//use iterative RANSAC to detect plane
};

#endif
