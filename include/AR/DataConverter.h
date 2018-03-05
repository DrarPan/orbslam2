#ifndef DATACONVERTER_H
#define DATACONVERTER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "Camera.h"

struct ColorPoint{
    cv::Point3f point;
    cv::Scalar color;
};

typedef std::vector<ColorPoint> ColorPointCloud;

cv::Mat ExpSO3(const float &x, const float &y, const float &z);
cv::Mat ExpSO3(cv::Mat v);
std::vector<cv::Point3f> generateFakePlaneData(double a,double b,double c,double d);
std::vector<cv::Point3f> depthImage2Point3d(cv::Mat depthimg,int skip=8);
std::vector<cv::Point3f> depthImage2Point3d(cv::Mat depthimg,double fx,double fy,double cx,double cy,int skip=8);

void XYZ2uv(double X,double Y,double Z,double &u,double &v);
void uv2XYZ(double u,double v,double d,double &X,double &Y,double &Z);
bool inImage(cv::Size s,double& u,double& v);
bool inImage(cv::Size s,int& u,int& v);
ColorPoint TransfromColorPoint(const cv::Mat& T,ColorPoint p);
std::vector<ColorPoint> TransfromColorPointCloud(cv::Mat& T,const std::vector<ColorPoint>& vp);
Eigen::Matrix4f cvMat44toEigenMatrix4(const cv::Mat& T);
std::vector<ColorPoint> TransfromColorPointCloud(Eigen::Matrix4f T,const std::vector<ColorPoint>& vp);
std::vector<ColorPoint> image2ColorPointCloud(cv::Mat img, double pixel_scale=1, cv::Point3f base=cv::Point3f(0,0,0), bool laid=true, bool remove_background=true, bool inversex=false);
#endif
