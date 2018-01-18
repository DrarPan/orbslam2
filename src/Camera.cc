#include <iostream>

#include "Camera.h"

using namespace std;

namespace ORB_SLAM2 {
std::string Camera::name;
float Camera::width=640;
float Camera::height=480;
float Camera::fx;
float Camera::fy;
float Camera::cx;
float Camera::cy;
float Camera::invfx;
float Camera::invfy;
float Camera::bf;
float Camera::b;
cv::Mat Camera::K=cv::Mat::eye(3,3,CV_32F);
cv::Mat Camera::DistCoef=cv::Mat::zeros(4,1,CV_32F);
float Camera::depthFactor=1.0;
bool Camera::initialized=false;


bool Camera::load(cv::FileStorage fSettings){
    width = fSettings["Camera.width"];
    height = fSettings["Camera.height"];
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];

    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.at<float>(4) = fSettings["Camera.k3"];

    const float k3 = fSettings["Camera.k3"];
    if(k3!=0){
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    bf = fSettings["Camera.bf"];

    invfx = 1.0/fx;
    invfy = 1.0/fy;
    b = bf/fx;

    initialized = true;
    return true;
}

bool Camera::load(string strSettingsFile){
    cv::FileStorage fs(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()){
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        return false;
    }
    return load(fs);
}

void Camera::show(){
    cout << "size: " << width << "x" <<  height << endl;
    cout << "fx: " << fx << endl;
    cout << "fy: " << fy << endl;
    cout << "cx: " << cx << endl;
    cout << "cy: " << cy << endl;
    cout << "k1: " << DistCoef.at<float>(0) << endl;
    cout << "k2: " << DistCoef.at<float>(1) << endl;
    cout << "k3: " << DistCoef.at<float>(4) << endl;
    cout << "p1: " << DistCoef.at<float>(2) << endl;
    cout << "p2: " << DistCoef.at<float>(3) << endl;
    cout << "bf: " << bf << endl;
    cout << "df: " << depthFactor<<endl;
}

}//namespace ORB_SLAM2



