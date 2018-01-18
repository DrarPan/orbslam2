//Camera configuration
#ifndef CAMERA_H
#define CAMERA_H

#include<opencv2/core/core.hpp>

namespace ORB_SLAM2 {
class Camera {
public:
    static bool load(cv::FileStorage fSettings);
    static bool load(std::string strSettingsFile);
    static void show();
    static std::string name;
    static float width;
    static float height;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    static float bf;          //stereo baseline multiplied by fx.
    static float b;           //stereo baseline in meters
    static cv::Mat K;	      //center and focus
    static cv::Mat DistCoef;  //distortion
    static float depthFactor; //convert mm to m
    static bool initialized;

};
} // namespace ORB_SLAM2

#endif
