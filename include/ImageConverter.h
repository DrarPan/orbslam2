#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct ColorDepthImagePair{
    cv::Mat colorimg;
    cv::Mat depthimg;
    ColorDepthImagePair(){}
    ColorDepthImagePair(cv::Mat c,cv::Mat d){
        colorimg=c.clone();
        depthimg=d.clone();
    }
};

class ImageConverter{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber colorimage_sub_;
    image_transport::Subscriber depthimage_sub_;
    bool color_captured_;
    bool depth_captured_;
    int maxcount_;
    cv::Mat colormat_;
    cv::Mat depthmat_;
public:
    ImageConverter(std::string colorimg_topic="/camera/rgb/image_raw",std::string depthimg_topic="/camera/depth_registered/image_raw") : it_(nh_),color_captured_(false),depth_captured_(false),maxcount_(50){
        colorimage_sub_ = it_.subscribe(colorimg_topic,1,&ImageConverter::colorimageCb, this);
        depthimage_sub_ = it_.subscribe(depthimg_topic,1,&ImageConverter::depthimageCb, this);
    }

    void depthimageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        depth_captured_=true;
        depthmat_=cv_ptr->image.clone();
    }

    void colorimageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        color_captured_=true;
        colormat_=cv_ptr->image.clone();
        cv::cvtColor(colormat_,colormat_,cv::COLOR_BGR2RGB);
    }

    cv::Mat captureColor(){
        color_captured_=false;
        int count=0;
        while(!color_captured_ && count++<maxcount_){
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        return colormat_.clone();
    }

    cv::Mat captureDepth(){
        depth_captured_=false;
        int count=0;
        while(!depth_captured_ && count++<maxcount_){
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        return depthmat_.clone();
    }

    ColorDepthImagePair capturePair(){
        color_captured_=false;
        depth_captured_=false;
        int count=0;
        while((!depth_captured_ || !color_captured_) && count++<maxcount_){
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        return ColorDepthImagePair(colormat_,depthmat_);
    }
};

