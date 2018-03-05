#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "Camera.h"

#include "AR/DataConverter.h"
#include "AR/MotionModel.h"
#include "AR/PlaneDetector.h"
#include <opencv/highgui.h>

using namespace std;

class ImageGrabber{
public:
    ORB_SLAM2::System* mpSLAM;
    vector<vector<ColorPoint> > vvp_;
    vector<ColorPoint> vp_;
    vector<ColorPoint> Tvp_;
    vector<Eigen::Vector3f> traj_;
    int t_;
    int p_;
    bool AR_initialized_;
    Eigen::Matrix4f Tm_;
    cv::Mat Tpc_;//while a plane has been detected, we don't change its postion
    cv::Mat Tcp_;//we need Tcp,
    cv::Mat curr_Tcw_;//could be empty;
    cv::Mat ref_Tcw_;//last non_empty transform
    cv::Mat ARinitial_Tcw_;//As initialTcw_ is always identity, we don't need to set it specificallly
    cv::Size imgsize_;
    bool launchAR_;
    bool recovered_;//Loading previous map
    int recovery_thresh_;
    int Ncontineous_nonempty_;
    cv::Mat canvas_;
    ros::Time init_time_;
    cv::VideoWriter writer_;
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,bool launchAR=false):mpSLAM(pSLAM),t_(0),p_(-1),AR_initialized_(false),launchAR_(launchAR),recovered_(false),Ncontineous_nonempty_(0),recovery_thresh_(10),
        writer_("/home/hitrobot/blessVideo.avi",CV_FOURCC('M','J','P','G'),15.0,cv::Size(640,480)){init_time_=ros::Time::now();}
    ~ImageGrabber(){writer_.release();}
    void setColorPointCloud(vector<ColorPoint> vp);
    void setColorPointCloud(vector<vector<ColorPoint> > vvp);
    void setTrajectory(vector<Eigen::Vector3f> traj);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

};

int main(int argc, char **argv){
    ros::init(argc, argv, "RGBD");
    ros::NodeHandle pnh("~");

    string path_to_vocabulary,path_to_settings,path_to_map;

    if(argc == 3){
        path_to_vocabulary=argv[1];
        path_to_settings=argv[2];
    }else{
        pnh.param<string>("path_to_vocabulary",path_to_vocabulary,ros::package::getPath("orbslam2")+"/vocabulary/ORBvoc.bin");
        pnh.param<string>("path_to_settings",path_to_settings,ros::package::getPath("orbslam2")+"/config/Asus_default.yaml");
        pnh.param<string>("path_to_map",path_to_map,ros::package::getPath("orbslam2")+"/map/map.bin");
    }

    ORB_SLAM2::ORBVocabulary voc;
    voc.loadFromBinaryFile(path_to_vocabulary);

    ORB_SLAM2::Camera::load(path_to_settings);
    ORB_SLAM2::Map prev_map;
    prev_map.load(path_to_map,voc);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    ORB_SLAM2::System SLAM(path_to_settings,ORB_SLAM2::System::RGBD,&voc,&prev_map,true);

    ImageGrabber igb(&SLAM,true);

    cv::Mat objimg=cv::imread("/home/hitrobot/catkin_ws/src/orbslam2/data/AR/fu.png");
    vector<ColorPoint> vcp=image2ColorPointCloud(objimg,0.5,cv::Point3f(0.0,0.0,0.0),true,true,true);
    igb.setColorPointCloud(vcp);

    objimg=cv::imread("/home/hitrobot/catkin_ws/src/orbslam2/data/AR/jiayou.png");
    vcp=image2ColorPointCloud(objimg,0.75,cv::Point3f(0.0,0.0,0.0),true,true,true);
    igb.setColorPointCloud(vcp);

    GravityDampingModel motion_model;
    vector<Eigen::Vector3f> traj=motion_model.calculateTrajectory(Eigen::Vector3f(0.0,0.0,4),Eigen::Vector3f(0.0,0.0,0.0),30.0,0.1);
    igb.setTrajectory(traj);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();
    // Stop all threads

    // Save camera trajectory
    SLAM.Shutdown();
    // SLAM.SaveKeyFrameTrajectoryTUM(ros::package::getPath("orbslam2")+"/map/KeyFrameTrajectory.txt");
    // SLAM.SaveMap(ros::package::getPath("orbslam2")+"/map/map.bin");

    sleep(1);
    ros::shutdown();
    return 0;
}

void ImageGrabber::setColorPointCloud(vector<ColorPoint> vp){
    vvp_.push_back(vp);
}

void ImageGrabber::setColorPointCloud(vector<vector<ColorPoint> > vvp){
    vvp_=vvp;
}

void ImageGrabber::setTrajectory(vector<Eigen::Vector3f> traj){
    traj_=traj;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    curr_Tcw_=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if(!recovered_){
        if(!curr_Tcw_.empty()){
            Ncontineous_nonempty_++;
        }else{
            Ncontineous_nonempty_=0;
        }
        if(Ncontineous_nonempty_>recovery_thresh_){
            cout<<"Pose recovered from saved map"<<endl;
            recovered_=true;
        }
    }

    if(launchAR_ && recovered_){ //(ros::Time::now()-init_time_).toSec()>10
        if(!curr_Tcw_.empty() && !AR_initialized_){
            PlaneDetector pd;
            vector<cv::Point3f> vPoints=depthImage2Point3d(cv_ptrD->image,8);
            Plane plane=pd.detectPlane(vPoints);
            Tpc_=plane.Tpc;
            Tcp_=Tpc_.inv();
            imgsize_=cv_ptrRGB->image.size();
            ARinitial_Tcw_=curr_Tcw_;
            AR_initialized_=true;
            cv::namedWindow("AR");
            cv::waitKey(20);
        }

        //attach the AR pattern to origin image
        if(AR_initialized_ && t_<traj_.size() && !curr_Tcw_.empty() ){
            if(p_<int(double(t_)/traj_.size()*vvp_.size())){
                p_=int(double(t_)/traj_.size()*vvp_.size());
                vp_=vvp_[p_];
            }
            canvas_=cv_ptrRGB->image.clone();
            cv::cvtColor(canvas_,canvas_,cv::COLOR_BGR2RGB);
            double selfrotation_w=0.05;
            double u,v;
            int ru,rv;
            Tm_<<cos(t_*selfrotation_w),-sin(t_*selfrotation_w),0,traj_[t_](0),
                 sin(t_*selfrotation_w),cos(t_*selfrotation_w),0,traj_[t_](1),
                 0,0,1,traj_[t_](2),
                 0,0,0,1;

            Tvp_=TransfromColorPointCloud(cvMat44toEigenMatrix4(curr_Tcw_*ARinitial_Tcw_.inv()*Tcp_)*Tm_,vp_);
            for(unsigned int j=0;j<vp_.size();j++){
                ColorPoint cp=Tvp_[j];
                XYZ2uv(cp.point.x,cp.point.y,cp.point.z,u,v);
                ru=round(u);rv=round(v);
                if(inImage(imgsize_,ru,rv)){
                    cv::Vec3b color;
                    color[0]=cp.color[0];
                    color[1]=cp.color[1];
                    color[2]=cp.color[2];
                    canvas_.at<cv::Vec3b>(imgsize_.height-rv,ru)=color;
                }
            }

            writer_<<canvas_;
            t_++;
        }
        cv::imshow("AR",canvas_);
        //cv::waitKey(1);
    }

}
