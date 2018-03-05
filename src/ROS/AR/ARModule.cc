#include <iostream>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include "Camera.h"
#include "ImageConverter.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include <AR/MotionModel.h>
#include <AR/DataConverter.h>
#include <AR/PlaneDetector.h>
#include <chrono>

#include <omp.h>

#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc,char** argv){
    //Initialize camera
    ros::init(argc,argv,"test_ARModule");
    string path_to_settings="/home/hitrobot/hitrobot_ws/orbslam2/config/Asus_default.yaml";
    ORB_SLAM2::Camera::load(path_to_settings);
    cv::Mat sceneimg=cv::imread("/home/hitrobot/catkin_ws/src/orbslam2/data/AR/scene.jpg");
    cv::Mat objimg=cv::imread("/home/hitrobot/catkin_ws/src/orbslam2/data/AR/rect.png");

    ImageConverter ic;
    ColorDepthImagePair imagepair=ic.capturePair();
    sceneimg=imagepair.colorimg;
    cv::Mat depthimg=imagepair.depthimg;
    PlaneDetector pd;
    vector<cv::Point3f> vPoints=depthImage2Point3d(depthimg,8);
    Plane plane=pd.detectPlane(vPoints);
    cout<<plane.Tpc<<endl;


    cv::imshow("sceneimg",sceneimg);
    cv::waitKey(1000);

    ORB_SLAM2::Camera::load(path_to_settings);
    vector<ColorPoint> vcp=image2ColorPointCloud(objimg,0.5,cv::Point3f(0.0,0.0,0.0));

    cv::Mat T=(cv::Mat_<float>(4,4)<<1,0,0,0,
                                     0,0,-1,0,
                                     0,1,0,0,
                                     0,0,0,1);

    vector<ColorPoint> Tvcp=vcp;//T*vcp;

//    vcp=TransfromColorPointCloud(cvMat44toEigenMatrix4(plane.Tpc.inv())*,vcp);
    GravityDampingModel motion_model;
    vector<Eigen::Vector3f> traj=motion_model.calculateTrajectory(Eigen::Vector3f(0.0,0.0,3),Eigen::Vector3f(0.05,0.05,0.0),20.0,0.1);

    double u,v;
    int ru,rv;

    cv::Size imgsize=sceneimg.size();

    Eigen::Matrix4f Tm;
    cv::Mat canvas;
    double selfrotation_w=0.1;
    for(int t=0;t<traj.size();t++){
        Tm<<cos(t*selfrotation_w),-sin(t*selfrotation_w),0,traj[t](0),
            sin(t*selfrotation_w),cos(t*selfrotation_w),0,traj[t](1),
            0,0,1,traj[t](2),
            0,0,0,1;

        Tvcp=TransfromColorPointCloud(cvMat44toEigenMatrix4(plane.Tpc.inv())*Tm,vcp);
        canvas=sceneimg.clone();
        for(int j=0;j<vcp.size();j++){
            ColorPoint cp=Tvcp[j];
            XYZ2uv(cp.point.x,cp.point.y,cp.point.z,u,v);
            ru=round(u);rv=round(v);
            if(inImage(imgsize,ru,rv)){
                cv::Vec3b color;
                color[0]=cp.color[0];
                color[1]=cp.color[1];
                color[2]=cp.color[2];
                canvas.at<cv::Vec3b>(imgsize.height-rv,ru)=color;
            }
        }
        cv::imshow("sceneimg",canvas);
        cv::waitKey(10);
    }

}

//    ros::init(argc,argv,"MotionModel");
//    string path_to_settings="/home/hitrobot/hitrobot_ws/orbslam2/config/Asus_default.yaml";
//    ORB_SLAM2::Camera::load(path_to_settings);

//    ImageConverter a;

//    cv::Mat depthimg=a.capture();
//    vector<cv::Point3f> vPoints=depthImage2Point3d(depthimg,16);

//    PlaneFunction plane0(1,2,3,4);
//    vector<cv::Point3f> vPoints0=plane0.generateFakePlaneData();

//    PlaneDetector pd;
//    Plane plane=pd.detectPlane(vPoints0);

//    PlaneFunction pf=plane.toPLaneFunction();
//    cout<<pf<<endl;

//    double fakedist=5;
//    double fakedist_a=fakedist*pf.a/pf.norm;
//    double fakedist_b=fakedist*pf.b/pf.norm;
//    double fakedist_c=fakedist*pf.c/pf.norm;

//    cv::Mat fakepoint=(cv::Mat_<float>(4,1)<<plane.o.at<float>(0)+fakedist_a,plane.o.at<float>(1)+fakedist_b,plane.o.at<float>(2)+fakedist_c,1);

//    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, 1.0f);
//    cv::Mat v = up.cross(plane.n);

//    const float sa = cv::norm(v);
//    const float ca = up.dot(plane.n);
//    const float ang = atan2(sa,ca);

//    cv::Mat Tpc = cv::Mat::eye(4,4,CV_32F);

//    ExpSO3(v*ang/sa).copyTo(Tpc.rowRange(0,3).colRange(0,3)); //ExpSO3(v*ang/sa)*ExpSO3(angle*up) //currently we don't need rotation of up
//    plane.o.copyTo(Tpc.col(3).rowRange(0,3));
//    cout<<Tpc<<endl;
//    cout<<Tpc.inv()*fakepoint<<endl;}

//chrono::steady_clock::time_point t1=chrono::steady_clock::now();
//chrono::steady_clock::time_point t2=chrono::steady_clock::now();
//chrono::duration<double> time_used=chrono::duration_cast<chrono::duration<double> >(t2-t1);
//cout<<"solve time cost ="<<time_used.count()<<endl;


