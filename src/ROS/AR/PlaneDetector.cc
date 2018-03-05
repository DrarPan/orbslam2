#include <iostream>
#include "AR/PlaneDetector.h"
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

using namespace std;

PlaneFunction::PlaneFunction():a(1),b(1),c(1),d(0){norm2=a*a+b*b+c*c;norm=sqrt(norm2);}
PlaneFunction::PlaneFunction(double _a, double _b, double _c, double _d):a(_a),b(_b),c(_c),d(_d){norm2=a*a+b*b+c*c;norm=sqrt(norm2);}

vector<cv::Point3f> PlaneFunction::generateFakePlaneData(double x_start, double x_end, double y_start, double y_end, double interval){
    vector<cv::Point3f> points;
    cv::Point3d point;
    cv::RNG rng(cv::getTickCount());
    for(double x=x_start;x<=x_end;x+=interval){
        for(double y=y_start;y<=y_end;y+=interval){
            double z=(-a*x-b*y-d)/c;
            point.x=x+rng.gaussian(0.1);
            point.y=y+rng.gaussian(0.1);
            point.z=z+rng.gaussian(0.1);
            points.push_back(point);
        }
    }
    return points;
}

ostream &operator<<(ostream &os, const PlaneFunction pf){
    os<<pf.a<<", "<<pf.b<<", "<<pf.c<<", "<<pf.d;
    return os;
}

Plane::Plane(){
    n=cv::Mat::zeros(3,1,CV_32F);
    o=cv::Mat::zeros(3,1,CV_32F);
    n.at<float>(2)=1;
    Tpc=cv::Mat::eye(4,4,CV_32F);
}

Plane::Plane(cv::Mat _n, cv::Mat _o){
    cv::RNG rng(cv::getTickCount());//cv::getTickCount()
    //float rang=rng.uniform((float)0.0,(float)M_PI)-M_PI/2;
    n=_n.clone();
    o=_o.clone();

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, 1.0f);
    cv::Mat v = up.cross(n);

    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);

    Tpc = cv::Mat::eye(4,4,CV_32F);
    cv::Mat R=ExpSO3(v*ang/sa);//*ExpSO3(up*rang);
    R.copyTo(Tpc.rowRange(0,3).colRange(0,3)); //ExpSO3(v*ang/sa)*ExpSO3(angle*up)
    o.copyTo(Tpc.col(3).rowRange(0,3));
}

PlaneFunction Plane::toPLaneFunction(){
    float a=n.at<float>(0);
    float b=n.at<float>(1);
    float c=n.at<float>(2);
    float d=-a*o.at<float>(0)-b*o.at<float>(1)-c*o.at<float>(2);
    return PlaneFunction(a,b,c,d);
}

PlaneDetector::PlaneDetector(){
    //normal_pub_=nh_.advertise<geometry_msgs::PoseStamped>("normal_vector",2,true);
}

Plane PlaneDetector::detectPlane(vector<cv::Point3f>& points, int iteration){
    int Noripoint=points.size();

    if(Noripoint<50){
        cout<<"Too few points to fit plane"<<endl;
        return Plane();
    }

    double bestMedianDist=100000;
    vector<float> bestvDist;

    double Npoint2fit=4;
    vector<cv::Point3f> fitting_points;
    double mindist=0.2;

    //RANSAC (randomly chose n point to fit a plane)
    cv::RNG rng(cv::getTickCount());;
    for(int k=0;k<iteration;k++){
        fitting_points.clear();
        while(fitting_points.size()<Npoint2fit){
            bool goodpoint=true;

            cv::Point3f fitting_point=points[rng.uniform(0,Noripoint)];
            for(int i=0;i<fitting_points.size();i++){
                cv::Point3f dist_vector=fitting_point-fitting_points[i];
                if(cv::norm(dist_vector)<mindist){
                    goodpoint=false;
                    break;
                }
            }
            if(goodpoint)
                fitting_points.push_back(fitting_point);
        }
        cv::Mat A(Npoint2fit,4,CV_32F);
        A.col(3) = cv::Mat::ones(Npoint2fit,1,CV_32F);
        for(int i=0;i<Npoint2fit;i++){
            A.row(i).col(0)=fitting_points[i].x;
            A.row(i).col(1)=fitting_points[i].y;
            A.row(i).col(2)=fitting_points[i].z;
        }
        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);//MSL

        float a=vt.at<float>(3,0);
        float b=vt.at<float>(3,1);
        float c=vt.at<float>(3,2);
        float d=vt.at<float>(3,3);

        float norm=1.0f/sqrt(a*a+b*b+c*c+d*d);

        vector<float> vDistances(Noripoint,0);
        for(int i=0;i<Noripoint;i++)
            vDistances[i]=fabs(points[i].x*a+points[i].y*b+points[i].z*c+d)*norm;

        vector<float> vSorted=vDistances;
        sort(vSorted.begin(),vSorted.end());

        float medianDist=vSorted[max((int)(0.2*Noripoint),20)];
        if(medianDist<bestMedianDist){
            bestMedianDist=medianDist;
            bestvDist=vDistances;
        }
    }

    //use points around the best RANSAC fitting plane to recomputer a better plane
    fitting_points.clear();
    for(int i=0;i<Noripoint;i++){
        if(bestvDist[i]<bestMedianDist*1.5)//Why 1.5(1.4)?
            fitting_points.push_back(points[i]);
    }

    Npoint2fit=fitting_points.size();
    cv::Mat A(Npoint2fit,4,CV_32F);

    cv::Point3f center(0,0,0);

    A.col(3) = cv::Mat::ones(Npoint2fit,1,CV_32F);
    for(int i=0;i<Npoint2fit;i++){
        A.row(i).col(0)=fitting_points[i].x;
        A.row(i).col(1)=fitting_points[i].y;
        A.row(i).col(2)=fitting_points[i].z;
        center+=fitting_points[i];
    }
    center.x/=Npoint2fit;
    center.y/=Npoint2fit;
    center.z/=Npoint2fit;

    cv::Mat o=(cv::Mat_<float>(3,1)<<center.x,center.y,center.z);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a=vt.at<float>(3,0);
    float b=vt.at<float>(3,1);
    float c=vt.at<float>(3,2);
    float d=vt.at<float>(3,3);

    float norm=1.0f/sqrt(a*a+b*b+c*c);

    if(d<0){
        a=-a;
        b=-b;
        c=-c;
    }

    cv::Mat n=(cv::Mat_<float>(3,1)<<a*norm,b*norm,c*norm);
    cout<<"normal:"<<endl<<n<<endl;
    cout<<"center:"<<endl<<center<<endl;

//    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, 1.0f);
//    cv::Mat v = up.cross(n);

//    const float sa = cv::norm(v);
//    const float ca = up.dot(n);
//    const float ang = atan2(sa,ca);

//    cv::Mat R=ExpSO3(n);
//    cout<<R<<endl;
//    cout<<"angle: "<<ang<<endl;
//    Eigen::AngleAxisf rotation(ang,Eigen::Vector3f(v.at<float>(0),v.at<float>(1),v.at<float>(2)));
//    Eigen::Quaternionf q(rotation);

//    geometry_msgs::PoseStamped pose;
//    pose.header.frame_id="camera_rgb_optical_frame";
//    pose.header.stamp=ros::Time::now();
//    pose.pose.position.x=center.x;
//    pose.pose.position.y=center.y;
//    pose.pose.position.z=center.z;
//    pose.pose.orientation.x=q.x();
//    pose.pose.orientation.y=q.y();
//    pose.pose.orientation.z=q.z();
//    pose.pose.orientation.w=q.w();


//    normal_pub_.publish(pose);
//    cout<<"pub"<<endl;

//    while(1){
//    broadcaster_.sendTransform(
//      tf::StampedTransform(
//        tf::Transform(tf::Quaternion(q.x(),q.y(),q.z(),q.w()),tf::Vector3(center.x,center.y,center.z)),
//        ros::Time::now(),"camera_rgb_optical_frame", "plane"));
//    ros::Rate(10).sleep();
//    }
    return Plane(n,o);
}
