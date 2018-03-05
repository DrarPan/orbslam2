#include<iostream>
#include<AR/MotionModel.h>

using namespace std;

GravityDampingModel::GravityDampingModel(double g, double c, double f, double r):g_(g),c_(c),f_(f),rebound_time_(r),calculate_interval_(0.01){}


vector<Eigen::Vector3d> GravityDampingModel::calculateTrajectory(Eigen::Vector3d initp, Eigen::Vector3d initv, double duration, double interval){
    vector<Eigen::Vector3d> vPose;
    if(initp(2)<f_){
        cout<<"The initial pose is already beneath beneath the floor"<<endl;
        return vPose;
    }

    Eigen::Vector3d p=initp;
    Eigen::Vector3d v=initv;

    //vPose.push_back(p);
    //double next_recordtime=interval;
    double next_recordtime=0.0;
    double rebound_start_time=-1.0;

    for(double t=0;t<=duration+1e-10;t+=calculate_interval_){
        if(t>=next_recordtime){
            vPose.push_back(p);
            next_recordtime+=interval;
        }

        //We check if the object is rebounding firstly
        if(t-rebound_start_time<rebound_time_)
            continue;

        if(p(2)<f_){
            p(2)=f_;
            v(2)=-v(2)*c_;
            rebound_start_time=t;
        }else{
            p(0)=p(0)+v(0)*calculate_interval_;p(1)=p(1)+v(1)*calculate_interval_;
            p(2)=p(2)+v(2)*calculate_interval_+0.5*g_*calculate_interval_*calculate_interval_;
            v(2)+=calculate_interval_*g_;
        }
    }
    return vPose;
}

vector<Eigen::Vector3f> GravityDampingModel::calculateTrajectory(Eigen::Vector3f initp, Eigen::Vector3f initv, double duration, double interval){
    vector<Eigen::Vector3f> vPose;
    if(initp(2)<f_){
        cout<<"The initial pose is already beneath beneath the floor"<<endl;
        return vPose;
    }

    Eigen::Vector3f p=initp;
    Eigen::Vector3f v=initv;

    //vPose.push_back(p);
    //double next_recordtime=interval;
    double next_recordtime=0.0;
    double rebound_start_time=-1.0;

    for(double t=0;t<=duration+1e-10;t+=calculate_interval_){
        if(t>=next_recordtime){
            vPose.push_back(p);
            next_recordtime+=interval;
        }

        //We check if the object is rebounding firstly
        if(t-rebound_start_time<rebound_time_)
            continue;

        if(p(2)<f_){
            p(2)=f_;
            v(2)=-v(2)*c_;
            rebound_start_time=t;
        }else{
            p(0)=p(0)+v(0)*calculate_interval_;p(1)=p(1)+v(1)*calculate_interval_;
            p(2)=p(2)+v(2)*calculate_interval_+0.5*g_*calculate_interval_*calculate_interval_;
            v(2)+=calculate_interval_*g_;
        }
    }
    return vPose;
}

template <class T>
vector<T> GravityDampingModel::calculateTrajectory(T initp, T initv, double duration, double interval){
    vector<Eigen::Vector3d> vPose;
    if(initp(2)<f_){
        cout<<"The initial pose is already beneath beneath the floor"<<endl;
        return vPose;
    }

    T p=initp;
    T v=initv;

    //vPose.push_back(p);
    //double next_recordtime=interval;
    double next_recordtime=0.0;
    double rebound_start_time=-1.0;

    for(double t=0;t<=duration+1e-10;t+=calculate_interval_){
        if(t>=next_recordtime){
            vPose.push_back(p);
            next_recordtime+=interval;
        }

        //We check if the object is rebounding firstly
        if(t-rebound_start_time<rebound_time_)
            continue;

        if(p(2)<f_){
            p(2)=f_;
            v(2)=-v(2)*c_;
            rebound_start_time=t;
        }else{
            p(0)=p(0)+v(0)*calculate_interval_;p(1)=p(1)+v(1)*calculate_interval_;
            p(2)=p(2)+v(2)*calculate_interval_+0.5*g_*calculate_interval_*calculate_interval_;
            v(2)+=calculate_interval_*g_;
        }
    }
    return vPose;
}



