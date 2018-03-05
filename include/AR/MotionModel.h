#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <Eigen/Core>
#include <Eigen/Dense>

class GravityDampingModel{
private:
    double g_;
    double c_;//damping coefficient
    double f_;//the height of floor
    double rebound_time_;
    double calculate_interval_;//for calculating rebound more accurately
public:
    GravityDampingModel(double g=-9.8,double c=0.9,double f=0.0,double r=0.1);
    std::vector<Eigen::Vector3d> calculateTrajectory(Eigen::Vector3d initp=Eigen::Vector3d(0.0,0.0,1.0),Eigen::Vector3d initv=Eigen::Vector3d(0.0,0.0,0.0),double duration=5.0,double interval=0.1);
    std::vector<Eigen::Vector3f> calculateTrajectory(Eigen::Vector3f initp=Eigen::Vector3f(0.0,0.0,1.0),Eigen::Vector3f initv=Eigen::Vector3f(0.0,0.0,0.0),double duration=5.0,double interval=0.1);
    template<class T> std::vector<T> calculateTrajectory(T initp=T(0.0,0.0,1.0), T initv=T(0.0,0.0,1.0), double duration=5.0, double interval=0.1);

};

#endif
