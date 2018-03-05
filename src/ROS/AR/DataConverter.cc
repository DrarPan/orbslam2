#include <iostream>

#include "AR/DataConverter.h"

using namespace std;

cv::Mat ExpSO3(const float &x, const float &y, const float &z){
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<1e-4)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(cv::Mat v){
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

std::vector<cv::Point3f> generateFakePlaneData(double a,double b,double c,double d){
    vector<cv::Point3f> points;
    cv::Point3d point;
    cv::RNG rng;
    for(double x=-10;x<=10;x+=1){
        for(double y=-10;y<=0;y+=1){
            double z=(-a*x-b*y-d)/c;
            point.x=x+rng.gaussian(0.1);
            point.y=y+rng.gaussian(0.1);
            point.z=z+rng.gaussian(0.1);
            points.push_back(point);
        }
    }
    return points;
}

std::vector<cv::Point3f> depthImage2Point3d(cv::Mat depthimg,int skip){//Generally we dont need too dense 3d point data
    if(!ORB_SLAM2::Camera::initialized){
        cout<<"Cannnot generate point cloud before camera initialized"<<endl;
        return vector<cv::Point3f>();
    }

    vector<cv::Point3f> points;
    cv::Point3f point;

    for(int v=skip;v<depthimg.rows-skip;v+=skip){
        for(int u=skip;u<depthimg.cols-skip;u+=skip){
            if(!isnan(depthimg.at<float>(v,u))){
                point.z=depthimg.at<float>(v,u);
                point.x=(u-ORB_SLAM2::Camera::cx)/ORB_SLAM2::Camera::fx*point.z;
                point.y=(v-ORB_SLAM2::Camera::cy)/ORB_SLAM2::Camera::fy*point.z;
                points.push_back(point);
            }
        }
    }
    return points;
}

std::vector<cv::Point3f> depthImage2Point3d(cv::Mat depthimg,double fx,double fy,double cx,double cy,int skip){//Generally we dont need too dense 3d point data
    vector<cv::Point3f> points;
    cv::Point3f point;

    for(int v=skip;v<depthimg.rows-skip;v+=skip){
        for(int u=skip;u<depthimg.cols-skip;u+=skip){
            if(!isnan(depthimg.at<float>(v,u))){
                point.z=depthimg.at<float>(v,u);
                point.x=(u-cx)/fx*point.z;
                point.y=(v-cy)/fy*point.z;
                //cout<<point.x<<","<<point.y<<","<<point.z<<endl;
                points.push_back(point);
            }
        }
    }
    return points;
}

void XYZ2uv(double X,double Y,double Z,double &u,double &v){
    u=X/Z*(ORB_SLAM2::Camera::fx)+ORB_SLAM2::Camera::cx;
    v=Y/Z*(ORB_SLAM2::Camera::fy)+ORB_SLAM2::Camera::cy;
}

void uv2XYZ(double u, double v, double d, double &X, double &Y, double &Z){
    Z=d;
    X=(u-ORB_SLAM2::Camera::cx)/ORB_SLAM2::Camera::fx*Z;
    Y=(v-ORB_SLAM2::Camera::cy)/ORB_SLAM2::Camera::fy*Z;
}

bool inImage(cv::Size s,double& u,double& v){
    return (u>=0 && u<=s.width-1 && v>=0 && v<s.height-1);
}

bool inImage(cv::Size s,int& u,int& v){
    return (u>=0 && u<=s.width-1 && v>=0 && v<s.height-1);
}

ColorPoint TransfromColorPoint(const cv::Mat& T,ColorPoint p){
    cv::Mat P=(cv::Mat_<float>(4,1)<<p.point.x,p.point.y,p.point.z,1);
    cv::Mat TP=T*P;
    p.point.x=TP.at<float>(0);
    p.point.y=TP.at<float>(1);
    p.point.z=TP.at<float>(2);
    return p;
}


vector<ColorPoint> TransfromColorPointCloud(cv::Mat& T,const vector<ColorPoint>& vp){
//    it is strange that the computation of T(4,4) is faster than T(3,4)
//    if(T.rows==4){
//        T=T.rowRange(0,3);
//    }
    Eigen::Matrix4f eigenT;
    Eigen::Vector4f p;
    Eigen::Vector4f Tp;

    cv::Mat TP=cv::Mat(4,1,CV_32F);
    vector<ColorPoint> vTp=vp;
    #pragma omp parallel for
    //for(ColorPoint p:vp){ //openmp doesn't support this syntax
    for(unsigned int i=0;i<vp.size();i++){
        cv::Mat P=(cv::Mat_<float>(4,1)<<vp[i].point.x,vp[i].point.y,vp[i].point.z,1);
        cv::Mat TP=T*P;
        vTp[i].point.x=TP.at<float>(0);
        vTp[i].point.y=TP.at<float>(1);
        vTp[i].point.z=TP.at<float>(2);
        //vTp.push_back(p); //Error when paralleling
    }
    return vTp;
}

Eigen::Matrix4f cvMat44toEigenMatrix4(const cv::Mat& T){
    Eigen::Matrix4f m;
    if(T.rows==4 && T.cols==4){
        m<<T.at<float>(0,0),T.at<float>(0,1),T.at<float>(0,2),T.at<float>(0,3),
           T.at<float>(1,0),T.at<float>(1,1),T.at<float>(1,2),T.at<float>(1,3),
           T.at<float>(2,0),T.at<float>(2,1),T.at<float>(2,2),T.at<float>(2,3),
           T.at<float>(3,0),T.at<float>(3,1),T.at<float>(3,2),T.at<float>(3,3);
    }
    return m;
}

vector<ColorPoint> TransfromColorPointCloud(Eigen::Matrix4f T,const vector<ColorPoint>& vp){//much faster than cv::Mat multiplication
    Eigen::Vector4f p;
    Eigen::Vector4f Tp;

    vector<ColorPoint> vTp=vp;
    //#pragma omp parallel for
    for(unsigned int i=0;i<vp.size();i++){
        p<<vp[i].point.x,vp[i].point.y,vp[i].point.z,1;
        Tp=T*p;
        vTp[i].point.x=Tp(0);
        vTp[i].point.y=Tp(1);
        vTp[i].point.z=Tp(2);
    }
    return vTp;
}

vector<ColorPoint> image2ColorPointCloud(cv::Mat img, double pixel_scale, cv::Point3f base, bool laid, bool remove_background, bool inversex){//pixel_scale means mm/pixel, the base means the bottom center
    int channels=img.channels();
    int height=img.rows;int width=img.cols;
    vector<ColorPoint> colorpoints;

    cv::Point3f start_points;
    if(!laid){//image stands at base
        start_points.x=base.x-pixel_scale*0.001*width/2;
        start_points.y=base.y;
        start_points.z=base.z;

        ColorPoint p;
        for(int i=0;i<img.cols;i++){
            for(int j=0;j<img.rows;j++){
                if(channels==1){
                    p.color=cv::Scalar(img.at<uchar>(img.rows-1-j,inversex?img.cols-1-i:i));
                    if(p.color==cv::Scalar(255) && remove_background)
                        continue;
                }else if(channels==3){
                    cv::Vec3b color=img.at<cv::Vec3b>(img.rows-1-j,inversex?img.cols-1-i:i);
                    if(color[0]==255 && color[1]==255 && color[2]==255 && remove_background)
                        continue;
                    else{
                        p.color=cv::Scalar(color[0],color[1],color[2]);
                    }
                }else{
                    continue;
                }
                p.point.x=start_points.x+i*pixel_scale*0.001;
                p.point.y=start_points.y+j*pixel_scale*0.001;
                p.point.z=start_points.z;
                colorpoints.push_back(p);
            }
        }
    }else{
        start_points.x=base.x-pixel_scale*0.001*width/2;
        start_points.y=base.y-pixel_scale*0.001*height/2;
        start_points.z=base.z;

        ColorPoint p;
        for(int i=0;i<img.cols;i++){
            for(int j=0;j<img.rows;j++){
                if(channels==1){
                    p.color=cv::Scalar(img.at<uchar>(img.rows-1-j,inversex?img.cols-1-i:i));
                    if(p.color==cv::Scalar(255) && remove_background)
                        continue;
                }else if(channels==3){
                    cv::Vec3b color=img.at<cv::Vec3b>(img.rows-1-j,inversex?img.cols-1-i:i);
                    if(color[0]==255 && color[1]==255 && color[2]==255 && remove_background)
                        continue;
                    else{
                        p.color=cv::Scalar(color[0],color[1],color[2]);
                    }
                }else{
                    continue;
                }
                p.point.x=start_points.x+i*pixel_scale*0.001;
                p.point.y=start_points.y+j*pixel_scale*0.001;
                p.point.z=start_points.z;
                colorpoints.push_back(p);
            }
        }
    }
    return colorpoints;
}
