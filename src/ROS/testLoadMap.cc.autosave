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
#include "Frame.h"
#include "Camera.h"
#include "Converter.h"

using namespace std;

ORB_SLAM2::KeyFrame* readKeyFrame(ifstream &f,map<unsigned long int, ORB_SLAM2::Map::ConnectInformation>& map_connection);

string mapname="/home/hitrobot/catkin_ws/src/orbslam2/map/map.bin";

int main(){
    ORB_SLAM2::Map orbmap;
    ifstream f;
    f.open(mapname,ios_base::in|ios::binary);
    unsigned long int NMapPoint;
    f.read((char*)&NMapPoint,sizeof(NMapPoint));
    cout<<"Mappoint: "<<NMapPoint<<endl;
    for(unsigned int i=0;i<NMapPoint;i++){
        ORB_SLAM2::MapPoint* mp=orbmap.readMapPoint(f);
    }
    unsigned long int NKeyFrame;
    f.read((char*)&NKeyFrame,sizeof(NKeyFrame));
    cout<<"NKeyFrame: "<<NKeyFrame<<endl;

    map<unsigned long int, ORB_SLAM2::Map::ConnectInformation> map_connection;

    for(unsigned int i=0;i<NKeyFrame;i++){
        readKeyFrame(f,map_connection);
    }
    f.close();
}


ORB_SLAM2::KeyFrame* readKeyFrame(ifstream &f,map<unsigned long int, ORB_SLAM2::Map::ConnectInformation>& map_connection){
    ORB_SLAM2::Frame fr;
    //fr.mpORBvocabulary=&voc;
    f.read((char*)&fr.mnId, sizeof(fr.mnId));
    f.read((char*)&fr.mTimeStamp, sizeof(fr.mTimeStamp));
    cout<<"ID: "<<fr.mnId<<", Time: "<<fr.mTimeStamp<<endl;
    cv::Mat T=cv::Mat::eye(4,4,CV_32F); // pose
    f.read((char*)&T.at<float>(0, 3), sizeof(float));
    f.read((char*)&T.at<float>(1, 3), sizeof(float));
    f.read((char*)&T.at<float>(2, 3), sizeof(float));
    cout<<T.at<float>(0, 3)<<", "<<T.at<float>(1, 3)<<", "<<T.at<float>(2, 3)<<endl;
    vector<float> Q(4); // orientation
    f.read((char*)&Q[0], sizeof(float));
    f.read((char*)&Q[1], sizeof(float));
    f.read((char*)&Q[2], sizeof(float));
    f.read((char*)&Q[3], sizeof(float));
    cout<<Q[0]<<", "<<Q[1]<<", "<<Q[2]<<", "<<Q[3]<<endl;
    cv::Mat rotMat=ORB_SLAM2::Converter::toRotationMat(Q);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            T.at<float>(i,j)=rotMat.at<float>(i,j);
    fr.SetPose(T);
    f.read((char*)&fr.N,sizeof(fr.N));
    cout<<"fr.n:"<<fr.N<<endl;
    fr.mvKeys.reserve(fr.N);
    fr.mDescriptors.create(fr.N,32,CV_8UC1);
    fr.mvpMapPoints=vector<ORB_SLAM2::MapPoint*>(fr.N,static_cast<ORB_SLAM2::MapPoint*>(NULL));
    fr.mvuRight.resize(fr.N);
    fr.mvDepth.resize(fr.N);
    for(int i=0;i<fr.N;i++){
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x,sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y,sizeof(kp.pt.y));
        f.read((char*)&kp.size,sizeof(kp.size));
        f.read((char*)&kp.angle,sizeof(kp.angle));
        f.read((char*)&kp.response,sizeof(kp.response));
        f.read((char*)&kp.octave,sizeof(kp.octave));
        fr.mvKeys.push_back(kp);
        for(int j=0;j<32;j++)
            f.read((char*)&fr.mDescriptors.at<uchar>(i,j),sizeof(char));
        f.read((char*)&fr.mvuRight[i],sizeof(float));
        f.read((char*)&fr.mvDepth[i],sizeof(float));
        unsigned long int mpidx;
        f.read((char*)&mpidx,sizeof(mpidx));
        cout<<mpidx<<endl;
        if (mpidx == ULONG_MAX)
            fr.mvpMapPoints[i] = NULL;
        else
            fr.mvpMapPoints[i] = 0;//amp[mpidx];
    }

    ORB_SLAM2::Map::ConnectInformation conn_info;
    unsigned long int Nconnection;
    f.read((char*)&conn_info.parent_id,sizeof(conn_info.parent_id));
    f.read((char*)&Nconnection,sizeof(Nconnection));
    conn_info.connect_id.resize(Nconnection);
    conn_info.weight.resize(Nconnection);
    for(unsigned long int i=0;i<Nconnection;i++){//connection_num times
        f.read((char*)&conn_info.connect_id[i],sizeof(conn_info.connect_id[i]));
        f.read((char*)&conn_info.weight[i],sizeof(conn_info.weight[i]));
    }
    map_connection[fr.mnId]=conn_info;
}
