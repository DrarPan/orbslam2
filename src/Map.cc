/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "Converter.h"
#include "ORBextractor.h"
#include "Camera.h"
#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    //cout<<"Add mnId: "<<pKF->mnId<<endl;

    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<KeyFrame*> Map::GetAllGoodKeyFrames(){
    unique_lock<mutex> lock(mMutexMap);
    vector<KeyFrame*> vpgoodKF;
    for(set<KeyFrame*>::iterator it=mspKeyFrames.begin();it!=mspKeyFrames.end();it++){
        if(!(*it)->isBad())
            vpgoodKF.push_back(*it);
    }
    return vpgoodKF;
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::writeKeyFrame(ofstream &f, KeyFrame *kf,map<MapPoint*,unsigned long int>& mp_idx,bool remove_bad_frame){
    //KeyFrame
    f.write((char*)&kf->mnId,sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp,sizeof(kf->mTimeStamp));

    cv::Mat T = kf->GetPose();//Transformation from
    f.write((char*)&T.at<float>(0,3),sizeof(float));
    f.write((char*)&T.at<float>(1,3),sizeof(float));
    f.write((char*)&T.at<float>(2,3),sizeof(float));
    vector<float> Q=Converter::toQuaternion(T.rowRange(0,3).colRange(0,3));
    f.write((char*)&Q[0],sizeof(float));
    f.write((char*)&Q[1],sizeof(float));
    f.write((char*)&Q[2],sizeof(float));
    f.write((char*)&Q[3],sizeof(float));
    //KeyPoint
    f.write((char*)&kf->N,sizeof(kf->N));
    for(int i=0;i<kf->N;i++){
        cv::KeyPoint kp=kf->mvKeys[i];
        f.write((char*)&kp.pt.x,sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y,sizeof(kp.pt.x));
        f.write((char*)&kp.size,sizeof(kp.size));
        f.write((char*)&kp.angle,sizeof(kp.angle));
        f.write((char*)&kp.response,sizeof(kp.response));
        f.write((char*)&kp.octave,sizeof(kp.octave));
        for (int j=0; j<32; j++)
            f.write((char*)&kf->mDescriptors.at<char>(i,j),sizeof(char));

        f.write((char*)&kf->mvuRight[i],sizeof(float));
        f.write((char*)&kf->mvDepth[i],sizeof(float));
        unsigned long int idx;
        MapPoint* mp=kf->GetMapPoint(i);
        if(mp==NULL)
            idx=ULONG_MAX;
        else
            idx=mp_idx[mp];
        f.write((char*)&idx,sizeof(idx));
    }

    unsigned long int parent_id=ULONG_MAX;
    KeyFrame* parent=kf->GetParent();
    if(parent!=NULL)
        parent_id = parent->mnId;
    f.write((char*)&parent_id,sizeof(parent_id));

    if(!remove_bad_frame){
        unsigned long int Nconnection = kf->GetConnectedKeyFrames().size();
        f.write((char*)&Nconnection,sizeof(Nconnection));
        for(KeyFrame* ckf:kf->GetConnectedKeyFrames()){//connection_num times
            long unsigned int id=ckf->mnId;
            int weight=kf->GetWeight(ckf);
            f.write((char*)&id, sizeof(id));
            f.write((char*)&weight, sizeof(weight));
        }
    }else{
        unsigned long int Nconnection = 0;
        vector<pair<unsigned long int,int> > good_connection;
        for(KeyFrame* ckf:kf->GetConnectedKeyFrames()){
            if(ckf && mspKeyFrames.count(ckf)){//sometimes keyframes of connection are not in mspKeyFrames
                good_connection.push_back(make_pair(ckf->mnId,kf->GetWeight(ckf)));
                Nconnection++;
            }
        }
        f.write((char*)&Nconnection,sizeof(Nconnection));
        for(pair<unsigned long int,int> p:good_connection){
            f.write((char*)(&p.first),sizeof(unsigned long int));
            f.write((char*)(&p.second),sizeof(int));
        }
    }
}

void Map::writeMapPoint(ofstream &f, MapPoint *mp){
    f.write((char*)&mp->mnId,sizeof(mp->mnId));
    cv::Mat pos = mp->GetWorldPos();
    f.write((char*)&pos.at<float>(0),sizeof(float));
    f.write((char*)&pos.at<float>(1),sizeof(float));
    f.write((char*)&pos.at<float>(2),sizeof(float));
}

KeyFrame *Map::readKeyFrame(ifstream &f, ORBVocabulary &voc, std::vector<MapPoint*>& amp, ORBextractor *orb_ext, map<unsigned long int,ConnectInformation>& map_connection){
    Frame fr;
    fr.mpORBvocabulary=&voc;
    f.read((char*)&fr.mnId, sizeof(fr.mnId));
    f.read((char*)&fr.mTimeStamp, sizeof(fr.mTimeStamp));
    cv::Mat T=cv::Mat::eye(4,4,CV_32F); // pose
    f.read((char*)&T.at<float>(0, 3), sizeof(float));
    f.read((char*)&T.at<float>(1, 3), sizeof(float));
    f.read((char*)&T.at<float>(2, 3), sizeof(float));
    vector<float> Q(4); // orientation
    f.read((char*)&Q[0], sizeof(float));
    f.read((char*)&Q[1], sizeof(float));
    f.read((char*)&Q[2], sizeof(float));
    f.read((char*)&Q[3], sizeof(float));
    cv::Mat rotMat=Converter::toRotationMat(Q);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            T.at<float>(i,j)=rotMat.at<float>(i,j);
    fr.SetPose(T);
    f.read((char*)&fr.N,sizeof(fr.N));
    fr.mvKeys.reserve(fr.N);
    fr.mDescriptors.create(fr.N,32,CV_8UC1);
    fr.mvpMapPoints=vector<MapPoint*>(fr.N,static_cast<MapPoint*>(NULL));
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
        f.read((char*)&mpidx,   sizeof(mpidx));
        if (mpidx == ULONG_MAX)
            fr.mvpMapPoints[i] = NULL;
        else
            fr.mvpMapPoints[i] = amp[mpidx];
    }

    fr.mpORBextractorLeft=orb_ext;
    fr.InitializeScaleLevels();
    fr.UndistortKeyPoints();
    fr.AssignFeaturesToGrid();
    fr.ComputeBoW();

    KeyFrame* kf=new KeyFrame(fr,this,NULL);
    kf->mnId=fr.mnId;//Important: for spanning covisibility tree
    for(int i=0;i<fr.N;i++){
        if(fr.mvpMapPoints[i]!=NULL){
            fr.mvpMapPoints[i]->AddObservation(kf,i);
            if (fr.mvpMapPoints[i]->GetReferenceKeyFrame()==NULL)
                fr.mvpMapPoints[i]->SetReferenceKeyFrame(kf);
        }
    }

    //for spanning covisibility tree
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
    map_connection[kf->mnId]=conn_info;
    return kf;
}

MapPoint *Map::readMapPoint(ifstream &f){
    long unsigned int id;
    f.read((char*)&id,sizeof(id));
    cv::Mat wp(3,1,CV_32F);
    f.read((char*)&wp.at<float>(0),sizeof(float));
    f.read((char*)&wp.at<float>(1),sizeof(float));
    f.read((char*)&wp.at<float>(2),sizeof(float));
    MapPoint* mp = new MapPoint(wp, this);
    mp->mnId = id;
    return mp;
}

bool Map::save(const string &filename){
    cout << "Saving map to " << filename << endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);

    unsigned long int NMappoints=mspMapPoints.size();
    f.write((char*)&NMappoints,sizeof(NMappoints));
    for(MapPoint* mp:mspMapPoints){
        writeMapPoint(f,mp);
    }

    cout<<"Wrote "<<mspMapPoints.size()<<" mappoints"<<endl;
    unsigned long int i = 0;

    map<MapPoint*,unsigned long int> mp_idx;
    for(MapPoint* mp: mspMapPoints)
        mp_idx[mp] = i++;

    unsigned long int NKeyframes = mspKeyFrames.size();
    f.write((char*)&NKeyframes, sizeof(NKeyframes));
    for(KeyFrame* kf: mspKeyFrames)
        writeKeyFrame(f, kf, mp_idx);
    cout<<"Wrote "<<mspKeyFrames.size()<<" keyframes"<<endl;

    f.close();
    return true;
}

bool Map::load(const string &filename,ORBVocabulary &voc){
    if(!Camera::initialized){
        cout<<"Should initialize camera parameter before loading map"<<endl;
        return false;
    }

    ORBextractor orb_ext = ORBextractor(ORBextractor::nFeaturesDefault, ORBextractor::scaleFactorDefault, ORBextractor::nLevelsDefault, ORBextractor::fIniThFASTDefault, ORBextractor::fMinThFASTDefault);
    ifstream f;
    cout<<"loading map from "<<filename<<endl;
    f.open(filename.c_str(), ios_base::in|ios::binary);
    if(!f.is_open())
        return false;
    unsigned long int Nmappoint,maxId=0;

    f.read((char*)&Nmappoint,sizeof(Nmappoint));
    cout<<"reading "<<Nmappoint<<" mappoints"<<endl;
    for(unsigned int i=0;i<Nmappoint;i++){
        ORB_SLAM2::MapPoint* mp=readMapPoint(f);
        if(mp->mnId >= maxId)
            maxId=mp->mnId;
        AddMapPoint(mp);
    }

    ORB_SLAM2::MapPoint::nNextId=maxId+1;
    std::vector<MapPoint*> amps=GetAllMapPoints();
//    MapPoint* vmp=amps[0];
//    cout<<vmp->mnId<<endl;

    long unsigned int Nkeyframes;
    maxId=0;
    map<unsigned long int,ConnectInformation> map_connection;
    f.read((char*)&Nkeyframes, sizeof(Nkeyframes));
    cout << "reading " << Nkeyframes << " keyframes" << endl;
    for (unsigned int i=0; i<Nkeyframes; i++) {
        KeyFrame* kf = readKeyFrame(f, voc, amps, &orb_ext,map_connection);
        if(kf->mnId > maxId)
           maxId=kf->mnId;
        AddKeyFrame(kf);
    }
    ORB_SLAM2::KeyFrame::nNextId=maxId+1;
    //Spanning tree
    map<unsigned long int,KeyFrame*> kf_by_id;
    for(KeyFrame* pkf:mspKeyFrames)
        kf_by_id[pkf->mnId]=pkf;
    for(map<unsigned long int,KeyFrame*>::iterator it=kf_by_id.begin();it!=kf_by_id.end();it++){
        ConnectInformation conn_info=map_connection[it->first];
        if(conn_info.parent_id!=ULONG_MAX){
            if(kf_by_id.find(conn_info.parent_id)==kf_by_id.end()){
                cerr<<"Bad parent"<<endl;
                continue;
            }
            it->second->ChangeParent(kf_by_id[conn_info.parent_id]);
        }
        for(unsigned long int i=0;i<conn_info.connect_id.size();i++){
            if(kf_by_id.find(conn_info.connect_id[i])==kf_by_id.end()){
                clog<<"Bad connection to KeyFrame with ID "<<conn_info.connect_id[i]<<endl;
                continue;
            }
            it->second->AddConnection(kf_by_id[conn_info.connect_id[i]],conn_info.weight[i]);
        }
    }

    for(MapPoint* mp: amps){
        mp->ComputeDistinctiveDescriptors();
        mp->UpdateNormalAndDepth();
    }

    f.close();
    return true;
}
}//namespace ORB_SLAM2
