#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr img2pc(cv::Mat& depth, cv::Mat& rgb, float fx,float fy,float cx,float cy,float scale, int colormode){
    PointCloud::Ptr cloud (new PointCloud);

    for(int m=0; m<depth.rows;m++){
        for(int n=0; n<depth.cols;n++){
            ushort d = depth.at<ushort>(m,n);

            if(d==0){
                //cout<<"No depth"<<endl;
                continue;
            }
            //cout<<"depth"<<endl;
            PointT p;

            //p.x = (double)d / scale;
            //p.y = (n - cx) * p.x / fx;
            //p.z = (m - cy) * p.x / fy; //The coordinate frames of PCL and Opencv are different

            double z = (double)d / scale;
            double x = (n - cx) * z / fx;
            double y = (m - cy) * z / fy; //The coordinate frames of PCL and Opencv are different

            p.x=z;
            p.y=-x;
            p.z=-y;

            if(colormode==0){
                p.r=rgb.ptr<uchar>(m)[n*3+2];
                p.g=rgb.ptr<uchar>(m)[n*3+1];
                p.b=rgb.ptr<uchar>(m)[n*3];
            }
            else if(colormode==1){
                p.r=rgb.ptr<uchar>(m)[n*3];
                p.g=rgb.ptr<uchar>(m)[n*3+1];
                p.b=rgb.ptr<uchar>(m)[n*3+2];
            }

            cloud->points.push_back(p);
        }
    }

    cloud->height=1;
    cloud->width=cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

Eigen::Isometry3d loadT(ifstream& f){
    float xx,yy,zz,qx,qy,qz,qw;
    f.read((char*)&xx,sizeof(float));
    f.read((char*)&yy,sizeof(float));
    f.read((char*)&zz,sizeof(float));
    f.read((char*)&qx,sizeof(float));
    f.read((char*)&qy,sizeof(float));
    f.read((char*)&qz,sizeof(float));
    f.read((char*)&qw,sizeof(float));

    Eigen::Quaterniond q(qw,qz,-qx,-qy);
    Eigen::Isometry3d t(q);

    t(0,3)=zz;t(1,3)=-xx;t(2,3)=-yy;
    
    //t(0,3)=zz;t(1,3)=-xx;t(2,3)=yy;
    cout<<xx<<","<<yy<<","<<zz<<","<<qx<<","<<qy<<","<<qz<<","<<qw<<endl;
    return t;
}

int main(int argc,char** argv){
    string camSettingPath;
    if(argc==2){
        camSettingPath=string(argv[1]);//"./config/param.yaml"
    }else{
        camSettingPath=string("./config/param.yaml");
    }

    cv::Mat depth;
    cv::FileStorage fSettings(camSettingPath,cv::FileStorage::READ);

    float fx,fy,cx,cy,scale,limxl,limxu;
    float gridsize;
    int colormode;

    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    scale = fSettings["Camera.scale"];
    limxl = fSettings["Camera.limxl"];
    limxu = fSettings["Camera.limxu"];

    gridsize = fSettings["Gridsize"];
    string scolormode = fSettings["Camera.colormode"];

    if(scolormode=="RGB")
        colormode=1;
    else
        colormode=0;

    string datapath = fSettings["dataPath"];
    string KeyFrameTraj=datapath+"/KeyFrameTrajectory.bin";
    bool visualize =1;
    //cout<<KeyFrameTraj<<endl;
    ifstream f;
    pcl::visualization::CloudViewer viewer("PCL viewer");

    int Nimg=0; //Number of images, corresponding to KeyFrame
    int is_goodframe;
    long unsigned int imId=0;
    //
    //string imgfilename;
    stringstream ss;
    cv::Mat depthim,rgbim;

    f.open(KeyFrameTraj.c_str(),ios_base::in|ios::binary);
    f.read((char*)&Nimg,sizeof(size_t));
    cout<<Nimg<<" KeyFrame to generate point cloud totally"<<endl;

    PointCloud::Ptr cloud(new PointCloud());
    PointCloud::Ptr fcloud(new PointCloud());

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(gridsize,gridsize,gridsize);

    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName("x");
    pass.setFilterLimits(limxl, limxu );

    for(int i=0;i<Nimg;i++){
        f.read((char*)&is_goodframe,sizeof(int));
        if(is_goodframe){
            f.read((char*)&imId,sizeof(long unsigned int));
            //Reading color and depth image
            ss.str("");
            ss <<datapath<<"dimgkf" <<imId<<".png";
            depthim=cv::imread(ss.str(),CV_LOAD_IMAGE_ANYDEPTH);
            
            ss.str("");
            ss <<datapath<<"cimgkf" <<imId<<".png";
            rgbim=cv::imread(ss.str());

            Eigen::Isometry3d t=loadT(f);

            PointCloud::Ptr origincloud = img2pc(depthim,rgbim,fx,fy,cx,cy,scale,colormode);
            PointCloud::Ptr passcloud(new PointCloud());    
            PointCloud::Ptr transformcloud(new PointCloud());

            //viewer.showCloud(origincloud);
            pass.setInputCloud(origincloud);
            pass.filter(*passcloud);

            pcl::transformPointCloud(*passcloud,*transformcloud,t.matrix());
            *cloud +=  *transformcloud;
            //cout<<xx<<","<<yy<<","<<zz<<","<<qx<<","<<qy<<","<<qz<<","<<qw<<endl;
        }
    }

    voxel.setInputCloud(cloud);
    voxel.filter(*fcloud);

    //cout<<"finishing"<<endl;
    //cout<<"Saving point cloud data to "<< datapath<<"/result.pcd"<<endl;
    //pcl::io::savePCDFile(datapath+"/PCresult.pcd", *cloud);

    if(visualize)
        viewer.showCloud(fcloud);
    
    while(1){
        sleep(1);
    }
    return 0;
}
