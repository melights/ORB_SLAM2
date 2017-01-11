/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */
#include <fstream>
#include "pointcloudmapping.h"
#include <MapPoint.h>
#include <KeyFrame.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/poisson.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "Converter.h"
#include "StereoEfficientLargeScale.h"
using namespace cv;
//pcl::visualization::PCLVisualizer surface_viewer("Surface");
bool init=false;
Eigen::Matrix3f intrinsics;
PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    namedWindow( "Disparity", WINDOW_NORMAL );
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& left_img, cv::Mat& right_img)
{

    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    //keyframes.push_back( kf );
    keyframe = kf;
    left=left_img;
    right=right_img;
    // colorImgs.push_back( color.clone() );
    // depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud()
{
    Mat left_grey, right_gery;
    cvtColor(left, left_grey, cv::COLOR_RGB2GRAY);
    cvtColor(right, right_gery, cv::COLOR_RGB2GRAY);


    // Ptr<StereoBM> bm = StereoBM::create(16,9);
    // Rect roi1, roi2;
    // Mat disp,disp8U;
    //     bm->setROI1(roi1);
    // bm->setROI2(roi2);
    // bm->setPreFilterCap(1);
    // bm->setBlockSize(23);
    // bm->setMinDisparity(0);
    // bm->setNumDisparities(48);
    // bm->setTextureThreshold(0);
    // bm->setUniquenessRatio(0);
    // bm->setSpeckleWindowSize(0);
    // bm->setSpeckleRange(0);
    // bm->setDisp12MaxDiff(1);
    // bm->compute(left_grey, right_gery, disp);


    Ptr<StereoBM> bm = StereoBM::create(96,55);
    Rect roi1, roi2;
    Mat disp,disp8U;
    //     bm->setROI1(roi1);
    // bm->setROI2(roi2);
    // bm->setPreFilterCap(1);
    // bm->setBlockSize(55);
    // bm->setMinDisparity(0);
    // bm->setNumDisparities(96);
    // bm->setTextureThreshold(0);
    // bm->setUniquenessRatio(0);
    // bm->setSpeckleWindowSize(0);
    // bm->setSpeckleRange(0);
    // bm->setDisp12MaxDiff(1);
    bm->compute(left_grey, right_gery, disp);

// Mat disp,disp8U;
// StereoEfficientLargeScale elas(0,128);
// elas(left_grey,right_gery,disp,100);


  double minVal; double maxVal;
  minMaxLoc( disp, &minVal, &maxVal );
  disp.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal));
  imshow( "Disparity", disp8U );



        double px, py, pz;
  uchar pr, pg, pb;
  PointCloud::Ptr tmp( new PointCloud() );
  for (int i = 0; i < left.rows; i++)
  {
      const short* disp_ptr = disp.ptr<short>(i);
      for (int j = 0; j < left.cols; j++)
      {
          double d = static_cast<double>(disp_ptr[j])/16;
          //double d = (double)disp.at<Vec3b>(i, j)[0];
          //std::cout<<d<<" ";
          if (d == -1||d == 0||d>100||d<0)
              continue; //Discard bad pixels

          pz = keyframe->mbf / d;
          px = (static_cast<double>(j) - keyframe->cx) * pz / keyframe->fx;
          py = (static_cast<double>(i) - keyframe->cy) * pz / keyframe->fy;
        //   if((i-240)*(i-240)+(j-320)*(j-320)>67600) //circle mask for simulation data
        //     continue;
          PointT point;
          point.x = px;
          point.y = py;
          point.z = pz;
          point.b = left.at<Vec3b>(i, j)[0];
          point.g = left.at<Vec3b>(i, j)[1];
          point.r = left.at<Vec3b>(i, j)[2];
        //   if (point.b==64 && point.g==64 && point.r==64) //remove grey background for simulation data
        //     continue;
          tmp->points.push_back(point);
      }
  }
      Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( keyframe->GetPose() );
      PointCloud::Ptr cloud(new PointCloud);
      pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
      cloud->is_dense = false;
      //cloud->width = (int) cloud->points.size();
      //cloud->height = 1;
      //pcl::io::savePCDFile( "/home/long/surface_reconstruction/PCL/data/testfilter.pcd", *cloud );

       cout<<"generate point cloud for kf "<<keyframe->mnId<<", size="<<cloud->points.size()<<endl;
      PointCloud::Ptr filter1(new PointCloud());
      PointCloud::Ptr filter2(new PointCloud());
      
      pcl::RadiusOutlierRemoval<PointT> outrem;
      outrem.setInputCloud(cloud);
      outrem.setRadiusSearch(0.08);
      outrem.setMinNeighborsInRadius(20);
      // apply filter
      outrem.filter(*filter1);

    //   pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    //   statistical_filter.setMeanK(80);
    //   statistical_filter.setStddevMulThresh(1.0);
    //   statistical_filter.setInputCloud(filter1);
    //   statistical_filter.filter(*filter2);

      return filter1;
}

void PointCloudMapping::viewer()
{
    
    pcl::visualization::CloudViewer cloudviewer("3D Reconstruction");
            //surface_viewer.setBackgroundColor(0, 0, 0);  //设置窗口颜色

        //surface_viewer.setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
        // surface_viewer.setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
        // //surface_viewer.setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示
        // surface_viewer.addCoordinateSystem(1);  //设置坐标系,参数为坐标显示尺寸
        // surface_viewer.initCameraParameters();
    while(1)
    {
        int64 t = getTickCount();
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

            PointCloud::Ptr p = generatePointCloud();
            *globalMap += *p;

        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        //globalMap->swap( *tmp );
//cloudviewer.removeAllShapes();
// cloudviewer.removeAllPointClouds();
//         pcl::ModelCoefficients cylinder_coeff;
// cylinder_coeff.values.resize (7);    // We need 7 values
// cylinder_coeff.values[0] = keyframe->GetCameraCenter().at<float>(0);
// cylinder_coeff.values[1] = keyframe->GetCameraCenter().at<float>(1);
// cylinder_coeff.values[2] = keyframe->GetCameraCenter().at<float>(2);
// Mat R,Rvec;
// R=keyframe->GetRotation();
// cv::Rodrigues(R, Rvec);
// cylinder_coeff.values[3] = Rvec.at<float>(0)*10;
// cylinder_coeff.values[4] = Rvec.at<float>(1)*10;
// cylinder_coeff.values[5] = Rvec.at<float>(2)*10;
// cylinder_coeff.values[6] = 2;

//         cloudviewer.addCylinder(cylinder_coeff);
//         cloudviewer.addPointCloud( globalMap );
       cout<<"globalMap size="<<globalMap->points.size()<<endl;

cloudviewer.showCloud( globalMap );
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
//cloudviewer.spinOnce(100);

    }

}
