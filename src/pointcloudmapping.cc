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
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/poisson.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "Converter.h"

pcl::visualization::PCLVisualizer surface_viewer("Surface");
bool init=false;
Eigen::Matrix3f intrinsics;
PointCloudMapping::PointCloudMapping(Map* pMap):mpMap(pMap)
{
    // voxel.setLeafSize( resolution, resolution, resolution);
    //globalMap = boost::make_shared< PointCloud >( );
    
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

void PointCloudMapping::insertKeyFrame(KeyFrame* kf)
{

    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    //keyframes.push_back( kf );
    keyframe = kf;
    // colorImgs.push_back( color.clone() );
    // depthImgs.push_back( depth.clone() );
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< pcl::PointXYZ >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    // if(vpRefMPs.empty())
    //     return;

    pcl::PointCloud< pcl::PointXYZ >::Ptr tmp( new pcl::PointCloud< pcl::PointXYZ > );

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZ p;
        p.z = pos.at<float>(2);
        p.x = pos.at<float>(0);
        p.y = pos.at<float>(1);
        tmp->points.push_back(p);
    }
    
    if(!init)
    {
        intrinsics<<kf->fx,0,kf->cx,
            0,kf->fy,kf->cy,
            0,0,1;
            init=true;
            std::cout<<intrinsics<<std::endl;
    }
    Eigen::Matrix4f extrinsics;

    cv::Mat pose =kf->GetPose();
    extrinsics << pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),pose.at<float>(0,3),
         pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),pose.at<float>(1,3),
         pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2),pose.at<float>(2,3),
         pose.at<float>(3,0), pose.at<float>(3,1), pose.at<float>(3,2),pose.at<float>(3,3);
         std::cout<<extrinsics<<std::endl;
         surface_viewer.setCameraParameters(intrinsics,extrinsics);
    // PointCloud::Ptr cloud(new PointCloud);
    // pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    // cloud->is_dense = false;

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<tmp->points.size()<<endl;
    // return cloud;
    return tmp;
}


void PointCloudMapping::viewer()
{
    
    pcl::visualization::CloudViewer cloudviewer("cloudviewer");
    cloudviewer.registerKeyboardCallback( &PointCloudMapping::keyboard_callback, *this );
            surface_viewer.setBackgroundColor(0, 0, 0.6);  //设置窗口颜色

        //surface_viewer.setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
        surface_viewer.setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
        //surface_viewer.setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示
        surface_viewer.addCoordinateSystem(1);  //设置坐标系,参数为坐标显示尺寸
        surface_viewer.initCameraParameters();
    while(1)
    {
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

            pcl::PointCloud< pcl::PointXYZ >::Ptr p = generatePointCloud( keyframe);
        cloudviewer.showCloud( p );




    }

}

void PointCloudMapping::keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* )
{
    std::cout<<"Generating Surface...."<<std::endl;
        // if( event.getKeyCode() && event.keyDown() ){
        // 	if(event.isShiftPressed())
        //         std::cout<<"shift"<<std::endl;
        // }
//pcl::PointCloud< pcl::PointXYZ >::Ptr p = generatePointCloud( keyframe);
pcl::PointCloud< pcl::PointXYZ >::Ptr input( new pcl::PointCloud< pcl::PointXYZ > );
pcl::io::loadPCDFile<pcl::PointXYZ> ("./pointcloud.pcd", *input);
std::cout<<"Retrieved point cloud:"<< input->size() <<std::endl;
        //////////////////////////*MLS*//////////////////////
pcl::PointCloud< pcl::PointXYZ >::Ptr filtered( new pcl::PointCloud< pcl::PointXYZ > );
pcl::PointCloud< pcl::PointXYZ >::Ptr filtered1( new pcl::PointCloud< pcl::PointXYZ > );
pcl::PointCloud< pcl::PointXYZ >::Ptr mls_output( new pcl::PointCloud< pcl::PointXYZ > );
pcl::PointCloud< pcl::PointXYZ >::Ptr outputCloud( new pcl::PointCloud< pcl::PointXYZ > );


    /// Filter the small points /////////
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(input);
    outrem.setRadiusSearch(0.05);
    outrem.setMinNeighborsInRadius (3);
    // apply filter
    outrem.filter (*filtered);

    std::cout<<"Filter the small points:"<< filtered->size() <<std::endl;



    ////// VoxelGrid Filter ////////
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (filtered);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*filtered1);
    std::cout << "VoxelGrid:"<<filtered1->size() << std::endl;

    // MLS filter
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kdtree用于法向计算时近邻搜索
    tree->setInputCloud(filtered1);//为kdtree输入点云

    mls.setInputCloud(filtered1);
    mls.setSearchMethod (tree);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder(4);
    mls.setComputeNormals(false);
    mls.setSearchRadius(0.2);
    // mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE); 
    // mls.setUpsamplingRadius(0.05);
    // mls.setUpsamplingStepSize(0.015);
    mls.process(*mls_output);

    std::cout<<"Mls:"<< mls_output->size() <<std::endl;


        //////////////////////////*法向估计模块*//////////////////////
        // Normal estimation（法向量估计）
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//创建法向估计对象
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//创建法向数据指针
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);//创建kdtree用于法向计算时近邻搜索
        tree->setInputCloud(mls_output);//为kdtree输入点云
        n.setInputCloud(mls_output);//为法向估计对象输入点云
        n.setSearchMethod(tree1);//设置法向估计时采用的搜索方式为kdtree
        n.setKSearch(20);//设置法向估计时,k近邻搜索的点数
        n.compute(*normals);  //进行法向估计
    
        /*点云数据与法向数据拼接*/
        // 创建同时包含点和法向的数据结构的指针
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        //将已获得的点数据和法向数据拼接
        pcl::concatenateFields(*mls_output, *normals, *cloud_with_normals);

        //////////////////////////*泊松重建模块*//////////////////////
             pcl::Poisson<pcl::PointNormal> poisson;
        // poisson.setDepth(9);
        //输入poisson重建点云数据
        poisson.setInputCloud(cloud_with_normals);
        //创建网格对象指针，用于存储重建结果
        pcl::PolygonMesh triangles;
        //poisson重建开始
        poisson.reconstruct(triangles);
        surface_viewer.removePolygonMesh("my");
        surface_viewer.addPolygonMesh(triangles, "my");  //设置所要显示的网格对象

        surface_viewer.spinOnce(100);
        //cout<<"show global map, size="<<globalMap->points.size()<<endl;
        //lastKeyframeSize = N;
}


