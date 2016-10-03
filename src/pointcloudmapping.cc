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

#include "pointcloudmapping.h"
#include <MapPoint.h>
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
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
        
        // p.b = 0;
        // p.g = 255;
        // p.r = 0;
            
        tmp->points.push_back(p);
    }
    // // point cloud is null ptr
    // for ( int m=0; m<depth.rows; m+=3 )
    // {
    //     for ( int n=0; n<depth.cols; n+=3 )
    //     {
    //         float d = depth.ptr<float>(m)[n];
    //         if (d < 0.01 || d>10)
    //             continue;
    //         PointT p;
    //         p.z = d;
    //         p.x = ( n - kf->cx) * p.z / kf->fx;
    //         p.y = ( m - kf->cy) * p.z / kf->fy;
            
    //         p.b = color.ptr<uchar>(m)[n*3];
    //         p.g = color.ptr<uchar>(m)[n*3+1];
    //         p.r = color.ptr<uchar>(m)[n*3+2];
                
    //         tmp->points.push_back(p);
    //     }
    // }
    
    // Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
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




//         //////////////////////////*MLS*//////////////////////
// pcl::PointCloud< pcl::PointXYZ >::Ptr filtered( new pcl::PointCloud< pcl::PointXYZ > );
// pcl::PointCloud< pcl::PointXYZ >::Ptr filtered1( new pcl::PointCloud< pcl::PointXYZ > );
// pcl::PointCloud< pcl::PointXYZ >::Ptr outputCloud( new pcl::PointCloud< pcl::PointXYZ > );


// 	///// Filter the small points /////////
//     // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//     // // build the filter
//     // outrem.setInputCloud(p);
//     // outrem.setRadiusSearch(0.03);
//     // outrem.setMinNeighborsInRadius (2);
//     // // apply filter
//     // outrem.filter (*filtered);

// 	std::cout << p->size() << std::endl;
// 	// MLS filter
// 	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

// 	// search method
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);



// 	//////// VoxelGrid Filter ////////
// // 	  pcl::VoxelGrid<pcl::PointXYZ> sor;
// //   sor.setInputCloud (filtered);
// //   sor.setLeafSize (0.01f, 0.01f, 0.01f);
// //   sor.filter (*filtered1);
// // std::cout << filtered1->size() << std::endl;



// 	mls.setInputCloud(p);
// 	mls.setSearchMethod (tree);
// 	mls.setPolynomialFit (true);
// 	mls.setPolynomialOrder(4);
// 	mls.setComputeNormals(false);
// 	mls.setSearchRadius(0.5);
// 	// mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE); 
// 	// mls.setUpsamplingRadius(0.05);
// 	// mls.setUpsamplingStepSize(0.015);
// 	mls.process(*outputCloud);










//         //////////////////////////*法向估计模块*//////////////////////
//         // Normal estimation（法向量估计）
//         pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//创建法向估计对象
//         pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//创建法向数据指针
//         //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kdtree用于法向计算时近邻搜索
//         //tree->setInputCloud(p);//为kdtree输入点云
//         n.setInputCloud(outputCloud);//为法向估计对象输入点云
//         n.setSearchMethod(tree);//设置法向估计时采用的搜索方式为kdtree
//         n.setKSearch(20);//设置法向估计时,k近邻搜索的点数
//         n.compute(*normals);  //进行法向估计
    
//         std::cerr << "法线计算   完成" << std::endl;

//         /*点云数据与法向数据拼接*/
//         // 创建同时包含点和法向的数据结构的指针
//         pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//         //将已获得的点数据和法向数据拼接
//         pcl::concatenateFields(*p, *normals, *cloud_with_normals);

//         //////////////////////////*泊松重建模块*//////////////////////
//              pcl::Poisson<pcl::PointNormal> poisson;
//         // poisson.setDepth(9);
//         //输入poisson重建点云数据
//         poisson.setInputCloud(cloud_with_normals);
//         //创建网格对象指针，用于存储重建结果
//         pcl::PolygonMesh triangles;
//         //poisson重建开始
//         poisson.reconstruct(triangles);
//         surface_viewer.removePolygonMesh("my");
//         surface_viewer.addPolygonMesh(triangles, "my");  //设置所要显示的网格对象

//         surface_viewer.spinOnce(100);
//         //cout<<"show global map, size="<<globalMap->points.size()<<endl;
//         //lastKeyframeSize = N;
    }

}
    void PointCloudMapping::keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* )
    {

			if( event.getKeyCode() && event.keyDown() ){
				if(event.isShiftPressed())
                    std::cout<<"space"<<std::endl;
			}

    }


