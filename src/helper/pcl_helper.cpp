#include <mireg.hpp>
#include <project_classes.hpp>

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

void best_fit_plane(std::vector<point>& ground, std::vector<long double>& normal, std::vector<long double>& centroid)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int n=int(ground.size());
  centroid[0]=0.0;
  centroid[1]=0.0;
  centroid[2]=0.0;
  for(int i=0; i<n; ++i)
  {
  	pcl::PointXYZRGB pointv;
  	uint8_t r(0), g(255), b(0);
    pointv.x = ground[i].coordinate(0);
    pointv.y = ground[i].coordinate(1);
    pointv.z = ground[i].coordinate(2);
    centroid[0]+=ground[i].coordinate(0)/n;
    centroid[1]+=ground[i].coordinate(1)/n;
    centroid[2]+=ground[i].coordinate(2)/n;
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		pointv.rgb = *reinterpret_cast<float*>(&rgb);
		cloud_ptr->points.push_back(pointv);
	}
	cloud_ptr->width = (int) cloud_ptr->points.size ();
	cloud_ptr->height = 1;
	
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.2);
  ne.compute (*cloud_normals);
  for(int i=0; i<n; ++i)
  {
		float norm=cloud_normals->points[i].normal_x*cloud_normals->points[i].normal_x+cloud_normals->points[i].normal_y*cloud_normals->points[i].normal_y+cloud_normals->points[i].normal_z*cloud_normals->points[i].normal_z;
		if(!(norm!=norm))
		{
			if(cloud_normals->points[i].normal_z>0)
			{
				normal[0]+=cloud_normals->points[i].normal_x;
				normal[1]+=cloud_normals->points[i].normal_y;
				normal[2]+=cloud_normals->points[i].normal_z;
			}
			else
			{
				normal[0]-=cloud_normals->points[i].normal_x;
				normal[1]-=cloud_normals->points[i].normal_y;
				normal[2]-=cloud_normals->points[i].normal_z;
			}
		}
  }
  float norm=sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
  normal[0]/=norm;
  normal[1]/=norm;
  normal[2]/=norm;
  std::cout<<normal[0]<<" "<<normal[1]<<" "<<normal[2]<<"\n";
  std::cout<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<"\n";
  /*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
  viewer = normalsVis(cloud_ptr, cloud_normals);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */
}

void best_fit_plane1(std::vector<point>& ground, Eigen::Vector3d& normal, Eigen::Vector3d& centroid)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int n=int(ground.size());
  normal << 0, 0, 0; centroid << 0, 0, 0;
  for(int i=0; i<n; ++i)
  {
  	pcl::PointXYZ point;
    point.x = ground[i].coordinate(0);
    point.y = ground[i].coordinate(1);
    point.z = ground[i].coordinate(2);
    centroid[0]+=ground[i].coordinate(0)/n;
    centroid[1]+=ground[i].coordinate(1)/n;
    centroid[2]+=ground[i].coordinate(2)/n;
    cloud->points.push_back(point);
	}
	cloud->width = (int) cloud->points.size ();
	cloud->height = 1;
	
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  
  if (inliers->indices.size () == 0)
  {
    std::cout<<"Could not estimate a planar model for the given dataset.";
    return;
  }
  normal[0]=coefficients->values[0];
  normal[1]=coefficients->values[1];
  normal[2]=coefficients->values[2];
  float norm=sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
  normal[0]/=norm;
  normal[1]/=norm;
  normal[2]/=norm;
  if(normal[2]<0.0)
  {
  	normal[0]=-normal[0];
  	normal[1]=-normal[1];
  	normal[2]=-normal[2];
  }
}
