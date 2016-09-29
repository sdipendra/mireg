#include <mireg.hpp>
#include <project_classes.hpp>

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 1.00, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<float>& normal, std::vector<float>& centroid)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();


  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (normal[0]);
  coeffs.values.push_back (normal[1]);
  coeffs.values.push_back (normal[2]);
  coeffs.values.push_back (-(normal[0]*centroid[0]+normal[1]*centroid[1]+normal[2]*centroid[2]));
  viewer->addPlane (coeffs, "plane");

  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> complete_color(cloud1);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, complete_color, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> ground_color(cloud2);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, ground_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}

void plot(std::vector<std::vector<float>>& cloud)
{
  int n=int(cloud.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0; i<n; ++i)
  {
    pcl::PointXYZ basic_point;
    basic_point.x = cloud[i][0];
    basic_point.y = cloud[i][1];
    basic_point.z = cloud[i][2];
    basic_cloud_ptr->points.push_back(basic_point);
	}
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis(basic_cloud_ptr);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void plot_ground(std::vector<std::vector<float>>& ground, std::vector<std::vector<float>>& rest)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int n=int(ground.size());
  for(int i=0; i<n; ++i)
  {
  	pcl::PointXYZRGB point;
  	uint8_t r(0), g(255), b(0);
    point.x = ground[i][0];
    point.y = ground[i][1];
    point.z = ground[i][2];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud1_ptr->points.push_back(point);
		cloud2_ptr->points.push_back(point);
  }
	
  int m=int(rest.size());
  for(int i=0; i<m; ++i)
  {
  	pcl::PointXYZRGB point;
 	  uint8_t r(0), g(0), b(255);
    point.x = rest[i][0];
    point.y = rest[i][1];
    point.z = rest[i][2];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud1_ptr->points.push_back (point);
  }
	cloud1_ptr->width = (int) cloud1_ptr->points.size ();
	cloud1_ptr->height = 1;
  
  cloud2_ptr->width = (int) cloud2_ptr->points.size ();
  cloud2_ptr->height = 1;
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
  viewer = viewportsVis(cloud1_ptr, cloud2_ptr);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void plot_plane(std::vector<std::vector<float>>& ground, std::vector<std::vector<float>>& rest, std::vector<float>& normal, std::vector<float>& centroid)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int n=int(ground.size());
  for(int i=0; i<n; ++i)
  {
  	pcl::PointXYZRGB point;
  	uint8_t r(0), g(255), b(0);
    point.x = ground[i][0];
    point.y = ground[i][1];
    point.z = ground[i][2];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud1_ptr->points.push_back(point);
		cloud2_ptr->points.push_back(point);
  }
	
  int m=int(rest.size());
  for(int i=0; i<m; ++i)
  {
  	pcl::PointXYZRGB point;
 	  uint8_t r(0), g(0), b(255);
    point.x = rest[i][0];
    point.y = rest[i][1];
    point.z = rest[i][2];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud1_ptr->points.push_back (point);
  }
	cloud1_ptr->width = (int) cloud1_ptr->points.size ();
	cloud1_ptr->height = 1;
  
  cloud2_ptr->width = (int) cloud2_ptr->points.size ();
  cloud2_ptr->height = 1;
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
  viewer = shapesVis(cloud2_ptr, normal, centroid);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void plot_scans(std::vector<std::vector<float>>& reading, std::vector<std::vector<float>>& reference)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  int n=int(reading.size());
  for(int i=0; i<n; ++i)
  {
  	pcl::PointXYZRGB point;
  	uint8_t r(255), g(0), b(0);
    point.x = reading[i][0];
    point.y = reading[i][1];
    point.z = reading[i][2];
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud1_ptr->points.push_back(point);
  }
	
  int m=int(reference.size());
	for(int i=0; i<m; ++i)
	{
		pcl::PointXYZRGB point;
		uint8_t r(0), g(255), b(0);
		point.x = reference[i][0];
		point.y = reference[i][1];
		point.z = reference[i][2];
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud2_ptr->points.push_back (point);
	}
	cloud1_ptr->width = (int) cloud1_ptr->points.size ();
	cloud1_ptr->height = 1;

	cloud2_ptr->width = (int) cloud2_ptr->points.size ();
	cloud2_ptr->height = 1;
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
	viewer = viewportsVis(cloud1_ptr, cloud2_ptr);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void plot_merged(std::vector<point>& reading, std::vector<point>& reference)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(auto it : reading)
	{
		pcl::PointXYZRGB point;
		uint8_t r(255), g(0), b(0);
		point.x = it.x;
		point.y = it.y;
		point.z = it.z;
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud_ptr->points.push_back(point);
	}
	
	for(auto it : reference)
	{
		pcl::PointXYZRGB point;
		uint8_t r(0), g(255), b(0);
		point.x = it.x;
		point.y = it.y;
		point.z = it.z;
		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		point.rgb = *reinterpret_cast<float*>(&rgb);
		cloud_ptr->points.push_back (point);
	}
	cloud_ptr->width = (int)cloud_ptr->points.size();
	cloud_ptr->height = 1;
	 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; 
	viewer = rgbVis(cloud_ptr);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void plot_feature_map(std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map)
{
	// Normalize the map to 0, 255;
	float maxv = -std::numeric_limits<float>::max(), minv = std::numeric_limits<float>::max();
	for(decltype(map.second.begin()) it1=map.second.begin(); it1!=map.second.end(); ++it1)
	{
		for(decltype(it1->begin()) it2=it1->begin(); it2!=it1->end(); ++it2)
		{
			if(maxv < *it2)
			{
				maxv = *it2;
			}
			if(minv > *it2)
			{
				minv = *it2;
			}
		}
	}
	minv=0.0;	// for not normalizing with respect to min;

	std::vector<std::vector<float>> nmap(map.second);
	for(int i=0; i < map.second.size(); i++)
	{
		for(int j=0; j < map.second[0].size(); j++)
		{
			nmap[i][j] = float(((map.second[i][j] - minv) * 255.0) / (maxv - minv));
		}
	}
	
	cv::Mat image(map.second.size(), map.second[0].size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for(int i=0; i < int(image.rows); i++)
	{
		for(int j=0; j < int(image.cols); j++)
		{
			cv::Vec3b color;
			color[0]=nmap[i][j];
			color[1]=255-nmap[i][j];
			color[2]=0;
			image.at<cv::Vec3b>(cv::Point(j, i))=color;
		}
	}
	
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
	cv::imshow("Display window", image);
	cv::waitKey(0);
}

void plot_feature_map_merged(std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map1, std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map2)
{
	// Normalize the map to 0, 255;
	float maxv = -std::numeric_limits<float>::max(), minv = std::numeric_limits<float>::max();
	for(decltype(map1.second.begin()) it1=map1.second.begin(); it1!=map1.second.end(); ++it1)
	{
		for(decltype(it1->begin()) it2=it1->begin(); it2!=it1->end(); ++it2)
		{
			if(maxv < *it2)
			{
				maxv = *it2;
			}
			if(minv > *it2)
			{
				minv = *it2;
			}
		}
	}
	minv=0.0;	// for not normalizing with respect to min;

	std::vector<std::vector<float>> nmap1(map1.second);
	for(int i=0; i < map1.second.size(); i++)
	{
		for(int j=0; j < map1.second[0].size(); j++)
		{
			nmap1[i][j] = float(((map1.second[i][j] - minv) * 255.0) / (maxv - minv));
		}
	}
	
	cv::Mat image1(map1.second.size(), map1.second[0].size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for(int i=0; i < int(image1.rows); i++)
	{
		for(int j=0; j < int(image1.cols); j++)
		{
			cv::Vec3b color;
			color[0]=nmap1[i][j];
			color[1]=255-nmap1[i][j];
			color[2]=0;
			image1.at<cv::Vec3b>(cv::Point(j, i))=color;
		}
	}
	
	maxv = -std::numeric_limits<float>::max(); minv = std::numeric_limits<float>::max();
	for(decltype(map2.second.begin()) it1=map2.second.begin(); it1!=map2.second.end(); ++it1)
	{
		for(decltype(it1->begin()) it2=it1->begin(); it2!=it1->end(); ++it2)
		{
			if(maxv < *it2)
			{
				maxv = *it2;
			}
			if(minv > *it2)
			{
				minv = *it2;
			}
		}
	}
	minv=0.0;	// for not normalizing with respect to min;

	std::vector<std::vector<float>> nmap2(map2.second);
	for(int i=0; i < map2.second.size(); i++)
	{
		for(int j=0; j < map2.second[0].size(); j++)
		{
			nmap2[i][j] = float(((map2.second[i][j] - minv) * 255.0) / (maxv - minv));
		}
	}
	
	cv::Mat image2(map2.second.size(), map2.second[0].size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for(int i=0; i < int(image2.rows); i++)
	{
		for(int j=0; j < int(image2.cols); j++)
		{
			cv::Vec3b color;
			color[0]=nmap2[i][j];
			color[1]=255-nmap2[i][j];
			color[2]=0;
			image2.at<cv::Vec3b>(cv::Point(j, i))=color;
		}
	}

	cv::namedWindow("Display window1", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Display window2", cv::WINDOW_AUTOSIZE);

	cv::imshow("Display window1", image1);
	cv::imshow("Display window2", image2);

	cv::waitKey(0);
}
