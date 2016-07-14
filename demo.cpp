#include "kinect2PointCloud.h"
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPcl(const cv::Mat colorMat, const cv::Mat cloudMat); 

int main(int argc, char *argv[])
{
  processor freenect2Processor = OPENCL;

  Kinect2PointCloud kinect2PointCloud(freenect2Processor, true);
  cv::Mat color, depth, pointCloudMat;
 
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  cloud = matToPcl(color, pointCloudMat);
  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  while(1)
    {
      kinect2PointCloud.updatePointCloud();
      color = kinect2PointCloud.getColor();
      // depth = kinect2PointCloud.getDepth();
      pointCloudMat = kinect2PointCloud.getPointCloud();

      cloud = matToPcl(color, pointCloudMat);
      cv::imshow("color", color);
      // cv::imshow("depth", depth);
      cv::imshow("cloud", pointCloudMat);
      char key = cv::waitKey(3);
      if(key == 'q')break;

      viewer->spinOnce ();
      std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
      // std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    }
  kinect2PointCloud.shutDown();
  return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToPcl(const cv::Mat colorMat, const cv::Mat cloudMat) {
  const short w = colorMat.cols;
  const short h = colorMat.rows;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w ,h));
  pcl::PointXYZRGB * itP = &cloud->points[0];
  float * itCloudMat = (float *)cloudMat.ptr();
  char * itColorMat = (char *)colorMat.ptr();
  for(int y = 0; y < h; ++y) {
    for(int x = 0; x < w; ++x) {
      itP->x = *(itCloudMat++);
      itP->y = *(itCloudMat++);
      itP->z = *(itCloudMat++);
      itP->b = *(itColorMat++);
      itP->g = *(itColorMat++);
      itP->r = *(itColorMat++);
      itColorMat++;
      itP++;
    }
    cloud->is_dense = false;
  }
  return cloud;
}
