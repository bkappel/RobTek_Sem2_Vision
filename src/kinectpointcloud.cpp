#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include "boost/shared_ptr.hpp"



sensor_msgs::PointCloud2 kinectcloud;
ros::Publisher visionPublisher;

const float grippersize = 0.2;

// Ros kinect topic callback
void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	//Inside the callback should be all the process that needed to be done with the point cloud
	pcl::PointCloud<pcl::PointXYZ> cloudKinect;
	pcl::fromROSMsg(*input, cloudKinect);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&cloudKinect);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

			 // Create the filtering object: downsample the dataset using a leaf size of 1cm
	  pcl::VoxelGrid<pcl::PointXYZ> vg;
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	  vg.setInputCloud (cloud);
	  vg.setLeafSize (0.01f, 0.01f, 0.01f);
	  vg.filter (*cloud_filtered);

	  // Create the segmentation object for the planar model and set all the parameters
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	  pcl::PCDWriter writer;
	  seg.setOptimizeCoefficients (true);
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setMaxIterations (100);
	  seg.setDistanceThreshold (0.02);

	  int i=0, nr_points = (int) cloud_filtered->points.size ();
	  while (cloud_filtered->points.size () > 0.3 * nr_points)
	  {
	    // Segment the largest planar component from the remaining cloud
	    seg.setInputCloud (cloud_filtered);
	    seg.segment (*inliers, *coefficients);
	    if (inliers->indices.size () == 0)
	    {
	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	      break;
	    }

	    // Extract the planar inliers from the input cloud
	    pcl::ExtractIndices<pcl::PointXYZ> extract;
	    extract.setInputCloud (cloud_filtered);
	    extract.setIndices (inliers);
	    extract.setNegative (false);

	    // Get the points associated with the planar surface
	    extract.filter (*cloud_plane);

	    // Remove the planar inliers, extract the rest
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	 *cloud_filtered = *cloud_f;
	  }

	  // Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud_filtered);

	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.02); // 2cm
	  ec.setMinClusterSize (100);
	  ec.setMaxClusterSize (25000);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (cloud_filtered);
	  ec.extract (cluster_indices);

	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	    {
	      float minX = 1000000.0;
	      float minY = 1000000.0;
	      float maxX = -1000000.0;
	      float maxY = -1000000.0;

	      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
	        pcl::PointXYZ p = cloud_filtered->points[*pit];
	        cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
	  	if(p.x < minX){ minX = p.x; }
	  	if(p.x > maxX){ maxX = p.x; }
	  	if(p.y < minY){ minY = p.y; }
	  	if(p.y > maxY){ maxY = p.y; }
	  }
	      cloud_cluster->width = cloud_cluster->points.size ();
	      cloud_cluster->height = 1;
	      cloud_cluster->is_dense = true;

	      Eigen::Vector4f centroid;
	      pcl::compute3DCentroid(*cloud_cluster, centroid);

	      /*for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud_cluster->points.begin(); p < cloud_cluster->points.end(); p++)
	      {
	  	if(p.x < minX){ minX = p; }
	  	if(p.x > maxX){ maxX = p; }
	  	if(p.y < minY){ minY = p; }
	  	if(p.y > maxY){ maxY = p; }
	      }*/

	      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	      std::cout << "Centroid " << centroid[0] << "," << centroid[1] << "," << centroid[2] << "." << std::endl;
	      std::cout << "min x: " << minX << std::endl;
	      std::cout << "max x: " << maxX << std::endl;
	      std::cout << "min y: " << minY << std::endl;
	      std::cout << "max y: " << maxY << std::endl;

	      float xdist = std::abs(minX-maxY);
	      float ydist = std::abs(minY-maxY);

	      std::cout << "dist x: " << xdist << std::endl;
	      std::cout << "dist y: " << ydist << std::endl;

	      if(xdist < grippersize || ydist < grippersize){
	        std::cout << "Graspable" << std::endl;
	     	std_msgs::String msg;
	     	std::stringstream ss;
	     	ss << centroid[0] << ",";
	     	ss << centroid[1] << ",";
	     	ss << centroid[2];
	     	msg.data = ss.str();
	     	visionPublisher.publish(msg);
	      }

	  }



}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "SUB_IND_PUB");
	ros::NodeHandle nh;
	visionPublisher = nh.advertise<std_msgs::String>("/vision/grabbing/location", 1000);
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
	ros::spin();
}
