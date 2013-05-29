#include <iostream>
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


sensor_msgs::PointCloud2 kinectcloud;
ros::Publisher visionPublisher;

// Ros kinect topic callback
void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	//Inside the callback should be all the process that you want to do with your point cloud and at the end publish the results.
	printf("Before filtering Cloud: width = %d, height = %d\n", input->width,
			input->height);

	sensor_msgs::PointCloud2 voxeloutput;

	//Down sizing the cloud because of the real time performance
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (voxeloutput);

	std_msgs::String msg;
	std::stringstream ss;
	ss << "width " << voxeloutput.width << "\n";
	ss << "height " << voxeloutput.height << "\n";
	msg.data = ss.str();
	visionPublisher.publish(msg);

	pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::fromROSMsg(voxeloutput, cloud);

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

	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud.makeShared();
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

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
	    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

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
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	    std::stringstream ss;
	    ss << "cloud_cluster_" << j << ".pcd";
	    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
	    j++;
	  }

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "SUB_IND_PUB");
	ros::NodeHandle nh;
	visionPublisher = nh.advertise<std_msgs::String>("/vision/grabbing/location", 1000);

	//ros::Subscriber sub = nh.subscribe<pcl::PointCloud>("/camera/depth_registered/points", 1, callback);
	ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);

	  //This will run until you shutdown the node
	ros::spin();
}

/*int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = 30;
	cloud.height = 1;
	cloud.points.resize(cloud.width * cloud.height);

	// Generate the data
	for (size_t i = 0; i < cloud.points.size() / 2; ++i) {
		//cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].x = 1.0;
		//cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1.0;
		//cloud.points[i].z = 1.0;
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	for (size_t i = cloud.points.size() / 2; i < cloud.points.size(); ++i) {
		//cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].x = 2.0;
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		//cloud.points[i].y= 2.0;
		//cloud.points[i].z = 2.0;
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	// Set a few outliers
	cloud.points[0].z = 2.0;
	cloud.points[3].z = -2.0;
	cloud.points[6].z = 4.0;
	cloud.points[7].z = 9.0;
	cloud.points[16].z = 12.0;
	cloud.points[23].z = -12.0;
	cloud.points[25].z = 14.0;
	cloud.points[27].z = 19.0;

	std::cerr << "Point cloud data: " << cloud.points.size() << " points"
			<< std::endl;
	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y
				<< " " << cloud.points[i].z << std::endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud.makeShared());
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " " << coefficients->values[2] << " "
			<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	for (size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    "
				<< cloud.points[inliers->indices[i]].x << " "
				<< cloud.points[inliers->indices[i]].y << " "
				<< cloud.points[inliers->indices[i]].z << std::endl;

	return (0);
}*/
