#include <pcl/pcl_config.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/time.h>
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
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni_grabber.h>
#include <boost/shared_ptr.hpp>


const float grippersize = 0.2;

//see http://www.openperception.net/documentation/tutorials/openni_grabber.php#openni-grabber

 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer() : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudcallback)
     {
       if (!viewer.wasStopped())
       {
		   
		   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> (*cloudcallback));
		  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		 //  cloud = cloudcallback.ma;
			 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
         viewer.showCloud (cloud);
         
         
         
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

// 	      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
// 	      std::cout << "Centroid " << centroid[0] << "," << centroid[1] << "," << centroid[2] << "." << std::endl;
// 	      std::cout << "min x: " << minX << std::endl;
// 	      std::cout << "max x: " << maxX << std::endl;
// 	      std::cout << "min y: " << minY << std::endl;
// 	      std::cout << "max y: " << maxY << std::endl;

	      float xdist = std::abs(minX-maxY);
	      float ydist = std::abs(minY-maxY);

// 	      std::cout << "dist x: " << xdist << std::endl;
// 	      std::cout << "dist y: " << ydist << std::endl;

	      if(xdist < grippersize || ydist < grippersize){
	        std::cout << "Graspable" << std::endl;
	     	std::cout << "" << centroid[0] << std::endl;
	     	std::cout << "" << centroid[1] << std::endl;
	     	std::cout << "" << centroid[2] << std::endl;
	      }
	  }
         
    	}
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

int main(int argc, char** argv)
{
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
}
