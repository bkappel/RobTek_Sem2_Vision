#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int KINECTCLOUDWIDTH = 640;
int KINECTCLOUDHEIGHT = 480;

pcl::PointCloud<pcl::PointXYZ> createPointCloudFromFile(std::string filelocation)

int main (int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZRGB> cloud  = createPointCloudFromFile("/images/kinect_depth_sample.txt")

	pcl_visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{

	}

	/*pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width    = 5;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
	cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

	for (size_t i = 0; i < cloud.points.size (); ++i)
	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;*/

	return (0);
}

pcl::PointCloud<pcl::PointXYZ> createPointCloudFromFile(std::string filelocation)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pointcloud.width = this.KINECTCLOUDWIDTH;
	pointcloud.height = this.KINECTCLOUDHEIGHT;
	pointcloud.points.resize (pointcloud.height * pointcloud.width); 

	int* depth_data = new int[pointcloud.height * pointcloud.width]; 

	std::ifstream depthDataFile(filelocation);
	std::string line;
	int pos = 0;
	while(std::getline(depthDataFile, line))
	{
		std::istringstream ss(line);
		std::string token;
		while(std::getline(ss, token, ','))
		{
			int val = atoi(token.c_str());
			//depth_data << val;
			depth_data[pos] = val;
		}
	}

	//copy the depth values of every pixel in here 

	register float constant = 1.0f / 525; 
	register int centerX = (pointcloud.width >> 1); 
	int centerY = (pointcloud.height >> 1); 
	register int depth_idx = 0; 
	for (int v = -centerY; v < centerY; ++v) 
	{ 
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx) 
		{ 
			pcl::PointXYZ& pt = pointcloud.points[depth_idx]; 
			pt.z = depth_data[depth_idx] * 0.001f; 
			pt.x = static_cast<float> (u) * pt.z * constant; 
			pt.y = static_cast<float> (v) * pt.z * constant; 
		} 
	} 
	pointcloud.sensor_origin_.setZero (); 
	pointcloud.sensor_orientation_.w () = 0.0f; 
	pointcloud.sensor_orientation_.x () = 1.0f; 
	pointcloud.sensor_orientation_.y () = 0.0f; 
	pointcloud.sensor_orientation_.z () = 0.0f; 
}




/*pcl::OpenNIGrabber::convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image) const 
{ 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>); 

cloud->height = depth_height_; 
cloud->width = depth_width_; 
cloud->is_dense = false; 

cloud->points.resize (cloud->height * cloud->width); 

//I inserted 525 as Julius Kammerl said as focal length 
register float constant = 1.0f / device_->getDepthFocalLength (depth_width_); 

//the frame id is completely ignored 
if (device_->isDepthRegistered ()) 
cloud->header.frame_id = rgb_frame_id_; 
else 
cloud->header.frame_id = depth_frame_id_;

register int centerX = (cloud->width >> 1); 
int centerY = (cloud->height >> 1); 

//I also ignore invalid values completely 
float bad_point = std::numeric_limits<float>::quiet_NaN ();

//this section is used to get the depth data array, I replaced it with my code (and you did it with your code) 
register const unsigned short* depth_map = depth_image->getDepthMetaData ().Data (); 
if (depth_image->getWidth() != depth_width_ || depth_image->getHeight () != depth_height_) 
{ 
static unsigned buffer_size = 0; 
static boost::shared_array<unsigned short> depth_buffer (0); 

if (buffer_size < depth_width_ * depth_height_) 
{ 
buffer_size = depth_width_ * depth_height_; 
depth_buffer.reset (new unsigned short [buffer_size]); 
} 
depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ()); 
depth_map = depth_buffer.get (); 
}

//these for loops are mostly used as they are 
register int depth_idx = 0; 
for (int v = -centerY; v < centerY; ++v) 
{ 
for (register int u = -centerX; u < centerX; ++u, ++depth_idx) 
{ 
pcl::PointXYZ& pt = cloud->points[depth_idx]; 

//This part is used for invalid measurements, I removed it 
if (depth_map[depth_idx] == 0 || 
depth_map[depth_idx] == depth_image->getNoSampleValue () || 
depth_map[depth_idx] == depth_image->getShadowValue ()) 
{ 
// not valid 
pt.x = pt.y = pt.z = bad_point; 
continue; 
}
pt.z = depth_map[depth_idx] * 0.001f; 
pt.x = static_cast<float> (u) * pt.z * constant; 
pt.y = static_cast<float> (v) * pt.z * constant; 
} 
} 
cloud->sensor_origin_.setZero (); 
cloud->sensor_orientation_.w () = 0.0f; 
cloud->sensor_orientation_.x () = 1.0f; 
cloud->sensor_orientation_.y () = 0.0f; 
cloud->sensor_orientation_.z () = 0.0f;   
return (cloud); 
}*/