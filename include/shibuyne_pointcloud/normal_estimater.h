#ifndef NORMAL_ESTIMATER_H
#define NORMAL_ESTIMATER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>

class NormalEstimater{
	public:
		NormalEstimater();
		typedef pcl::PointNormal PointNormal;
		typedef pcl::PointCloud<PointNormal> CloudNormal;
		typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr;

		// typedef pcl::PointXYZINormal PointXYZIN;
		// typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
		// typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
	
	private:
		void cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void estimate_normal(void);
		void Visualization(void);
		void publish(void);
		void disp(const CloudNormalPtr &);
		void edge_detection(const CloudNormalPtr &);
		ros::NodeHandle nh;
    	ros::NodeHandle local_nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		ros::Publisher pub_mk;
		ros::Publisher pub_edge_cloud;
		// pcl::visualization::PCLVisualizer viewer{"pc_normals"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		// visualization_msgs::Marker mk;
		double CURVATURE_THRESHOLD;
};
#endif
