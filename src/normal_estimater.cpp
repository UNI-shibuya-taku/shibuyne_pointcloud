#include <shibuyne_pointcloud/normal_estimater.h>
NormalEstimater::NormalEstimater()
{
	sub = nh.subscribe("/cloud/lcl", 1, &NormalEstimater::cloud_callback, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/shibuyne_pointcloud/normals", 1);
  pub_mk = nh.advertise<visualization_msgs::Marker>("/shibuyne_pointcloud/normal_vector",1);
	// viewer.setBackgroundColor(1, 1, 1);
	// viewer.addCoordinateSystem(0.5, "axis");
}
void NormalEstimater::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "-----normal cloud_callback------" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	copyPointCloud(*cloud, *normals);
	estimate_normal();
	disp(normals);
	// Visualization();
	publish();
}
void NormalEstimater::estimate_normal(void)
{
	// pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> ne;
	ne.setInputCloud(normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.5); // 探索範囲
	ne.compute(*normals);
}
void NormalEstimater::disp(const CloudNormalPtr &pc)
{
    // mk.header.frame_id = "/velodyne";
    // mk.header.frame_id = "sqlidar_link";
		visualization_msgs::Marker mk;
    mk.header.frame_id = "base_link"; // SQ2
    mk.header.stamp = ros::Time();
    mk.ns = "vector";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.scale.x = 0.01;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 0.3;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    // size_t sz = pc.points.size();
    size_t sz = pc->points.size();
    mk.points.clear();
    for(size_t i = 0; i < sz; i += 1){
			// std::cout << "x: " << pc->points[i].x << std::endl;
			// std::cout << "y: " << pc->points[i].y << std::endl;
			// std::cout << "z: " << pc->points[i].z << std::endl;
			geometry_msgs::Point p1,p2;
			p1.x = pc->points[i].x;
			p1.y = pc->points[i].y;
			p1.z = pc->points[i].z;
			p2.x = pc->points[i].x - 0.1*pc->points[i].normal_x;
			p2.y = pc->points[i].y - 0.1*pc->points[i].normal_y;
			p2.z = pc->points[i].z - 0.1*pc->points[i].normal_z;
			mk.points.push_back(p1);
			mk.points.push_back(p2);
    }
    pub_mk.publish(mk);
}
void NormalEstimater::publish(void)
{
	sensor_msgs::PointCloud2 normals_pub;
	pcl::toROSMsg(*normals, normals_pub);
	pub.publish(normals_pub);
}
// void NormalEstimater::Visualization(void)
// {

// 	viewer.removeAllPointClouds();

// 	viewer.addPointCloud(cloud, "cloud");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

// 	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 10, 0.5, "normals");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");

// 	viewer.spinOnce();
// }
int main(int argc, char** argv)
{
	std::cout << "Normal Estimate!!!" << std::endl;
	ros::init(argc, argv, "normal_estimater");
	NormalEstimater normal_estimater;
	ros::spin();
}