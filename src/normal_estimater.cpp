#include <shibuyne_pointcloud/normal_estimater.h>
NormalEstimater::NormalEstimater() : local_nh("~")
{
	sub = nh.subscribe("/cloud", 1, &NormalEstimater::cloud_callback, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/shibuyne_pointcloud/normals", 1);
	// pub_mk = nh.advertise<visualization_msgs::Marker>("/shibuyne_pointcloud/normal_vector",1);
	pub_mk = nh.advertise<visualization_msgs::MarkerArray>("/shibuyne_pointcloud/normal_vector",1);
	pub_edge_cloud = nh.advertise<sensor_msgs::PointCloud2>("/shibuyne_pointcloud/edge_cloud", 1);
	// viewer.setBackgroundColor(1, 1, 1);
	// viewer.addCoordinateSystem(0.5, "axis");

    local_nh.param("CURVATURE_THRESHOLD", CURVATURE_THRESHOLD, {0.1});
}
void NormalEstimater::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "-----normal cloud_callback------" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	copyPointCloud(*cloud, *normals);
	estimate_normal();
	edge_detection(normals);
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
	ne.setRadiusSearch(0.3); // 探索範囲
	ne.compute(*normals);
}
void NormalEstimater::edge_detection(const CloudNormalPtr &pc)
{
	CloudNormalPtr edge_cloud_ptr(new CloudNormal);
    edge_cloud_ptr->header = pc->header;
    pcl::PassThrough<PointNormal> pass;
    pass.setInputCloud(pc);
	pass.setFilterFieldName("curvature");
	pass.setFilterLimits(0, CURVATURE_THRESHOLD);
	pass.setFilterLimitsNegative(true);

        // pass.setFilterFieldName("normal_z");
        // pass.setFilterLimits(-CURVATURE_THRESHOLD, CURVATURE_THRESHOLD);

    pass.filter(*edge_cloud_ptr);
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*edge_cloud_ptr, pc_);
	pub_edge_cloud.publish(pc_);
}
void NormalEstimater::disp(const CloudNormalPtr &pc)
{
    // mk.header.frame_id = "/velodyne";
    // mk.header.frame_id = "sqlidar_link";
	visualization_msgs::MarkerArray normal_arrows;

	// visualization_msgs::Marker normal_arrow;
	// normal_arrow.header.frame_id = "base_link";
	// normal_arrow.id = i;
	// normal_arrow.ns = "center_of_gravity_vel_estimater";
	// normal_arrow.type = visualization_msgs::Marker::ARROW;
	// normal_arrow.action = visualization_msgs::Marker::ADD;
	// normal_arrow.lifetime = ros::Duration();
	// normal_arrow.pose.position.x = current_centroid_ptr->points[i].x;
	// normal_arrow.pose.position.y = current_centroid_ptr->points[i].y;

    // size_t sz = pc.points.size();
    size_t sz = pc->points.size();
    // mk.points.clear();
    for(size_t i = 0; i < sz; i += 1){
		visualization_msgs::Marker mk;
		mk.header.frame_id = "base_link"; // SQ2
		mk.header.stamp = ros::Time();
		mk.ns = "normal_vector";
		mk.id = i;
		mk.lifetime = ros::Duration();
		mk.type = visualization_msgs::Marker::LINE_LIST;
		// mk.type = visualization_msgs::Marker::ARROW;
		mk.action = visualization_msgs::Marker::ADD;
		mk.scale.x = 0.01;
		mk.scale.y = 0.1;
		mk.scale.z = 0.1;
		mk.color.a = 0.3;
		mk.color.r = 0.0;
		mk.color.g = 1.0;
		mk.color.b = 0.0;
		mk.pose.position.x = pc->points[i].x;
		mk.pose.position.y = pc->points[i].y;
		mk.pose.position.z = pc->points[i].z;
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
		normal_arrows.markers.push_back(mk);
    }
    // pub_mk.publish(mk);
    pub_mk.publish(normal_arrows);
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
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud");

// 	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 10, 0.5, "normals");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");

// 	viewer.spinOnce();
// }
int main(int argc, char** argv)
{
	std::cout << "Normal Estimate!!!" << std::endl;
	ros::init(argc, argv, "normal_estimater");
	NormalEstimater normal_estimater;
	ros::spin();
}