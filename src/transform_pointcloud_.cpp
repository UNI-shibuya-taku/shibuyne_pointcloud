#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


using namespace std;

class PointCloudTransform{
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Time t;
		sensor_msgs::PointCloud pc_;
		string target_frame;

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

PointCloudTransform::PointCloudTransform()
	: nhPrivate("~")
{
    std::cout << "first settings" << std::endl;
    nhPrivate.getParam("target_frame", target_frame);
		sub = nh.subscribe("/cloud", 10, &PointCloudTransform::Callback, this);
		pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/tf", 10);
	// target_frame = "centerlaser";	//変更
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
  std::cout << "------transform callback-----------" << std::endl;
  std::cout << "msg size: " << msg->data.size() << std::endl; // 8340
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
  std::cout << "[cloud] size: " << pc_.points.size() << std::endl; // 417
	
	t = msg->header.stamp;
	string source_frame = msg->header.frame_id;

	sensor_msgs::PointCloud pc_trans;
	sensor_msgs::PointCloud2 pc2_trans;

	try{
		listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
		listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
		pub.publish(pc2_trans);
	}catch (tf::TransformException& ex) {
		ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tramsform_pointcloud");
    std::cout << "start transformPointCloud" << std::endl;
	PointCloudTransform transform;
	ros::spin();

    return 0;
}

