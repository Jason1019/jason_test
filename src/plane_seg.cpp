#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <std_msgs/String.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::PointCloud<pcl::PointXYZ>::Ptr sr300Cloud(new pcl::PointCloud<pcl::PointXYZ>());


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloudMsg)
{
	//ROS_INFO("callback");
	pcl::fromROSMsg(*pointcloudMsg, *sr300Cloud);
	ROS_INFO("size : %d",(int)(sr300Cloud->size()));
	

	// pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
 //  	downSizeFilter.setLeafSize(0.005, 0.005, 0.005);
 //    downSizeFilter.setInputCloud(sr300Cloud);
 //    downSizeFilter.filter(*sr300Cloud);

}

int main(int argc, char** argv)
{

	ros::init(argc, argv,"plane_segmentation");
	ros::NodeHandle nh;

	ros::Subscriber subPointcloud = nh.subscribe<sensor_msgs::PointCloud2>
										("/camera/depth_registered/points",10,pointCloudCallback);

    ros::Publisher pubPoints = nh.advertise<sensor_msgs::PointCloud2>("/laserpoints", 2);
	ros::Publisher pubinputPoints = nh.advertise<sensor_msgs::PointCloud2>("/input_points", 2);

	pcl::SACSegmentation<pcl::PointXYZ> seg; 
	
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	
    
	ros::Rate rate(3);
	bool status = ros::ok();

	while(status){

		ros::spinOnce();
		if(sr300Cloud->size() != 0){
			// ROS_INFO("seg - start ");
			outputCloud->clear();

			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PLANE);
			
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.01);
			seg.setMaxIterations(10000);

			seg.setInputCloud(sr300Cloud);
			seg.segment(*inliers,*coeff);


			if(inliers->indices.size()==0){
				ROS_ERROR("Cannot estimate");
				return -1;
			}


			pcl::PointXYZ inlier_list; //pcl::PointXYZ input_list;


			for(size_t i = 0; i < inliers->indices.size(); i++)
			{
				inlier_list.x = sr300Cloud->points[inliers->indices[i]].x;
				inlier_list.y = sr300Cloud->points[inliers->indices[i]].y;
				inlier_list.z = sr300Cloud->points[inliers->indices[i]].z;

				outputCloud-> push_back(inlier_list);

			}

			
			sensor_msgs::PointCloud2 inputPointMsg;
			pcl::toROSMsg(*sr300Cloud, inputPointMsg);
			inputPointMsg.header.stamp = ros::Time().now();
			inputPointMsg.header.frame_id = "/camera";
			pubinputPoints.publish(inputPointMsg);


			sensor_msgs::PointCloud2 laserPointMsg;
			pcl::toROSMsg(*outputCloud, laserPointMsg);
			laserPointMsg.header.stamp = ros::Time().now();
			laserPointMsg.header.frame_id = "/camera";
			pubPoints.publish(laserPointMsg);

		}
		

		status = ros::ok();
		rate.sleep();

	}
	

	//ROS_INFO("Start plane_seg start");
	//ros::spin();

	return 0;

}