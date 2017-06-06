#include <math.h>

// #include <laser_mapping/common.h>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#define RAD_TO_DEG 180.0/3.141592
#define DEG_TO_RAD 3.141592/180.0

pcl::PointCloud<pcl::PointXYZ>::Ptr pointList(new pcl::PointCloud<pcl::PointXYZ>());

const float angularRes = 360.0/ (30000.0/(16.0));
const float verticalRes = 2.0;
const float distance = 100.0;

const float rotRes = 360.0/(10.0 * 2);

int main(int argc, char** argv){

	ros::init(argc,argv,"rot_test");
	ros::NodeHandle nh;
	ros::Publisher pubLaser = nh.advertise<sensor_msgs::PointCloud2>
								("laserPoints",2);
	ros::Publisher pubRot = nh.advertise<sensor_msgs::PointCloud2>
								("laserRotate",2);

	int numPointsChannel = (int)360/angularRes;
	for(int j = -8; j < 8; j++)
	{

		for(int i = 0 ; i < numPointsChannel ; i++ )
		{ 
			pcl::PointXYZ pt;
			pt.x = distance * cos(verticalRes * j * DEG_TO_RAD)* cos( i * angularRes * DEG_TO_RAD );
			pt.y = distance * cos(verticalRes * j * DEG_TO_RAD)* sin( i * angularRes * DEG_TO_RAD );
			pt.z = distance * sin(verticalRes * j * DEG_TO_RAD);;
			pointList -> push_back(pt);
		}
	}

	ros::Rate rate(10);
	bool status = ros::ok();

	int rotationStep = 0;
	while(status){
		ros::spinOnce();
		
		Eigen::Affine3f transformRotate = Eigen::Affine3f::Identity();

		transformRotate.rotate(Eigen::AngleAxisf ( rotationStep* rotRes * DEG_TO_RAD,Eigen::Vector3f::UnitX()));
		// transformRotate.rotate(Eigen::Quaternionf (1,0,0,0));
		rotationStep++;
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*pointList,*transformedCloud,transformRotate);


		sensor_msgs::PointCloud2 laserPointsMsg;
		pcl::toROSMsg(*transformedCloud,laserPointsMsg);
		laserPointsMsg.header.stamp = ros::Time().now();
		laserPointsMsg.header.frame_id = "/velodyne";
		pubLaser.publish(laserPointsMsg);

		status = ros::ok();
		rate.sleep();
	}				

	return 0;
}