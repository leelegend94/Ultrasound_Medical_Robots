#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>

#include <boost/circular_buffer.hpp>
#include <mutex>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #define SIM

const std::string BASE_LINK = "iiwa_link_0";

#ifdef SIM  
const std::string EE_LINK = "iiwa_link_ee";
#else
const std::string EE_LINK = "cephalinear_link_ee";
#endif

using namespace Eigen;

class PointCloudBuffer{
private:
	ros::NodeHandle nh_;

	ros::Subscriber sub_pc2_;

	//tf2
    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener;

	void update(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
	boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>> ringBuf;
	std::mutex mtx;

	PointCloudBuffer(ros::NodeHandle nh):
	nh_(nh),
	tf_listener(tf_buf),
	ringBuf(10) //pc2 send at about 15Hz 
	{
		sub_pc2_ = nh_.subscribe("us_vessel_pointcloud", 10, &PointCloudBuffer::update,this);
	}

	void send(std::vector<double> &n);
};

void PointCloudBuffer::update(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//ROS_INFO_STREAM("update..");
	//sensor_msgs::PointCloud2 pc_msg_base;
	pcl::PointCloud< pcl::PointXYZ > pc_ee, pc_base;

	pcl::fromROSMsg(*msg, pc_ee);
	
	geometry_msgs::TransformStamped Te0;
	try{
        Te0 = tf_buf.lookupTransform(BASE_LINK, EE_LINK, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
	
	pcl_ros::transformPointCloud(pc_ee, pc_base, Te0.transform);
	//pcl::fromROSMsg(pc_msg_base, pc_base);
	//pcl::fromROSMsg(*msg, pc_base);
	mtx.lock();
	ringBuf.push_back(pc_base);
	mtx.unlock();

	return;
}



int main(int argc, char** argv){
	ros::init(argc, argv, "estimator");
	ros::NodeHandle nh;

	PointCloudBuffer node(nh);

	

	// ros::Publisher pub_vesselState = nh.advertise<us_image_processing::VesselState>("/vessel_state",10);
	ros::Publisher pub_vesselState = nh.advertise<pcl_msgs::ModelCoefficients>("/vessel_state",10);
	
	//debug
	ros::Publisher pub_debug_pc2 = nh.advertise<sensor_msgs::PointCloud2>("debug_vessel_pc2_trans",10);
	//

	//wait until the buffer is full
	Eigen::Vector3d y_axis;

	geometry_msgs::TransformStamped Te0,Te0_vct;
	tf2_ros::Buffer tf_buf;
	tf2_ros::TransformListener tf_listener(tf_buf);
	ros::AsyncSpinner spinner(0);
	spinner.start();
	y_axis << 0.0, 1.0, 0.0; 

	while(node.ringBuf.size() != node.ringBuf.capacity()){

		try{
	        Te0 = tf_buf.lookupTransform(BASE_LINK, EE_LINK, ros::Time(0));
	        Te0_vct = Te0;
	        Te0_vct.transform.translation.x=0;
	        Te0_vct.transform.translation.y=0;
	        Te0_vct.transform.translation.z=0;
	    }
	    catch (tf2::TransformException &ex) {
	        ROS_WARN("%s",ex.what());
	        continue;
	    }

	    tf2::Stamped<Eigen::Vector3d> stampedY(y_axis,ros::Time(),EE_LINK);
    	tf2::doTransform(stampedY, stampedY, Te0_vct);
    	
    	pcl_msgs::ModelCoefficients ros_coefficients;
    	ros_coefficients.values.push_back(Te0.transform.translation.x);
    	ros_coefficients.values.push_back(Te0.transform.translation.y);
    	ros_coefficients.values.push_back(Te0.transform.translation.z);
		ros_coefficients.values.push_back(stampedY(0));
		ros_coefficients.values.push_back(stampedY(1));
		ros_coefficients.values.push_back(stampedY(2));
		ros_coefficients.values.push_back(0);

		pub_vesselState.publish(ros_coefficients);

		ROS_INFO_STREAM("Loading buffer: "<<node.ringBuf.size()<<"/"<<node.ringBuf.capacity());
		ros::Duration(0.05).sleep();

	}
	//ros::shutdown();
	ros::Time ti,tf;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

	while(ros::ok()){
		ti = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ> vessel_points;
		Eigen::Vector4d centroid_curr_pos;

		node.mtx.lock();
		for(int i=0;i<node.ringBuf.capacity();i++){
			vessel_points += node.ringBuf[i];
		}
		pcl::compute3DCentroid(node.ringBuf[node.ringBuf.capacity()], centroid_curr_pos);
		node.mtx.unlock();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vessel = vessel_points.makeShared();
/*
		//Transform point cloulds in the buffer to origion.
		Eigen::Vector4d centroid_buf;
		pcl::compute3DCentroid(vessel_points, centroid_buf);

		Eigen::Matrix4d transform_cp = Eigen::Matrix4d::Identity();
		transform_cp(0,3) = -centroid_buf(0);
		transform_cp(1,3) = -centroid_buf(1);
		transform_cp(2,3) = -centroid_buf(2);

		//ROS_INFO_STREAM("centroid: "<<centroid(0)<<','<<centroid(1)<<','<<centroid(2));
		pcl::transformPointCloud(vessel_points, vessel_points, transform_cp);

		for (int i = 0; i < vessel_points.points.size(); i++)
		{
		    pcl::PointXYZ pt = vessel_points.points[i];
			vessel_points.points[i] = pcl::PointXYZ(pt.x * PC_SCALING, pt.y * PC_SCALING, pt.z * PC_SCALING);
		}
*/
		// Estimate point normals
		ne.setSearchMethod (tree);
		ne.setInputCloud (cloud_vessel);
		ne.setKSearch (50);
		ne.compute (*cloud_normals);

		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_CYLINDER);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.0);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.05);
		seg.setRadiusLimits (0.005, 0.04);
		seg.setInputCloud (cloud_vessel);
		seg.setInputNormals (cloud_normals);

		 // Obtain the cylinder inliers and coefficients
		seg.segment (*inliers_cylinder, *coefficients_cylinder);
		std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
		
		// // Write the cylinder inliers to disk
		// extract.setInputCloud (cloud_filtered2);
		// extract.setIndices (inliers_cylinder);
		// extract.setNegative (false);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
		// extract.filter (*cloud_cylinder);

		// //debug
		// sensor_msgs::PointCloud2 msg_debug_pc2;
		// pcl::toROSMsg(vessel_points, msg_debug_pc2);
		// msg_debug_pc2.header.frame_id = BASE_LINK;
		// pub_debug_pc2.publish(msg_debug_pc2);
		// //

		// //send vessel state
		// us_image_processing::VesselState msg_vesselState;
		// msg_vesselState.header.stamp = ros::Time::now();
		// msg_vesselState.header.frame_id = BASE_LINK;
		// msg_vesselState.centroid.x = centroid_curr_pos(0);
		// msg_vesselState.centroid.y = centroid_curr_pos(1);
		// msg_vesselState.centroid.z = centroid_curr_pos(2);
		// msg_vesselState.direction.x = n[0];
		// msg_vesselState.direction.y = n[1];
		// msg_vesselState.direction.z = 1;
		// pub_vesselState.publish(msg_vesselState);

		coefficients_cylinder->values[0] = centroid_curr_pos[0];
		coefficients_cylinder->values[1] = centroid_curr_pos[1];
		coefficients_cylinder->values[2] = centroid_curr_pos[2];

		if(coefficients_cylinder->values[4]<0){
			coefficients_cylinder->values[3] = -coefficients_cylinder->values[3];
			coefficients_cylinder->values[4] = -coefficients_cylinder->values[4];
			coefficients_cylinder->values[5] = -coefficients_cylinder->values[5];
		}

		pcl_msgs::ModelCoefficients ros_coefficients;
		pcl_conversions::fromPCL(*coefficients_cylinder, ros_coefficients);
		pub_vesselState.publish(ros_coefficients);

		tf = ros::Time::now();
		//ROS_INFO_STREAM("est. rate: "<<1/(tf-ti).toSec()<<"Hz");
		//ros::shutdown();
	}

	return 0;
}