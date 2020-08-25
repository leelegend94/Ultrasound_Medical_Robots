#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#include <us_image_processing/VesselState.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/circular_buffer.hpp>
#include <mutex>

#include <nlopt.hpp>
#include <math.h>

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
	ringBuf(20) //pc2 send at about 15Hz 
	{
		sub_pc2_ = nh_.subscribe("us_vessel_pointcloud", 10, &PointCloudBuffer::update,this);
	}

	void send(std::vector<double> &n);
};

void PointCloudBuffer::update(const sensor_msgs::PointCloud2::ConstPtr& msg){
	ROS_INFO_STREAM("update..");
	//sensor_msgs::PointCloud2 pc_msg_base;
	pcl::PointCloud< pcl::PointXYZ > pc_ee, pc_base;

	pcl::fromROSMsg(*msg, pc_ee);
	
	geometry_msgs::TransformStamped Te0;
	try{
        Te0 = tf_buf.lookupTransform("iiwa_link_0", "iiwa_link_ee", ros::Time(0));
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

	/*
	sensor_msgs::PointCloud2 pc_msg_base;
	pcl::PointCloud< pcl::PointXYZ > pc_base;

	pcl_ros::transformPointCloud("iiwa_link_0", *msg, pc_msg_base, tf_listener);
	//pcl_ros::transformPointCloud(*msg, pc_msg_base, Te0);
	pcl::fromROSMsg(pc_msg_base, pc_base);
	//pcl::fromROSMsg(*msg, pc_base);
	mtx.lock();
	ringBuf.push_back(pc_base);
	mtx.unlock();
	*/
	return;
}



class Optimizer{
private:
	int dim_;
	nlopt::opt opt_;

	struct data_struct{
		pcl::PointCloud< pcl::PointXYZ > *pc_ptr;
		std::vector<double> n_d;
	} data_;

	static double costFunc_(const std::vector<double> &n, std::vector<double> &grad, void* data){
		data_struct *data_ptr = reinterpret_cast<data_struct *>(data);
		double cp1,cp2,cp3,n1,n2,n3,n_d1,n_d2,n_d3;

		int num_pc = data_ptr->pc_ptr->width;

		double a = 0.0;

		grad.assign({0.0, 0.0});
		double cost = 0;

		n1 = n[0]; n2 = n[1]; n3 = 1;
		n_d1 = data_ptr->n_d[0];
		n_d2 = data_ptr->n_d[1];
		n_d3 = 1;
	
		for(int i=0; i<num_pc; i++){
			cp1 = data_ptr->pc_ptr->points[i].x;
			cp2 = data_ptr->pc_ptr->points[i].y;
			cp3 = data_ptr->pc_ptr->points[i].z;

			//TODO: optimize this block by using matlab ccode
			/*
			grad[0] += (cp2*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
			grad[1] += (cp1*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;

			cost += pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0);
			*/
			grad[0] += (cp2*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
			grad[1] += (cp1*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
		
			cost += pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0);
		}
		grad[0] += n1*2.0-n_d1*2.0;
		grad[1] += n2*2.0-n_d2*2.0;

		cost += pow(n1-n_d1,2.0)+pow(n2-n_d2,2.0);
		//ROS_INFO_STREAM("grad: "<<grad[0]<<','<<grad[1]);
		return cost;
	}
public:
	Optimizer(unsigned dim):
	opt_(nlopt::LD_SLSQP, dim)
	{
		dim_ = dim;
		opt_.set_xtol_rel(1e-4);
	}

	void set_data(pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<double> n_d){
		data_.pc_ptr = &pc;
		data_.n_d = n_d;
		opt_.set_min_objective(costFunc_, &data_);
	}

	std::vector<double> optimize(std::vector<double> n_init){
		double min_cost;
		try{
			nlopt::result result = opt_.optimize(n_init, min_cost);
			ROS_INFO_STREAM("Minimum cost: " << min_cost);
		}
		catch (std::exception &e){
			ROS_ERROR_STREAM("Failed to find solution in optimization problem: " << e.what());
			ros::shutdown();
			return {0};
		}
		return n_init;
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "estimator");
	ros::NodeHandle nh;

	PointCloudBuffer node(nh);

	ros::AsyncSpinner spinner(0);
	spinner.start();

	Optimizer optim(2);

	ros::Publisher pub_vesselState = nh.advertise<us_image_processing::VesselState>("/vessel_state",10);
	
	//debug
	ros::Publisher pub_debug_pc2 = nh.advertise<sensor_msgs::PointCloud2>("debug_vessel_pc2_trans",10);
	//

	//wait until the buffer is full
	while(node.ringBuf.size() != node.ringBuf.capacity()){
		ROS_INFO_STREAM("Loading buffer: "<<node.ringBuf.size()<<"/"<<node.ringBuf.capacity());
	}
	
	std::vector<double> n = {-10,10};
	std::vector<double> n_ = {-10,10};

	ros::Time ti,tf;

	while(ros::ok()){
		ti = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ> vessel_points;

		node.mtx.lock();
		for(int i=0;i<node.ringBuf.capacity();i++){
			vessel_points += node.ringBuf[i];
		}
		node.mtx.unlock();

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(vessel_points, centroid);

		Eigen::Matrix4f transform_cp = Eigen::Matrix4f::Identity();
		transform_cp(0,3) = -centroid(0);
		transform_cp(1,3) = -centroid(1);
		transform_cp(2,3) = -centroid(2);

		ROS_INFO_STREAM("centroid: "<<centroid(0)<<','<<centroid(1)<<','<<centroid(2));
		pcl::transformPointCloud(vessel_points, vessel_points, transform_cp);

		//debug
		sensor_msgs::PointCloud2 msg_debug_pc2;
		pcl::toROSMsg(vessel_points, msg_debug_pc2);
		msg_debug_pc2.header.frame_id = "iiwa_link_0";
		pub_debug_pc2.publish(msg_debug_pc2);
		//

		optim.set_data(vessel_points, n_);
		n = optim.optimize(n_);

		//send vessel state
		us_image_processing::VesselState msg_vesselState;
		msg_vesselState.header.stamp = ros::Time::now();
		msg_vesselState.header.frame_id = "iiwa_link_0";
		msg_vesselState.centroid.x = centroid(0);
		msg_vesselState.centroid.y = centroid(1);
		msg_vesselState.centroid.z = centroid(2);
		msg_vesselState.direction.x = n[0];
		msg_vesselState.direction.y = n[1];
		msg_vesselState.direction.z = 1;
		pub_vesselState.publish(msg_vesselState);

		n_ = n;

		tf = ros::Time::now();
		//ROS_INFO_STREAM("est. rate: "<<1/(tf-ti).toSec()<<"Hz");
	}

	//ros::waitForShutdown(); //dummy spin
	return 0;
}