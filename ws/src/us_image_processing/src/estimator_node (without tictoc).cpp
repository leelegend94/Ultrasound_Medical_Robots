#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#include <us_image_processing/VesselState.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <eigen3/Eigen/Dense>

#include <boost/circular_buffer.hpp>
#include <mutex>

#include <nlopt.hpp>
#include <math.h> /* tanh, log */
#include <limits>
#include <random>

// #define SIM

const double PC_SCALING = 100;

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
		double r_d;
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
			//squared dist
			grad[0] += (cp2*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
			grad[1] += (cp1*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;

			cost += pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0);
		}
		grad[0] += 0.5*(n1*2.0-n_d1*2.0);
		grad[1] += 0.5*(n2*2.0-n_d2*2.0);
		//ROS_INFO_STREAM("\ncurrent n: "<<n1<<", "<<n2<<", 1.0"<<"\ngrad0: "<<grad[0]<<", grad1: "<<grad[1]);

		cost += 0.5*(pow(n1-n_d1,2.0)+pow(n2-n_d2,2.0));
		//ROS_INFO_STREAM("grad: "<<grad[0]<<','<<grad[1]);
		return cost;
	}

	static double costFunc_cf(const std::vector<double> &x, std::vector<double> &grad, void* data){
		data_struct *data_ptr = reinterpret_cast<data_struct *>(data);
		double cp1,cp2,cp3,n1,n2,n3,n_d1,n_d2, r, r_d, epsilon;

		int num_pc = data_ptr->pc_ptr->width;

		double mu = 1.0;

		grad.assign({0.0, 0.0, 0.0, 0.0});
		double cost = 0;

		n1 = x[0]; n2 = x[1]; n3 = 1;
		n_d1 = data_ptr->n_d[0];
		n_d2 = data_ptr->n_d[1];
		
		r = x[2];
		r_d = data_ptr->r_d;
		epsilon = x[3];

		for(int i=0; i<num_pc; i++){
			cp1 = data_ptr->pc_ptr->points[i].x;
			cp2 = data_ptr->pc_ptr->points[i].y;
			cp3 = data_ptr->pc_ptr->points[i].z;

			//TODO: optimize this block by using matlab ccode
			//cylinder fitting
			cost += pow(pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-r*r,2.0);

			grad[0] += (pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-r*r)*((cp2*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0)*-2.0;
			grad[1] += (pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-r*r)*((cp1*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0)*-2.0;
			grad[2] += r*(pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-r*r)*-4.0;
		}

		cost /= num_pc;
		grad[0] /=  num_pc;
		grad[1] /=  num_pc;
		grad[2] /=  num_pc;
		grad[3] = 2*mu*epsilon;

		cost += pow(epsilon,2.0);

		//ROS_INFO_STREAM("\ncurrent x: "<<n1<<", "<<n2<<", "<<r<<"\ngrad0: "<<grad[0]<<", grad1: "<<grad[1]<<", grad2: "<<grad[2]);
		//std::cout<<"total: "<< num_pc <<std::endl;

		// cost += 0.5*(pow(n1-n_d1,2.0)+pow(n2-n_d2,2.0)+pow(r-r_d,2.0));
		// grad[0] += 1.0*(n1-n_d1);
		// grad[1] += 1.0*(n2-n_d2);
		// grad[2] += 1.0*(r-r_d);

		cost += 0.5*(pow(atan(n2/n1)-atan(n_d2/n_d1),2.0)+pow(r-r_d,2.0));
		grad[0] +=  (atan(n2/n1)-atan(n_d2/n_d1))/(1+pow(n2/n1,2.0))/n1;
		grad[1] += -(atan(n2/n1)-atan(n_d2/n_d1))/(1+pow(n2/n1,2.0))*n2/pow(n1,2.0);
		grad[2] += (r-r_d);

		grad[2] = 0; //r is not updated

		//ROS_INFO_STREAM("grad: "<<grad[0]<<','<<grad[1]);
		return cost;
	}
/*
	static double costFunc2_(const std::vector<double> &n, std::vector<double> &grad, void* data){
		data_struct *data_ptr = reinterpret_cast<data_struct *>(data);
		double cp1,cp2,cp3,n1,n2,n3,n_d1,n_d2,n_d3;

		int num_pc = data_ptr->pc_ptr->width;

		//double a = 0.0;

		grad.assign({0.0, 0.0});
		double cost = 0;

		n1 = n[0]; n2 = n[1]; n3 = 1;
		n_d1 = data_ptr->n_d[0];
		n_d2 = data_ptr->n_d[1];
		n_d3 = 1;
		
		Eigen::VectorXd dist;
		Eigen::Matrix<double, 2, Dynamic> grad_dist;

		dist.resize(num_pc);
		grad_dist.resize(2,num_pc);

		for(int i=0; i<num_pc; i++){
			cp1 = data_ptr->pc_ptr->points[i].x;
			cp2 = data_ptr->pc_ptr->points[i].y;
			cp3 = data_ptr->pc_ptr->points[i].z;

			dist(i) = sqrt(pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0));

			grad_dist(0,i) = 1.0/sqrt(pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0))*((cp2*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0)*(-1.0/2.0);
			grad_dist(1,i) = 1.0/sqrt(pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0))*((cp1*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0+fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0)*(-1.0/2.0);
			
			// grad[0] += (cp2*(cp1*n2-cp2*n1)*-2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp1-cp3*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n1)*((n1/fabs(n1)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n1)*((n1/fabs(n1)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
			// grad[1] += (cp1*(cp1*n2-cp2*n1)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-(cp3*(cp2-cp3*n2)*2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)-fabs(n2)*((n2/fabs(n2)))*pow(cp1*n2-cp2*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp1-cp3*n1,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0-fabs(n2)*((n2/fabs(n2)))*pow(cp2-cp3*n2,2.0)*1.0/pow(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0,2.0)*2.0;
		
			// cost += pow(cp1*n2-cp2*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp1-cp3*n1,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0)+pow(cp2-cp3*n2,2.0)/(pow(fabs(n1),2.0)+pow(fabs(n2),2.0)+1.0);
		
		}

		double mu_dist = dist.mean();
		Eigen::Vector2d mu_grad_dist = grad_dist.rowwise().mean();

		for(int i=0; i<num_pc; i++){
			grad[0] += (dist(i)-mu_dist)*(grad_dist(0,i)-mu_grad_dist(0));
			grad[1] += (dist(i)-mu_dist)*(grad_dist(1,i)-mu_grad_dist(1));
		}

		ROS_INFO_STREAM("grad0: "<<grad[0]<<", grad1: "<<grad[1]);
		grad[0] *= 2/num_pc;
		grad[1] *= 2/num_pc;

		//grad[0] += n1*2.0-n_d1*2.0;
		//grad[1] += n2*2.0-n_d2*2.0;

		VectorXd centered = dist-mu_dist*VectorXd::Ones(dist.rows());
		//ROS_INFO_STREAM("mean: "<<mu_dist);
		//ROS_INFO_STREAM("???: "<<(centered.adjoint() * centered)/ num_pc);
		auto variance = ((centered.adjoint() * centered) / num_pc)(0);
		ROS_INFO_STREAM("var: "<<variance);
		//cost = 1000*variance + pow(n1-n_d1,2.0)+pow(n2-n_d2,2.0);
		cost = 1000000*variance;
		//ROS_INFO_STREAM("grad: "<<grad[0]<<','<<grad[1]);
		return cost;
	}
	*/
	static double fcons(const std::vector<double> &x, std::vector<double> &grad, void* data){
		grad[0] = 0; //n1
		grad[1] = 0; //n2
		grad[2] = 1.0; //r
		grad[3] = -1.0; //epsilon

		return x[2] - 0.02*PC_SCALING - x[3];
	}
public:
	Optimizer(unsigned dim):
	opt_(nlopt::LD_SLSQP, dim)
	{
		dim_ = dim;
		opt_.set_xtol_abs(1e-8);
		//opt_.set_x_weights({1.0, 1.0, 1.0, 0.0});

		opt_.add_inequality_constraint(fcons, NULL, 1e-6);

		double MAX_VAL = std::numeric_limits<double>::max();
		//double MIN_VAL = std::numeric_limits<double>::min();

		opt_.set_lower_bounds({-MAX_VAL,  0.0, 0.1, 0.0}); //(n1,n2,r,epsilon)
		opt_.set_upper_bounds({ MAX_VAL,  MAX_VAL, 5.0, 2.0}); //(n1,n2,r,epsilon)

		opt_.set_maxtime(1.0);
	}

	void set_data(pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<double> n_d){
		data_.pc_ptr = &pc;
		data_.n_d = n_d;
		opt_.set_min_objective(costFunc_cf, &data_);
	}

	// void opt_init_r(pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<double> n_d){
	// 	data_.pc_ptr = &pc;
	// 	data_.n_d = n_d;
	// 	opt_.set_min_objective(costFunc_cf, &data_);
	// }

	double optimize(std::vector<double> &n){
		double min_cost;
		try{
			nlopt::result result = opt_.optimize(n, min_cost);
			ROS_INFO_STREAM("\nMinimum cost: " << min_cost<<"\ncurrent n: "<<n[0]<<", "<<n[1]<<", 1.0 r = "<<n[2]<<"\nepsilon = "<<n[3]);
		}
		catch (std::exception &e){
			ROS_WARN_STREAM("Failed to find solution in optimization problem: " << e.what());
			ROS_WARN_STREAM("\nMinimum cost: " << min_cost<<"\ncurrent n: "<<n[0]<<", "<<n[1]<<", 1.0 r = "<<n[2]<<"\nepsilon = "<<n[3]);
			//ros::shutdown();
			//return {0};
		}

		//if(min_cost>100) throw "MSE too large, optimization failed.";

		return min_cost;
	}

};

void rescaling(std::vector<double> &n){
	double length = sqrt(pow(n[0],2.0)+pow(n[1],2.0));
	double factor = 100*tanh(length);
	n[0] = n[0]/length*factor;
	n[1] = n[1]/length*factor;
	return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "estimator");
	ros::NodeHandle nh;

	PointCloudBuffer node(nh);

	ros::AsyncSpinner spinner(0);
	spinner.start();
	// Optimizer optim(2);
	Optimizer optim(4);

	ros::Publisher pub_vesselState = nh.advertise<us_image_processing::VesselState>("/vessel_state",10);
	
	//debug
	ros::Publisher pub_debug_pc2 = nh.advertise<sensor_msgs::PointCloud2>("debug_vessel_pc2_trans",10);
	//

	//wait until the buffer is full
	while(node.ringBuf.size() != node.ringBuf.capacity()){
		ROS_INFO_STREAM("Loading buffer: "<<node.ringBuf.size()<<"/"<<node.ringBuf.capacity());
		ros::Duration(0.1).sleep();
	}
	
	//double MIN_VAL = std::numeric_limits<double>::min();
	// std::vector<double> n = {-10,10};
	// std::vector<double> n_ = {-10,10};
	std::vector<double> n = {-10,10,0.5,0.0};
	//std::vector<double> n_ = {-10,10,0.015,1e-10};

	ros::Time ti,tf;

	bool INIT = true;

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,2.0);

	std::srand(std::time(nullptr));

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

		//Transform point cloulds in the buffer to origion.
		Eigen::Vector4d centroid_buf;
		pcl::compute3DCentroid(vessel_points, centroid_buf);

		Eigen::Matrix4d transform_cp = Eigen::Matrix4d::Identity();
		transform_cp(0,3) = -centroid_buf(0);
		transform_cp(1,3) = -centroid_buf(1);
		transform_cp(2,3) = -centroid_buf(2);
		//transform_cp(3,3) = 0.001;

		//ROS_INFO_STREAM("centroid: "<<centroid(0)<<','<<centroid(1)<<','<<centroid(2));
		pcl::transformPointCloud(vessel_points, vessel_points, transform_cp);

		for (int i = 0; i < vessel_points.points.size(); i++)
		{
		    pcl::PointXYZ pt = vessel_points.points[i];
			vessel_points.points[i] = pcl::PointXYZ(pt.x * PC_SCALING, pt.y * PC_SCALING, pt.z * PC_SCALING);
		}

		optim.set_data(vessel_points, n);

		
		// try{
		// 	result = optim.optimize(n);
		// }catch(const char* msg){
		// 	std::cout << msg << std::endl;
		// 	n = {-10,10,0.015,1e-10};
		// 	//n_ = {-10,10,0.015,1e-10};
		// 	std::cout << "reset!" << std::endl;
		// }

		int cntr = 1;
		n[3] = 0.0;
		while(optim.optimize(n)>500){
			if(cntr>1) break;
			ROS_WARN_STREAM("MSE too large, optimization failed. Trying to use another initial value");
			n[0] = 200*(rand()/double(RAND_MAX)-0.5); //rand(-100 ~ 100)
			n[1] = 100*(rand()/double(RAND_MAX)); //rand(0 ~ 100)
			n[2] = 0.5;
			n[3] = 0.0;
			std::cout << "random init. val: " <<n[0]<<", "<< n[1]<< std::endl;
			cntr ++;
		}
		
		// if(n[1]<0){
		// 	n[0] = -n[0];
		// 	n[1] = -n[1];
		// 	n[2] = -n[2];
		// }

		//debug
		sensor_msgs::PointCloud2 msg_debug_pc2;
		pcl::toROSMsg(vessel_points, msg_debug_pc2);
		msg_debug_pc2.header.frame_id = BASE_LINK;
		pub_debug_pc2.publish(msg_debug_pc2);
		//

		//send vessel state
		us_image_processing::VesselState msg_vesselState;
		msg_vesselState.header.stamp = ros::Time::now();
		msg_vesselState.header.frame_id = BASE_LINK;
		msg_vesselState.centroid.x = centroid_curr_pos(0);
		msg_vesselState.centroid.y = centroid_curr_pos(1);
		msg_vesselState.centroid.z = centroid_curr_pos(2);
		msg_vesselState.direction.x = n[0];
		msg_vesselState.direction.y = n[1];
		msg_vesselState.direction.z = 1;

		if(INIT) pub_vesselState.publish(msg_vesselState);

		std::cout << "!!! " <<n[0]<< std::endl;
		rescaling(n);
		n[0] += distribution(generator);
		n[1] += distribution(generator);
		std::cout << "???" <<n[0]<< std::endl;

		tf = ros::Time::now();
		//ROS_INFO_STREAM("est. rate: "<<1/(tf-ti).toSec()<<"Hz");
		INIT = true;
		//ros::shutdown();
	}

	//ros::waitForShutdown(); //dummy spin
	return 0;
}