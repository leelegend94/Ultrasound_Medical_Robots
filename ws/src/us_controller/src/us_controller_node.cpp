#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <us_image_processing/VesselState.h>

#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <iiwa_msgs/ControlMode.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_msgs/CartesianImpedanceControlMode.h>
#include <iiwa_msgs/CartesianPose.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <pcl_msgs/ModelCoefficients.h>

#define SIM
//#define USE_PCL

const std::string BASE_LINK = "iiwa_link_0";

#ifdef SIM  
const std::string EE_LINK = "iiwa_link_ee";
#else
const std::string EE_LINK = "cephalinear_link_ee";
#endif

class USController{
private:
    ros::NodeHandle nh_;
    
    ros::ServiceClient client_config_;
    ros::Subscriber sub_eePose_;
    ros::Subscriber sub_eeFT_;
    ros::Publisher pub_desiredPose_;
    ros::Subscriber sub_vesselState_;

    ros::ServiceServer srv_start_;

    //controller config
    std::vector<double> cartesian_stiffness_;
    std::vector<double> cartesian_damping_;
    double nullspace_stiffness_;
    double nullspace_damping_;

    double marching_vel_;

    //robot states
    geometry_msgs::Pose curr_pose;
    std::vector<double> force_;
    std::vector<double> torque_;

    //visual data
    //tf2::Stamped<Eigen::Vector3d> centroid_d;

    ros::Time ti_,tf_;

    //tf2
    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener;

    //status
    bool INIT;

    //debug
    //ros::Publisher pub_debug_centroid_;
    //
    /*------------------------------*/
    /*          Call Backs          */
    /*------------------------------*/
    #ifdef SIM
    void updatePose(const geometry_msgs::PoseStampedConstPtr& msg);
    #else
    void updatePose(const iiwa_msgs::CartesianPoseConstPtr& msg);
    #endif

    void updateWrench(const geometry_msgs::WrenchStampedConstPtr& msg);
    
#ifndef USE_PCL
    void plan(const us_image_processing::VesselState::ConstPtr& msg);
#else
    void plan(const pcl_msgs::ModelCoefficients::ConstPtr& msg);
#endif
    bool scan_init(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

    /*------------------------------*/
    /*       helper functions       */
    /*------------------------------*/
    void fill_msg(iiwa_msgs::CartesianQuantity& msg, const std::vector<double>& arg_list);
    void controller_init(void);
    void init_pos(void);

public:
    USController(ros::NodeHandle nh): 
    nh_(nh),
    tf_listener(tf_buf),
    force_({0,0,0}), torque_({0,0,0})
    {
        
        INIT = false;
        nh_.param<std::vector<double>>("/controller/cartesian_stiffness", cartesian_stiffness_, {20,20,20,20,20,20});
        //ROS_INFO_STREAM("cartesian stiffness: "<<cartesian_stiffness_[0]);
        nh_.param<std::vector<double>>("/controller/cartesian_damping", cartesian_damping_, {0.2,0.2,0.2,0.2,0.2,0.2});
        nh_.param<double>("/controller/nullspace_stiffness", nullspace_stiffness_, 10);
        nh_.param<double>("/controller/nullspace_damping", nullspace_damping_, 0.2);
        nh_.param<double>("/scan/marching_vel", marching_vel_, 0.2);

        client_config_ = nh_.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
        controller_init();

        sub_eePose_ = nh_.subscribe("/iiwa/state/CartesianPose",10,&USController::updatePose,this);
        sub_eeFT_ = nh_.subscribe("/iiwa/state/CartesianWrench",10,&USController::updateWrench,this);
#ifdef SIM
        pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
#else
        pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPoseLin",10);
#endif
        sub_vesselState_ = nh_.subscribe("/vessel_state",10,&USController::plan,this);
        
        srv_start_ = nh_.advertiseService("/start", &USController::scan_init, this);
        ROS_INFO_STREAM("Controller Class initialized");

        //debug
        //pub_debug_centroid_ = nh_.advertise<geometry_msgs::PointStamped>("/debug",10);
        //
        
    }
    
    ~USController(){}

};

bool USController::scan_init(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    if(req.data){
        init_pos();
        //debug
        // //pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
        // while(1){
        //     geometry_msgs::PoseStamped command;
        //     command.header.frame_id = BASE_LINK;

        //     std::vector<double> init_pose;
        //     nh_.param<std::vector<double>>("/init_pose", init_pose, {0.6, 0.1, 0, 0, 1, 0, 0});

        //     command.pose.position.x = init_pose[0];
        //     command.pose.position.y = init_pose[1];
        //     command.pose.position.z = init_pose[2];
        //     command.pose.orientation.x = init_pose[3];
        //     command.pose.orientation.y = init_pose[4];
        //     command.pose.orientation.z = init_pose[5];
        //     command.pose.orientation.w = init_pose[6];
        //     command.pose.position.y = 0.1+0.05*std::sin(ros::Time::now().toSec());
        //     pub_desiredPose_.publish(command);
        // }
        //

        INIT = true;
    }else{
        INIT = true;
    }
    res.success = true;
    return true;
}

void USController::init_pos(void){
    geometry_msgs::PoseStamped command;
    command.header.frame_id = BASE_LINK;

    std::vector<double> init_pose;
    nh_.param<std::vector<double>>("/init_pose", init_pose, {0.0, -0.6, 0.4, 1, 0, 0, 0});

    command.pose.position.x = init_pose[0];
    command.pose.position.y = init_pose[1];
    command.pose.position.z = init_pose[2];
    command.pose.orientation.x = init_pose[3];
    command.pose.orientation.y = init_pose[4];
    command.pose.orientation.z = init_pose[5];
    command.pose.orientation.w = init_pose[6];

    pub_desiredPose_.publish(command);
    
    while(ros::ok() && (std::abs(curr_pose.position.x-init_pose[0])+std::abs(curr_pose.position.y-init_pose[1])+std::abs(curr_pose.position.z-init_pose[2]) > 0.1)){
        pub_desiredPose_.publish(command);
    }
    //while(tf_==ti_);
    
    return;
}

#ifndef USE_PCL 
void USController::plan(const us_image_processing::VesselState::ConstPtr& msg)
#else
void USController::plan(const pcl_msgs::ModelCoefficients::ConstPtr& msg)
#endif
{
    
    tf_ = ros::Time::now();
    //double dt = 1e-9*(tf_.nsec-ti_.nsec);
    double dt = (tf_-ti_).toSec();
    if(dt>1.5){
        ROS_WARN_STREAM("too large dt: "<<dt);
        dt = 0;
        
    }

    if(!INIT){
        ROS_WARN_STREAM("waiting for initialization");
        ti_ = ros::Time::now();
        return;
    }
    geometry_msgs::TransformStamped Te0,Te0_vct;
    Eigen::Vector3d n, centroid; //vessel axial vector, centroid of the vessel in the current image wrt base_link
    
    try{
        Te0 = tf_buf.lookupTransform(BASE_LINK, EE_LINK, ros::Time(0));
        Te0_vct = Te0;
        Te0_vct.transform.translation.x=0;
        Te0_vct.transform.translation.y=0;
        Te0_vct.transform.translation.z=0;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        //continue;
    }
    /*
    centroid << msg->pose.x, 0.0, msg->pose.y;
    //centroid = Te0*centroid;
    tf2::Stamped<Eigen::Vector3d> stampedCentroid(centroid,ros::Time(),EE_LINK);
    tf2::doTransform(stampedCentroid, stampedCentroid, Te0);
    //debug
    geometry_msgs::PointStamped msg_debug_centroid;
    msg_debug_centroid.header.frame_id = stampedCentroid.frame_id_;
    msg_debug_centroid.header.stamp = stampedCentroid.stamp_;
    msg_debug_centroid.point.x = stampedCentroid[0];
    msg_debug_centroid.point.y = stampedCentroid[1];
    msg_debug_centroid.point.z = stampedCentroid[2];
    pub_debug_centroid_.publish(msg_debug_centroid);
    //
    */

#ifndef USE_PCL 
    //n wrt base link
    n << msg->direction.x, msg->direction.y, msg->direction.z;
    n.normalize();

    centroid << msg->centroid.x, msg->centroid.y, msg->centroid.z;
#else
    n << msg->values[3], msg->values[4], msg->values[5];
    centroid << msg->values[0], msg->values[1], msg->values[2];
#endif

    Eigen::Vector3d x_axis,y_axis,z_axis;

    // z_axis << 0.0, 0.0, 1.0; // z_ef_
    // tf2::Stamped<Eigen::Vector3d> stampedZ(z_axis,ros::Time(),EE_LINK);

    // tf2::doTransform(stampedZ, stampedZ, Te0_vct);
    // stampedZ.normalize(); //z_0_

    // x_axis = (n.cross(stampedZ)).normalized(); // x_0
    // y_axis = n; // y_0
    // z_axis = (x_axis.cross(y_axis)).normalized(); // z_0

    //////
    y_axis = n; // y_0
    y_axis[2] = 0;
    y_axis.normalized();
    z_axis << 0.0, 0.0, -1.0;
    x_axis = (y_axis.cross(z_axis)).normalized(); // x_0
    //////

    Eigen::Matrix3d pose_rotm;
    pose_rotm << x_axis, y_axis, z_axis;

    Eigen::Quaterniond pose_quat(pose_rotm);

    Eigen::Vector3d marching;
    //marching.setZero();
    //marching = marching_vel_*y_axis*dt;
    marching = marching_vel_*y_axis;
    ROS_INFO_STREAM("y: "<<y_axis(0)<<','<<y_axis(1)<<','<<y_axis(2));
    ROS_INFO_STREAM("centroid: "<<centroid(0)<<','<<centroid(1)<<','<<centroid(2));
    std::cout<<"dt: "<< std::setprecision(9)<<dt<<std::endl;
    ROS_INFO_STREAM("marching: "<<marching(0)<<','<<marching(1)<<','<<marching(2));
    
    geometry_msgs::PoseStamped command;
    command.header.frame_id = BASE_LINK;
    command.header.stamp = ros::Time::now();
    command.header.seq = msg->header.seq;
    command.pose.position.x = centroid[0] + marching[0];
    command.pose.position.y = centroid[1] + marching[1];
    command.pose.position.z = centroid[2] + marching[2];
    command.pose.orientation.x = pose_quat.x();
    command.pose.orientation.y = pose_quat.y();
    command.pose.orientation.z = pose_quat.z();
    command.pose.orientation.w = pose_quat.w();

    
    pub_desiredPose_.publish(command);
    
    //centroid_d = centroid;
    ti_ = ros::Time::now();
    return;
}

#ifdef SIM
void USController::updatePose(const geometry_msgs::PoseStampedConstPtr& msg){
    curr_pose = msg->pose;
    
    return;
}
#else
void USController::updatePose(const iiwa_msgs::CartesianPoseConstPtr& msg){
    curr_pose = msg->poseStamped.pose;
    //ROS_INFO_STREAM("updated");
    return;
}
#endif

void USController::updateWrench(const geometry_msgs::WrenchStampedConstPtr& msg){
    force_[0] = msg->wrench.force.x;
    force_[1] = msg->wrench.force.y;
    force_[2] = msg->wrench.force.z;
    torque_[0] = msg->wrench.torque.x;
    torque_[1] = msg->wrench.torque.y;
    torque_[2] = msg->wrench.torque.z;

    return;
}

void USController::controller_init(void){
    iiwa_msgs::ConfigureControlMode msg_config;
    msg_config.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
    
    iiwa_msgs::CartesianImpedanceControlMode impedance_config;
    fill_msg(impedance_config.cartesian_stiffness, cartesian_stiffness_);
    fill_msg(impedance_config.cartesian_damping, cartesian_damping_);
    impedance_config.nullspace_stiffness = nullspace_stiffness_;
    impedance_config.nullspace_damping = nullspace_damping_;

    msg_config.request.cartesian_impedance = impedance_config;
    
    if (client_config_.call(msg_config)) {
        if(!msg_config.response.success)
            ROS_ERROR_STREAM("Config failed, Java error: " << msg_config.response.error);
        else
            ROS_INFO_STREAM("SmartServo Service successfully called.");
    }
    else {
        ROS_ERROR_STREAM("Config failed - service could not be called - QUITTING NOW !");
        ros::shutdown();
    }
    
    return;
}

void USController::fill_msg(iiwa_msgs::CartesianQuantity& msg, const std::vector<double>& arg_list){
    msg.x = arg_list[0];
    msg.y = arg_list[1];
    msg.z = arg_list[2];
    msg.a = arg_list[3];
    msg.b = arg_list[4];
    msg.c = arg_list[5];
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "us_controller");
    ros::NodeHandle nh;

    USController node(nh);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();

    return 0;
}