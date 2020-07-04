#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <image_segmentation/Ellipse.h>

#include <iiwa_msgs/ControlMode.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_msgs/CartesianImpedanceControlMode.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

const std::string BASE_LINK = "iiwa_link_0";

class USController{
private:
    ros::NodeHandle nh_;
    
    ros::ServiceClient client_config_;
    ros::Subscriber sub_eePose_;
    ros::Subscriber sub_eeFT_;
    ros::Publisher pub_desiredPose_;
    ros::Subscriber sub_mask_;

    ros::ServiceServer srv_start_;

    //controller config
    std::vector<double> cartesian_stiffness_;
    std::vector<double> cartesian_damping_;
    double nullspace_stiffness_;
    double nullspace_damping_;

    double marching_vel;

    //robot states
    geometry_msgs::Pose curr_pose;
    std::vector<double> force_;
    std::vector<double> torque_;

    //visual data
    tf2::Stamped<Eigen::Vector3d> stampedCentroid_d;

    ros::Time ti_,tf_;

    //tf2
    tf2_ros::Buffer tf_buf;
    tf2_ros::TransformListener tf_listener;

    //status
    bool INIT;

    //debug
    ros::Publisher pub_debug_centroid_, pub_debug_marching_;
    //
    /*------------------------------*/
    /*          Call Backs          */
    /*------------------------------*/
    void updatePose(const geometry_msgs::PoseStampedConstPtr& msg);
    void updateWrench(const geometry_msgs::WrenchStampedConstPtr& msg);
    void plan(const image_segmentation::EllipseConstPtr& msg);

    bool scan_init(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

    /*------------------------------*/
    /*       helper functions       */
    /*------------------------------*/
    void fill_msg(iiwa_msgs::CartesianQuantity& msg, const std::vector<double>& arg_list);
    void controller_init(void);
    void init_pos(void);

    Eigen::Quaterniond curr_quat;
public:
    USController(ros::NodeHandle nh): 
    nh_(nh),
    tf_listener(tf_buf),
    curr_quat(0,0,1,0),
    force_({0,0,0}), torque_({0,0,0})
    {
        
        INIT = false;
        nh_.param<std::vector<double>>("/controller/cartesian_stiffness", cartesian_stiffness_, {20,20,20,20,20,20});
        nh_.param<std::vector<double>>("/controller/cartesian_damping", cartesian_damping_, {0.2,0.2,0.2,0.2,0.2,0.2});
        nh_.param<double>("/controller/nullspace_stiffness", nullspace_stiffness_, 10);
        nh_.param<double>("/controller/nullspace_damping", nullspace_damping_, 0.2);
        nh_.param<double>("/scan/marching_vel", marching_vel, 0.1);

        client_config_ = nh_.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
        controller_init();

        sub_eePose_ = nh_.subscribe("/iiwa/state/CartesianPose",10,&USController::updatePose,this);
        sub_eeFT_ = nh_.subscribe("/iiwa/state/CartesianWrench",10,&USController::updateWrench,this);
        pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
        sub_mask_ = nh_.subscribe("/ellipse_param",10,&USController::plan,this);
        
        srv_start_ = nh_.advertiseService("/start", &USController::scan_init, this);
        ROS_INFO_STREAM("Controller Class initialized");

        //debug
        pub_debug_centroid_ = nh_.advertise<geometry_msgs::PointStamped>("/debug/centroid",10);
        pub_debug_marching_ = nh_.advertise<geometry_msgs::PointStamped>("/debug/marching",10);
        //

        
    }
    
    ~USController(){}

};

bool USController::scan_init(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    if(req.data){
        init_pos();
        //debug
        //pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose_debug",10);
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
    nh_.param<std::vector<double>>("/init_pose", init_pose, {0.6, 0.1, 0, 0, 1, 0, 0});

    command.pose.position.x = init_pose[0];
    command.pose.position.y = init_pose[1];
    command.pose.position.z = init_pose[2];
    command.pose.orientation.x = init_pose[3];
    command.pose.orientation.y = init_pose[4];
    command.pose.orientation.z = init_pose[5];
    command.pose.orientation.w = init_pose[6];

    pub_desiredPose_.publish(command);
    
    while( (curr_pose.position.x-init_pose[0])+(curr_pose.position.y-init_pose[1])+(curr_pose.position.z-init_pose[2]) > 0.1);
    //while(tf_==ti_);
    ros::Duration(0.5).sleep();
    return;
}

void USController::plan(const image_segmentation::EllipseConstPtr& msg){
    tf_ = ros::Time::now();
    double dt = (tf_-ti_).toSec();
    
    if(!INIT){
        ROS_WARN_STREAM("waiting for position initialization");
        ti_ = ros::Time::now();
        return;
    }
    geometry_msgs::TransformStamped Te0,Te0_vct;
    //tf2::Stamped<tf2::Transform> Te0_vct;
    Eigen::Vector3d n, centroid; //vessel axial vector, centroid of the vessel in the current image wrt base_link

    try{
        Te0 = tf_buf.lookupTransform("iiwa_link_0", "iiwa_link_ee", ros::Time(0));
        Te0_vct = Te0;
        Te0_vct.transform.translation.x=0;
        Te0_vct.transform.translation.y=0;
        Te0_vct.transform.translation.z=0;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        //continue;
    }

    centroid << msg->pose.x, 0.0, msg->pose.y;
    //centroid = Te0*centroid;
    tf2::Stamped<Eigen::Vector3d> stampedCentroid(centroid,ros::Time(),"iiwa_link_ee");
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

    Eigen::Vector3d y_axis,z_axis;
    y_axis << 0.0, 1.0, 0.0;
    z_axis << 0.0, 0.0, 1.0;
    
    tf2::Stamped<Eigen::Vector3d> stampedY(y_axis,ros::Time(),"iiwa_link_ee");
    tf2::doTransform(stampedY, stampedY, Te0_vct);

    tf2::Stamped<Eigen::Vector3d> stampedZ(z_axis,ros::Time(),"iiwa_link_ee");
    tf2::doTransform(stampedZ, stampedZ, Te0_vct);

    Eigen::Vector3d marching;
    marching = stampedY*marching_vel*((tf_-ti_).toSec());

    //debug
    geometry_msgs::PointStamped msg_debug_marching;
    msg_debug_marching.header.frame_id = "iiwa_link_ee";
    msg_debug_marching.header.stamp = stampedCentroid.stamp_;
    msg_debug_marching.point.x = marching[0];
    msg_debug_marching.point.y = marching[1];
    msg_debug_marching.point.z = marching[2];
    pub_debug_marching_.publish(msg_debug_marching);
    //
    double rz = -std::atan(centroid[0]/marching_vel*((tf_-ti_).toSec()));
    Eigen::Quaterniond rel_quat(Eigen::AngleAxisd(rz,stampedZ));
    Eigen::Quaterniond cmd_quat;
    curr_quat = Eigen::Quaterniond(curr_pose.orientation.w,curr_pose.orientation.x,curr_pose.orientation.y,curr_pose.orientation.z);
    cmd_quat = rel_quat*curr_quat;

    geometry_msgs::PoseStamped command;
    command.header.frame_id = BASE_LINK;
    command.header.stamp = ros::Time::now();
    command.header.seq = msg->header.seq;
    command.pose.position.x = stampedCentroid[0] + marching[0];
    command.pose.position.y = stampedCentroid[1] + marching[1];
    command.pose.position.z = marching[2];
    // command.pose.position.x = 0.6;
    // command.pose.position.y = 0.1;
    // command.pose.position.z = 0;
    command.pose.orientation.x = cmd_quat.x();
    command.pose.orientation.y = cmd_quat.y();
    command.pose.orientation.z = cmd_quat.z();
    command.pose.orientation.w = cmd_quat.w();
    // command.pose.orientation.x = 0;
    // command.pose.orientation.y = 1;
    // command.pose.orientation.z = 0;
    // command.pose.orientation.w = 0;

    pub_desiredPose_.publish(command);

    stampedCentroid_d = stampedCentroid;
    ti_ = ros::Time::now();
    return;
}


void USController::updatePose(const geometry_msgs::PoseStampedConstPtr& msg){
    curr_pose = msg->pose;
    return;
}

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
    // ros::spin();

    return 0;
}