#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <iiwa_msgs/ControlMode.h>
#include <iiwa_msgs/ConfigureControlMode.h>
#include <iiwa_msgs/CartesianImpedanceControlMode.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

//#define SIM
const std::string BASE_LINK = "iiwa_link_0";

void fill_msg(iiwa_msgs::CartesianQuantity& msg, const std::vector<double>& arg_list){
    msg.x = arg_list[0];
    msg.y = arg_list[1];
    msg.z = arg_list[2];
    msg.a = arg_list[3];
    msg.b = arg_list[4];
    msg.c = arg_list[5];
    return;
}

void init_CartImp(ros::NodeHandle nh){

    ros::ServiceClient client_config_ = nh.serviceClient<iiwa_msgs::ConfigureControlMode>("/iiwa/configuration/ConfigureControlMode");
    iiwa_msgs::ConfigureControlMode msg_config;
    msg_config.request.control_mode = iiwa_msgs::ControlMode::CARTESIAN_IMPEDANCE;
    
    iiwa_msgs::CartesianImpedanceControlMode impedance_config;
    std::vector<double> cartesian_stiffness_;
    std::vector<double> cartesian_damping_;
    double nullspace_stiffness_;
    double nullspace_damping_;
    nh.param<std::vector<double>>("/controller/cartesian_stiffness", cartesian_stiffness_, {200,200,200,10,10,10});
    nh.param<std::vector<double>>("/controller/cartesian_damping", cartesian_damping_, {0.5,0.5,0.5,0.2,0.2,0.2});
    nh.param<double>("/controller/nullspace_stiffness", nullspace_stiffness_, 10);
    nh.param<double>("/controller/nullspace_damping", nullspace_damping_, 0.5);

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
class statesTranslator{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_jpv_; //JointPositionVelocity
    ros::Publisher pub_js_; 

    void trans(const iiwa_msgs::JointPositionVelocityConstPtr& msg){
        sensor_msgs::JointState msg_js;
        msg_js.header = msg->header;
        msg_js.name = {"iiwa_joint_1","iiwa_joint_2","iiwa_joint_3","iiwa_joint_4","iiwa_joint_5","iiwa_joint_6","iiwa_joint_7"};
        msg_js.position = {msg->position.a1,msg->position.a2,msg->position.a3,msg->position.a4,msg->position.a5,msg->position.a6,msg->position.a7};
        msg_js.velocity = {msg->velocity.a1,msg->velocity.a2,msg->velocity.a3,msg->velocity.a4,msg->velocity.a5,msg->velocity.a6,msg->velocity.a7};
        pub_js_.publish(msg_js);
        return;
    }

public:
    statesTranslator(ros::NodeHandle nh):nh_(nh){
        sub_jpv_ = nh_.subscribe("/iiwa/state/JointPositionVelocity",1,&statesTranslator::trans,this);
        pub_js_ = nh_.advertise<sensor_msgs::JointState>("/joint_states",2);
    }

};

class SubsPose{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_eePose_;
    ros::Publisher pub_desiredPose_;
    geometry_msgs::Pose curr_pose_;

public:
    SubsPose(ros::NodeHandle nh):nh_(nh){
        sub_eePose_ = nh_.subscribe("/iiwa/state/CartesianPose",10,&SubsPose::updatePose,this);
        pub_desiredPose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
        INIT = false;
    }

#ifdef SIM
    void updatePose(const geometry_msgs::PoseStampedConstPtr& msg){
        //ROS_INFO_STREAM("update");
        INIT = true;
        curr_pose_ = msg->pose;
        return;
    }
#else
    void updatePose(const iiwa_msgs::CartesianPoseConstPtr& msg){
        //ROS_INFO_STREAM("update");
        INIT = true;
        curr_pose_ = msg->poseStamped.pose;
        return;
    }
#endif

    geometry_msgs::Pose getPose(void){
        //ROS_INFO_STREAM(curr_pose_);
        return curr_pose_;
    }

    void init_pose(void){
        geometry_msgs::PoseStamped command;
        command.header.frame_id = BASE_LINK;

        std::vector<double> init_pose;
        nh_.param<std::vector<double>>("/init_pose", init_pose, {0.0, -0.65, 0.45, 1, 0, 0, 0});

        command.pose.position.x = init_pose[0];
        command.pose.position.y = init_pose[1];
        command.pose.position.z = init_pose[2];
        command.pose.orientation.x = init_pose[3];
        command.pose.orientation.y = init_pose[4];
        command.pose.orientation.z = init_pose[5];
        command.pose.orientation.w = init_pose[6];

        pub_desiredPose_.publish(command);
        
        while( ros::ok() && (std::abs(curr_pose_.position.x-init_pose[0])+std::abs(curr_pose_.position.y-init_pose[1])+std::abs(curr_pose_.position.z-init_pose[2]) > 0.1)){
            ROS_INFO_STREAM(curr_pose_);
            pub_desiredPose_.publish(command);
        };


        //while(tf_==ti_);
        ROS_INFO_STREAM("init finished");
        return;
    }

    bool INIT;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    
    init_CartImp(nh);

    ros::Publisher pub_desiredPose = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);

    SubsPose pose_updater(nh);
    statesTranslator translator(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    //ros::waitForShutdown();
    //ros::spin();

    //get current position 
    while(ros::ok() && pose_updater.INIT==false){
        ROS_INFO_STREAM("waiting for init");
    }

    //goto init position
    pose_updater.init_pose();

    geometry_msgs::Pose init_pose = pose_updater.getPose();

    ROS_INFO_STREAM("pos: \n"<<init_pose);
    //geometry_msgs::PoseStamped command;
    //command.header.frame_id = "iiwa_link_0";
    // command.pose.position.x = init_pose[0];
    // command.pose.position.y = init_pose[1];
    // command.pose.position.z = init_pose[2];
    // command.pose.orientation.x = init_pose[3];
    // command.pose.orientation.y = init_pose[4];
    // command.pose.orientation.z = init_pose[5];
    // command.pose.orientation.w = init_pose[6];
    //command.pose = init_pose;

    double t0 = ros::Time::now().toSec();
    double t;

    ros::Rate r(50);
    while(ros::ok()){
        geometry_msgs::PoseStamped command;
        command.header.frame_id = "iiwa_link_0";
        //command.header.stamp = ros::Time::now();
        command.pose = init_pose;
        //ROS_INFO_STREAM("Ha?");
        t = ros::Time::now().toSec()-t0;
        //ROS_INFO_STREAM("t: "<<t);
        command.pose.position.x = init_pose.position.x + 0.1*std::sin(6*0.25*t);
        command.pose.position.z = init_pose.position.z + 0.1*std::sin(6*0.25*t);
        //command.pose.position.x = init_pose.position.x + 0.1;
        //("x: "<<command.pose.position.x);
        pub_desiredPose.publish(command);
        //ROS_INFO_STREAM(command.pose);
        r.sleep();
    }

    return 0;
}