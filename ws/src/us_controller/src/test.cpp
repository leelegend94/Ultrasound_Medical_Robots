#include <ros/ros.h>

void init_CartImp(void){
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

void fill_msg(iiwa_msgs::CartesianQuantity& msg, const std::vector<double>& arg_list){
    msg.x = arg_list[0];
    msg.y = arg_list[1];
    msg.z = arg_list[2];
    msg.a = arg_list[3];
    msg.b = arg_list[4];
    msg.c = arg_list[5];
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_intMarker
    

    init_CartImp();

    while(ros::ok()){

    }

    return 0;
}