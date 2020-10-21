#include <ros/ros.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <sensor_msgs/JointState.h>

// #include <rosgraph_msgs/Clock.h>
// #include <chrono>

class statesTranslator{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_jpv_; //JointPositionVelocity
    ros::Publisher pub_js_; 

    void trans(const iiwa_msgs::JointPositionVelocityConstPtr& msg){
        sensor_msgs::JointState msg_js;
        msg_js.header = msg->header;
        msg_js.header.stamp = ros::Time::now();
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

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    statesTranslator translator(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // ros::Publisher pub_clk = nh.advertise<rosgraph_msgs::Clock>("/clock",2); 
    // const auto t0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};

    // rosgraph_msgs::Clock curr_time;
    // while(ros::ok()){
    //     auto stamp = std::chrono::high_resolution_clock::now()-t0;

    //     curr_time.clock.sec  = std::chrono::duration_cast<std::chrono::seconds>(stamp).count();
    //     curr_time.clock.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count() % 1000000000UL;

    //     pub_clk.publish(curr_time);
    //     //ROS_INFO_STREAM(curr_time.data.sec);
    // }

    ros::waitForShutdown();
    //ros::spin();
}