#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "igtlOSUtil.h"
#include "igtl/igtlMessageHeader.h"
#include "igtl/igtlImageMessage.h"
#include "igtl/igtlClientSocket.h"


class IGTL_ROS_Transponder{
private:
	ros::NodeHandle nh_;

	ros::Publisher pub_image;

	igtl::ClientSocket::Pointer socket_;

	// Create a message buffer to receive header
	igtl::MessageHeader::Pointer headerMsg_;

	// Allocate a time stamp
	igtl::TimeStamp::Pointer ts_;

	void receiveImage_(const igtl::MessageHeader::Pointer& header) {
		igtl::ImageMessage::Pointer image = igtl::ImageMessage::New();

		image->SetMessageHeader(header);
		image->AllocatePack();

		// Receive image data from the socket.
		socket_->Receive(image->GetPackBodyPointer(), image->GetPackBodySize());

		// Deserialize the image data.
		image->Unpack(); //without CRC arg (1) to activate

		// Retrive the image data
		int size[3]; //ijk?
		//int channels;
		image->GetDimensions(size);
		//channels = image->GetNumComponents();

		cv::Mat resultImg8(size[0], size[1], CV_8UC1);
		std::memcpy((void *) resultImg8.data, image->GetScalarPointer(), image->GetImageSize());

		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr->image = resultImg8;
		pub_image.publish(cv_ptr->toImageMsg());
	}
	

public:
	IGTL_ROS_Transponder(ros::NodeHandle nh, const std::string& address, const std::string& port):
	nh_(nh)
	{
		pub_image = nh_.advertise<sensor_msgs::Image>("/us_image",10);

		// Establish Connection
		socket_ = igtl::ClientSocket::New();
		int r = socket_->ConnectToServer(address.c_str(), std::atoi(port.c_str()));
		
		if (r != 0){
			ROS_INFO_STREAM("Cannot connect to the server.");
			ros::shutdown();
		}

		headerMsg_ = igtl::MessageHeader::New();
		ts_ = igtl::TimeStamp::New();
	}

	void receive(){
		// Initialize receive buffer
		headerMsg_->InitPack();
		int received_bytes = socket_->Receive(headerMsg_->GetPackPointer(), headerMsg_->GetPackSize());
		
		// Check if the size of the received bytes is correct.
		if (received_bytes == headerMsg_->GetPackSize()) {
			headerMsg_->Unpack();

			if (std::string(headerMsg_->GetDeviceType()) == "IMAGE") {
				//receiveImage_(headerMsg_);
			}
		}
		return;
	}

};




int main(int argc, char** argv){
	ros::init(argc, argv, "igtl_read");
	ros::NodeHandle nh;
	IGTL_ROS_Transponder transponder(nh, "172.31.1.160","18944");

	while(ros::ok()){
		//transponder.receive();
		ros::spinOnce();
	}

	ros::spin();
	return 0;
}
