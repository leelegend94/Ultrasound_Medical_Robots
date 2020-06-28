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
	//igtl::TimeStamp::Pointer ts_;

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
		int channels;
		image->GetDimensions(size);
		channels = image->GetNumComponents();

		// ROS_INFO_STREAM("image size: "<<size[0]<<'x'<<size[1]<<'x'<<size[2]);
		// ROS_INFO_STREAM("channels: "<<channels);
		// ROS_INFO_STREAM("image data size: "<<image->GetImageSize());
		// ROS_INFO_STREAM("element size: "<<image->GetScalarSize());

		cv::Mat resultImg8(size[1], size[0], CV_8UC1);
		// ROS_INFO_STREAM("translating..");
		std::memcpy((void *)resultImg8.data, (uint8_t *)image->GetScalarPointer(), image->GetImageSize());
		// ROS_INFO_STREAM("succeed!");

		cv_bridge::CvImage cv_img;
		cv_img.header.stamp = ros::Time::now();
		cv_img.encoding = "mono8";
		cv_img.image = resultImg8;

		pub_image.publish(cv_img.toImageMsg());
		// ROS_INFO_STREAM("published!");
		return;
	}
	

public:
	IGTL_ROS_Transponder(ros::NodeHandle nh, const std::string& address, const std::string& port):
	nh_(nh)
	{
		pub_image = nh_.advertise<sensor_msgs::Image>("/us_image",10);

		// Establish Connection
		socket_ = igtl::ClientSocket::New();
		socket_->SetReceiveTimeout(1000);

		int r = socket_->ConnectToServer(address.c_str(), std::atoi(port.c_str()));
		
		if (r != 0){
			ROS_ERROR_STREAM("Cannot connect to the server.");
			ros::shutdown();
		}

		headerMsg_ = igtl::MessageHeader::New();
		//ts_ = igtl::TimeStamp::New();
	}

	void receive(){
		// Initialize receive buffer
		//igtl::MessageHeader::Pointer headerMsg_ = igtl::MessageHeader::New();
		headerMsg_->InitPack();

		int received_bytes = socket_->Receive(headerMsg_->GetPackPointer(), headerMsg_->GetPackSize());
		//ROS_INFO_STREAM("received_bytes: "<<received_bytes);
		//ROS_INFO_STREAM("should have received: "<<headerMsg_->GetPackSize());
		// Check if the size of the received bytes is correct.
		if (received_bytes == headerMsg_->GetPackSize()) {
			headerMsg_->Unpack();

			if (std::string(headerMsg_->GetDeviceType()) == "IMAGE") {
				ROS_INFO_STREAM("IMAGE received!");
				receiveImage_(headerMsg_);
			}else{
				ROS_WARN_STREAM("device type mismatch, got: "<<std::string(headerMsg_->GetDeviceType()));
				socket_->Skip(headerMsg_->GetBodySizeToRead(), 0);
				//ros::shutdown();
			}
		}else{
			ROS_ERROR_STREAM("transmission error");
			ros::shutdown();
		}
		return;
	}

};


int main(int argc, char** argv){
	ros::init(argc, argv, "igtl_read");
	ros::NodeHandle nh;
	IGTL_ROS_Transponder transponder(nh, "172.31.1.160","18944");
	ROS_INFO_STREAM("transponder initialized!");
	while(ros::ok()){
		//ROS_INFO_STREAM("receving...");
		transponder.receive();
	}
	return 0;
}
