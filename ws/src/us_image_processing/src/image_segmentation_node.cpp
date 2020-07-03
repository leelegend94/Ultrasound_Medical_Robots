#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <typeinfo>

#include <torch/script.h>

#include <eigen3/Eigen/Dense>
//

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

class ImageSegmentation{
private:
	ros::NodeHandle nh_;
	//ros::NodeHandle priv_nh_;

	ros::Subscriber sub_image_;
	ros::Publisher pub_segmentation_;
	ros::Publisher pub_pc2_;
    //ros::Publisher pub_ellipse_;
    //ros::Publisher pub_foldingPoints_;

	torch::jit::script::Module *module_;

    Eigen::Matrix3f calibMtx;

	void unet_inference(const sensor_msgs::Image::ConstPtr& msg);

public:
	ImageSegmentation(ros::NodeHandle nh,torch::jit::script::Module *module) : nh_(nh)
	{
		sub_image_ = nh_.subscribe("us_image", 10, &ImageSegmentation::unet_inference,this);
		pub_segmentation_ = nh_.advertise<sensor_msgs::Image>("us_segment",10);
		pub_pc2_ = nh_.advertise<sensor_msgs::PointCloud2>("us_vessel_pointcloud",10);
		//pub_ellipse_ = nh_.advertise<image_segmentation::Ellipse>("ellipse_param",10);
        //pub_foldingPoints_ = nh_.advertise<sensor_msgs::PointCloud>("folding_points",10);
        module_ = module;

        float sx,sy,cx,cz;
        nh_.param<float>("/calibration/scaling_x", sx, 0.2/512);
        nh_.param<float>("/calibration/scaling_y", sy, 0.2/512);
        nh_.param<float>("/calibration/c_x", cx, -0.1);
        nh_.param<float>("/calibration/c_z", cz, 0);

        calibMtx << sx,  0, cx,
                    0 ,  0,  0,
                    0 , sy, cz;
	}
	~ImageSegmentation(){}
};
 
void ImageSegmentation::unet_inference(const sensor_msgs::Image::ConstPtr& msg){
    //cv_bridge::CvImageConstPtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img_cv = cv_ptr->image;

    //cv::Mat img_fixedsize(512,512, img_cv.type());
    //cv::resize(img_cv, img_fixedsize, img_fixedsize.size(), 0,0,cv::INTER_LINEAR);
    //img_cv = img_fixedsize;
    //ROS_INFO_STREAM("image ready");
    torch::Tensor img_tensor = torch::from_blob(img_cv.data, { img_cv.rows, img_cv.cols, 1}, torch::kByte);
    img_tensor = img_tensor.permute({2,0,1});
    img_tensor = img_tensor.toType(torch::kFloat);
    img_tensor = img_tensor.div(255).sub(0.5).mul(2).unsqueeze(0);
    img_tensor = img_tensor.to(torch::kCUDA);
    //ROS_INFO_STREAM("before inference");
    torch::Tensor segmentation = module_->forward({img_tensor}).toTensor();
    //ROS_INFO_STREAM("after inference");
    segmentation = segmentation.squeeze(3); //I have no idea why only this works, probably a bug in libtorch
    segmentation = segmentation.detach();
    
    //segmentation = segmentation.permute({1,2,0});
    segmentation = segmentation.mul(255).clamp(0, 255).to(torch::kU8);
    segmentation = segmentation.to(torch::kCPU);

    cv::Mat resultImg8(img_cv.rows, img_cv.cols, CV_8UC1);
    std::memcpy((void *) resultImg8.data, segmentation.data_ptr(), sizeof(torch::kU8) * segmentation.numel());

    cv::Mat resultImg16(img_cv.rows, img_cv.cols, CV_16SC1);

    cv::Laplacian( resultImg8, resultImg16, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( resultImg16, resultImg16);

    threshold( resultImg16, resultImg8, 50, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> locations;
    cv::findNonZero(resultImg8, locations);

    Eigen::MatrixXf pointCloud(3,locations.size());
    for(unsigned int i=0; i<locations.size();i++){
        pointCloud(0,i) = locations[i].x;
        pointCloud(1,i) = locations[i].y;
        pointCloud(2,i) = 1;
    }
    pointCloud = calibMtx*pointCloud;

    //Format Transform: Eigen::MatrixXf --> pointcloud2.data
    uint8_t *pc2_data_ptr;
    pc2_data_ptr = reinterpret_cast<uint8_t *>(pointCloud.data());
    std::vector<uint8_t> pc2_data(pc2_data_ptr, pc2_data_ptr + 12*locations.size());
    
    sensor_msgs::PointCloud2 msg_pc2;
    msg_pc2.header.stamp = ros::Time::now();
    msg_pc2.header.frame_id = "iiwa_link_ee";
    msg_pc2.height = 1;
    msg_pc2.width = locations.size();
    msg_pc2.is_bigendian = false;
    msg_pc2.point_step = 12; // 3(x,y,z)*4(byte)
    msg_pc2.row_step = locations.size()*12;
    msg_pc2.is_dense = false;
    sensor_msgs::PointField f1,f2,f3;
    f1.name = "x";
    f1.offset = 0;
    f1.datatype = 7;
    f1.count = 1;
    f2.name = "y";
    f2.offset = 4;
    f2.datatype = 7;
    f2.count = 1;
    f3.name = "z";
    f3.offset = 8;
    f3.datatype = 7;
    f3.count = 1;
    msg_pc2.fields.push_back(f1);
    msg_pc2.fields.push_back(f2);
    msg_pc2.fields.push_back(f3);
    msg_pc2.data = pc2_data;

    pub_pc2_.publish(msg_pc2);

/*
    std::vector<cv::Moments> m;
    m = cv::moments(resultImg8,false);
    cv::Point2i centroid(m.m10/m.m00, m.m01/m.m00);

    int hori=0;
    for(int i=0;i<resultImg8.cols;i++){
        if(resultImg8.at(centroid.y,i)>0) hori++;
    }

    int verti=0;
    for(int i=0;i<resultImg8.rows;i++){
        if(resultImg8.at(i,centroid.x)>0) verti++;
    }
    
*/

    cv_ptr->image = resultImg8;

    pub_segmentation_.publish(cv_ptr->toImageMsg());

    /*--------------- Ellipse Fitting -----------------*/
    // cv::Mat locations;
    // cv::findNonZero(resultImg8, locations);

    // //ROS_INFO_STREAM("points: "<<locations.total());
    // cv::RotatedRect bbox = fitEllipse(locations);

    // cv::ellipse(cv_ptr->image, bbox, cv::Scalar(255), 1, 8);
    // pub_segmentation_.publish(cv_ptr->toImageMsg());

    // image_segmentation::Ellipse msg_ellipse;
    // msg_ellipse.header = cv_ptr->header;

    // ROS_INFO_STREAM("centroid: "<<bbox.center.x<<", "<<bbox.center.y);
    // Eigen::Vector3f centroid(bbox.center.x, bbox.center.y, 1);
    // centroid = calibMtx*centroid;

    // msg_ellipse.pose.x = centroid(0);
    // msg_ellipse.pose.y = centroid(2);
    // msg_ellipse.pose.theta = bbox.angle;
    // msg_ellipse.diameter1 = bbox.size.width;
    // msg_ellipse.diameter2 = bbox.size.height;
    // pub_ellipse_.publish(msg_ellipse);
    /*-------------------------------------------------*/
    
    return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "image_segmentation");
	ros::NodeHandle nh;

	torch::jit::script::Module module;
	try {
		// Deserialize the ScriptModule from a file using torch::jit::load().
		module = torch::jit::load("/home/eadu/workspace/us_robot/unet_usseg_traced.pt");
	}
	catch (const c10::Error& e) {
		std::cerr << "error loading the model: "<<e.what()<<"\n";
		return -1;
	}
	module.to(at::kCUDA);

	ImageSegmentation node(nh,&module);
	ros::spin();

	return 0;
}