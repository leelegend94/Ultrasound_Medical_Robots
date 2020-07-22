#include "sensor_msgs/Image.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Point.h"
#include "opencv2/opencv.hpp"
#include <Eigen/Eigen>

class Segmenter
{
public:
  Segmenter(sensor_msgs::Image image, geometry_msgs::Transform worldToImage);
  ~Segmenter();

  /**
	 * @brief Locates vessels in 2D US image, fits an ellipse to each vessel, computes the ellipse center and map the image coordinate to world coordinate.
	 */
  std::vector<geometry_msgs::Point> compute();

private:

  /**
	 * @brief Smoothes the original image for the given scale. Computes the 2nd derivatives XX, XY and YY (Hessian) and stores it's eigenvalues (in growing order) into the matrices lambda1_ and lambda2_.
   * @param scale Determines the scale of the smoothing filer (scale = sigma)
   * @param vesselness Matrix containing max vesselness measure of each pixel over all scales.
	 */
  void computeVesselnessAtScale(int scale, cv::Mat& vesselness);

  /**
	 * @brief Spans a range of scales and computes the vesselness response of each pixel. Keeps the maximum score over all scales.
	 */
  cv::Mat computeVesselnessMultiscale();

  /**
	 * @brief
	 */
  void vesselnessMask(cv::Mat& vesselness);

  /**
	 * @brief
	 */
  std::vector<std::vector<cv::Point>> groupVesselRegions(cv::Mat& mask);

  /**
	 * @brief
	 */
  std::vector<cv::Point2f> fitEllipseToVessels(std::vector<std::vector<cv::Point>> contours);


  /**
	 * @brief
	 */
  void transformPointsToWorld(std::vector<cv::Point2f>);

// Data Members
  cv::Mat originalImage_;
  Eigen::Matrix4d transform_;
};
