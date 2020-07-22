#include "segmenter.h"


Segmenter::Segmenter(sensor_msgs::Image image, geometry_msgs::Transform worldToImage)
{

}

Segmenter::~Segmenter()
{


}

std::vector<geometry_msgs::Point> Segmenter::compute()
{




}

void Segmenter::computeVesselnessAtScale(int scale, cv::Mat& vesselness)
{
  //Try fixed filter size = 99
  cv::Mat blurred_image;
  cv::GaussianBlur(originalImage_,blurred_image,cv::Size(6*scale+1,6*scale+1),scale);

  //Convert to gray.
  cv::cvtColor(blurred_image,blurred_image,cv::COLOR_BGR2GRAY);

  //Apply 3x3 derivatives in XX, YY and XY
  cv::Mat Ixx, Iyy, Ixy;
  cv::Sobel(blurred_image,Ixx,cv::CV_16U,2,0,3);
  cv::Sobel(blurred_image,Iyy,cv::CV_16U,0,2,3);
  cv::Sobel(blurred_image,Ixy,cv::CV_16U,1,1,3);

  //Correct for scale
  Ixx = Ixx*(scale^2);
  Ixy = Ixy*(scale^2);
  Iyy = Iyy*(scale^2);

  //Compute the eigenvalues of the Hessian and store them if they are the largest so far.

  int rows = Ixx.rows;
  int cols = Ixx.cols;
  for(int i=0;i<rows;i++)
  {
    for(int j=0;j<cols;j++)
    {
      Eigen::Matrix2d H;
      H << Ixx.at<uchar>(j,i), Ixy.at<uchar>(j,i), Iyy.at<uchar>(j,i), Ixy.at<uchar>(j,i);
      Eigen::Vector2d eigenvalues = H.eigenvalues();
      double lambda1 = eigenvalues.abs().minCoeff();
      double lambda2 = eigenvalues.abs().maxCoeff();

      //Compute the vesselness of the pixel.
      if(lambda2 > 0 && lambda1 > 0)
      {
        Rb = lambda1 / lambda2;
        S2 = lambda1^2 + lambda2^2;
        double v = exp(-Rb^2/2) * (1 - exp(-S2/200));
        if(v > vesselness.at<uchar>(j,i))
        {
          vesselness.at<uchar>(j,i) = v;
        }
      }
    }
  }
}

cv::Mat Segmenter::computeVesselnessMultiscale()
{
  //Create range of scales.
  int sigma_min = 30;
  int sigma_max = 40;
  int sigma_step = 2;

  int s = sigma_min;

  cv::Mat max_vesselness_all_scales = cv::Mat::zeros(originalImage_.size(), cv::CV_8UC1);

  while(s <= sigma_max)
  {
    computeVesselnessAtScale(s, max_vesselness_all_scales);

    s = s + sigma_step;
  }
  return max_vesselness_all_scales;
}

void Segmenter::vesselnessMask(cv::Mat& vesselness)
{
  cv::threshold(vesselness,vesselness,0.2,1,0);
}

std::vector<std::vector<cv::Point>> Segmenter::groupVesselRegions(cv::Mat& mask)
{
  //Get edges from the vesselness mask.
  cv::Canny( mask, mask, 1, 3 );

  //Group each contour as a vector of points.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask,contours,cv::CV_RETR_EXTERNAL,cv::CV_CHAIN_APPROX_NONE);

  return contours;

}

std::vector<cv::Point2f> Segmenter::fitEllipseToVessels(std::vector<std::vector<cv::Point>> contours)
{
  //Create a vector to store the fitted ellipses.
  std::vector<cv::Point2f> ellipse_centers{};

  //For each region of pixels, compute average of pixels intensities in original image.
  for(auto region : contours)
  {
    //Create mask where all pixels inside the region are filled.
    cv::Mat masked_region = cv::Mat::zeros(originalImage_.size(), cv::CV_8UC1);
    cv::fillConvexPoly(masked_region,region,cv::Scalar(255));

    std::vector<cv::Point> pixels;

    cv::findNonZero(masked_region,pixels);

    int sum_intensities;
    for(auto point : pixels)
    {
      sum_intensities += originalImage_.at<uchar>(point);
    }
    double average = sum_intensities/pixels.size();

    //If the average intensity is lower than a threshold, fit an elllipse to that region.
    if(average < 35)
    {
      auto ellipse = cv::fitEllipse(region);

      //Exclude ellipses with short axes ratio and with smaller axis larger than a threshold.
      double axis_1 = ellipse.size.width;
      double axis_2 = ellipse.size.height;

      if (axis_1 < axis_2)
      {
        if (axis_1 > 20 && axis_2 < 120 && axis_1/axis_2 > 0.5)
        {
          ellipse_centers.emplace_back(ellipse.center);
        }
      }
      else
      {
        if (axis_1 < 120 && axis_2 > 20 && axis_2/axis_1 > 0.5)
        {
          ellipse_centers.emplace_back(ellipse.center);
        }
      }

    }
  }

  return ellipse_centers;
}
