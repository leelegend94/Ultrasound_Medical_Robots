#include "vessel_segmentation/ultrasoundFrame.h"
#include "segmenter.h"

bool findVesselCenterPoint(vessel_segmentation::ultrasoundFrame::Request& req, vessel_segmentation::ultrasoundFrame::Response& res)
{
  Segmenter segmenter(req.image,req.worldToImage);
  res.centerpoints = segmenter.compute();

  return true;
}
