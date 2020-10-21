#include "orien_adj/OrienAdjFactory.h"
#include "orien_adj/OrienAdjAlgorithm.h"
#include "orien_adj/OrienAdjController.h"

namespace ImFusion {
namespace OrienAdj {

PluginAlgorithmFactory::PluginAlgorithmFactory() { registerAlgorithm<PluginAlgorithm>("Robot;OrientAdj"); }

AlgorithmController* PluginControllerFactory::create(ImFusion::Algorithm* a) const {
  if (PluginAlgorithm* algorithm = dynamic_cast<PluginAlgorithm*>(a)) { return new PluginController(algorithm); }
  return nullptr;
}

}  // namespace OrienAdj
}  // namespace ImFusion
