#include "imf_publisher_plugin/factory.h"
#include "imf_publisher_plugin/algorithm.h"
#include "imf_publisher_plugin/controller.h"

namespace ImFusion {
namespace ImfPublisher {

PluginAlgorithmFactory::PluginAlgorithmFactory() { registerAlgorithm<PluginAlgorithm>("ROS;ImfPublisher"); }

AlgorithmController* PluginControllerFactory::create(ImFusion::Algorithm* a) const {
  if (PluginAlgorithm* algorithm = dynamic_cast<PluginAlgorithm*>(a)) { return new PluginController(algorithm); }
  return nullptr;
}

}  // namespace DemoROS
}  // namespace ImFusion
