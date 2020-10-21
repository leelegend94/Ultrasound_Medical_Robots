#include "imf_publisher_plugin/plugin.h"
#include "imf_publisher_plugin/factory.h"

#ifdef WIN32
extern "C" __declspec(dllexport) ImFusion::ImFusionPlugin* createPlugin()
#else
extern "C" ImFusion::ImFusionPlugin* createPlugin()
#endif
{
  return new ImFusion::ImfPublisher::Plugin;
}

namespace ImFusion {
namespace ImfPublisher {

Plugin::Plugin() {
  algorithm_factory_ = std::make_unique<PluginAlgorithmFactory>();
  algorithm_controller_factory_ = std::make_unique<PluginControllerFactory>();
}

const ImFusion::AlgorithmFactory* Plugin::getAlgorithmFactory() { return algorithm_factory_.get(); }

const ImFusion::AlgorithmControllerFactory* Plugin::getAlgorithmControllerFactory() {
  return algorithm_controller_factory_.get();
}
}  // namespace DemoROS
}  // namespace ImFusion
