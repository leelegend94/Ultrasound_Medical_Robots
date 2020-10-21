#pragma once

#include <ImFusion/GUI/AlgorithmController.h>
#include <QtWidgets/QWidget>

#include <memory>

#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>
#include<ImFusion/Cephasonics/CephasonicsStream.h>

class Ui_Controller;

namespace ImFusion {
namespace ImfPublisher {

class PluginAlgorithm;

class PluginController : public QWidget, public AlgorithmController{
  Q_OBJECT
public:
  /// Constructor with the algorithm instance
  PluginController(PluginAlgorithm* algorithm);
  
  virtual ~PluginController() = default;

  void init();

public slots:
  void onStartClicked();
  void onStopClicked();

private:
  std::shared_ptr<Ui_Controller> ui_{nullptr};  ///< The actual GUI
  PluginAlgorithm* algorithm_{nullptr};         ///< The algorithm instance

};

}  // namespace DemoROS
}  // namespace ImFusion
