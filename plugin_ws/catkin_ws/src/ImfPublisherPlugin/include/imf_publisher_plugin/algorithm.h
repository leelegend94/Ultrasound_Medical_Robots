#pragma once

#include <ImFusion/Base/Algorithm.h>

#include <QObject>
#include <QtCore/QThread>

#include <string>

#include <ImFusion/Stream/ImageStream.h>
#include <ImFusion/Cephasonics/CephasonicsStream.h>
#include <ImFusion/ROS/Stream/ROSTopicImageOutStream.h>

namespace ImFusion {
namespace ImfPublisher {

//class PluginAlgorithm : public QObject, public ImFusion::Algorithm {
class PluginAlgorithm : public QThread, public ImFusion::Algorithm {
  Q_OBJECT
public:
  PluginAlgorithm() = default;
  ~PluginAlgorithm() = default;

  void compute();

  void run() override;

  void startStream(ImageStream* usStream);
  void stopStream();

  //! Methods implementing the Configurable interface.
  void configure(const Properties* p);
  void configuration(Properties* p) const;

  static bool createCompatible(const DataList& data, Algorithm** a = nullptr);

private:
  ImageStream* usStream;
  ROSTopicImageOutStream* rosOutStr;

protected:
  bool run_{true};

};
}  // namespace DemoROS
}  // namespace ImFusion
