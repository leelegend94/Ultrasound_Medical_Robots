#pragma once

#include <ImFusion/GUI/AlgorithmController.h>
#include <QtWidgets/QWidget>
#include <memory>
#include <QtCore/QObject>
#include <orien_adj/ConfidentMapConvex.h>

#include <qt5/QtNetwork/QTcpServer>  //for TCP
#include <qt5/QtNetwork/QTcpSocket>  //for TCP

#include <QTimer>

//#include <ImFusion/IO/PngIO.h>
#define TIM_INTERVAL_REC 25
#define TIM_INTERVAL_UPDATE 100


class Ui_Controller;

namespace ImFusion {
namespace OrienAdj {

class PluginAlgorithm;
//class ConfidentMapConvex;  //what happen if added this line?

class PluginController : public QWidget, public AlgorithmController {
  Q_OBJECT
public:
  /// Constructor with the algorithm instance
  PluginController(PluginAlgorithm* algorithm);
  virtual ~PluginController() = default;

  void init();

public slots:
  void onTestClick();

  void onConnectToRobotClick();
  void onDisconnectFromRobotClick();
  void onExecuteMovementClick();

  void onFanMotionTypeIndexChanged(int nCurrentIndex);



  //OpenCV Part
  void onReadImageClick();
  void onSaveImageClick();
  void onRefineFanClick();
  void onGenCarFigClick();
  void onGenPolFigClick();
  void onComputeConfiMapClick();   //sepreate step to calculate confidence map
  void onConfiMapInterClick();   //calculate from live us Image
  void onGoResultClick();   //adjust the probe according to the confidence map result

  bool recordUSimages(DataList* dataList);

  void setImageDir(DataList *dataList);   //always set the image in the right direction

  //task
  void onGoFanStartClick();
  void onFanMotionClick();

  void onStop();
  void onSaveHome();
  void onSaveOut();
  void onGoHome();
  void onGoOut();

  void onForceModeClick();
  void onPositionModeClick();

  //save the direction find by volunteer
  void onSaveDirectionClick();

signals:
  // Emit connection signals so other plugins can check the robot status.
  void robotConnected();
  void robotDisconnected();

private:
  void updateUIToLastPose();
  void updateUIToLastWrench();
  void recordRobotStatus();

  std::shared_ptr<Ui_Controller> ui_{nullptr};  ///< The actual GUI
  PluginAlgorithm* algorithm_{nullptr};         ///< The algorithm instance



  //opencv stuff
public: 
  cv::Mat m_CvCatesian;   //save the catesian figure obtained from polar image

private:
  ConfidentMapConvex* m_confiMapConvex;

public:
  QTcpServer *tcpServer;
  QTcpSocket *serverSocket;
  void onRecForce();
  std::vector<float> m_forceData;
  QTimer *m_recordTimer;
  QTimer *m_updateTimer;

public slots:
  void acceptConnection();
  void replyToClient();

  void onSynRecord();
  void onchbRecordStateChanged();
  void onUpdateState();
  void onChbUpdateStateChanged();



};

}  // namespace OrienAdj
}  // namespace ImFusion
