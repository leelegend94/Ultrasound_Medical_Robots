#include "orien_adj/OrienAdjController.h"   //JZL//orien_adj = iiwa_imfusion is the name of iiwa packge
#include "orien_adj/OrienAdjAlgorithm.h"
#include "orien_adj/ConfidentMapConvex.h"
#include "ui_controller.h"

#include <ImFusion/GUI/MainWindowBase.h>
#include <ImFusion/Base/Log.h>   //for LOG_INFO...ect
#include <iiwa_msgs/DOF.h>

//imfusion part
#include "ImFusion/Base/ImFusionFile.h"
#include <ImFusion/GUI/AnnotationModel.h>
#include <ImFusion/GUI/DataWidget.h>
#include <ImFusion/GUI/DisplayWidgetMulti.h>
#include <ImFusion/GUI/ImageView2D.h>
#include <ImFusion/GUI/ImageView3D.h>
#include <ImFusion/GUI/Interactive.h>
#include <ImFusion/GUI/MainWindowBase.h>

#include <QtDebug>
#include <QFile>
#include <QTextStream>
#include <QDir>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <QTime>

#include <opencv2/core/mat.hpp>



namespace ImFusion {
namespace OrienAdj {

PluginController::PluginController(PluginAlgorithm* algorithm)
    : AlgorithmController(algorithm)
    , algorithm_{algorithm} {
  ui_ = std::make_shared<Ui_Controller>();
  ui_->setupUi(this);
  ui_->cmbRotationStep->setEnabled(false);   //disable the step value when start the plugin
  ui_->pbtnGoResult->setEnabled(false);
  m_confiMapConvex = new ConfidentMapConvex();



  //TCP
  tcpServer = new QTcpServer(this);
  serverSocket = new QTcpSocket(this);
  m_forceData.resize(6);

  //synchronous acquisition
  m_recordTimer  = new QTimer(this);
  m_recordTimer->setTimerType(Qt::PreciseTimer);

  m_updateTimer = new QTimer(this);
  m_updateTimer->setTimerType(Qt::PreciseTimer);


}

void PluginController::init() {
  addToAlgorithmDock();

  //update the robot state after checking checking box

  connect(ui_->pbtnTest, &QPushButton::clicked, this, &PluginController::onTestClick);
  connect(ui_->pbtnConnect, &QPushButton::clicked, this, &PluginController::onConnectToRobotClick);
  connect(ui_->pbtnDisconnect, &QPushButton::clicked, this, &PluginController::onDisconnectFromRobotClick);

  connect(ui_->pbtnStop, &QPushButton::clicked, this, &PluginController::onStop);
  connect(ui_->pbtnForceMode, &QPushButton::clicked, this, &PluginController::onForceModeClick);
  connect(ui_->pbtnPositionMode, &QPushButton::clicked, this, &PluginController::onPositionModeClick);
  connect(ui_->pbtnSaveHome, &QPushButton::clicked, this, &PluginController::onSaveHome);
  connect(ui_->pbtnSaveOut, &QPushButton::clicked, this, &PluginController::onSaveOut);
  connect(ui_->pbtnGoHome, &QPushButton::clicked, this, &PluginController::onGoHome);
  connect(ui_->pbtnGoOut, &QPushButton::clicked, this, &PluginController::onGoOut);
  connect(ui_->pbtnExcuteCmd, &QPushButton::clicked, this, &PluginController::onExecuteMovementClick);


  //openCV part
  connect(ui_->pbtnReadImage, &QPushButton::clicked, this, &PluginController::onReadImageClick);
  connect(ui_->pbtnSaveImage, &QPushButton::clicked, this, &PluginController::onSaveImageClick);
  connect(ui_->pbtnRefineFan, &QPushButton::clicked, this, &PluginController::onRefineFanClick);
  connect(ui_->pbtnGenCarFig, &QPushButton::clicked, this, &PluginController::onGenCarFigClick);
  connect(ui_->pbtnComputeConfiMap, &QPushButton::clicked, this, &PluginController::onComputeConfiMapClick);
  connect(ui_->pbtnGenPolFig, &QPushButton::clicked, this, &PluginController::onGenPolFigClick);
  connect(ui_->pbtnConfiMapInter, &QPushButton::clicked, this, &PluginController::onConfiMapInterClick);
  connect(ui_->pbtnGoResult, &QPushButton::clicked, this, &PluginController::onGoResultClick);




  // Task
  connect(ui_->pbtnGoFanStart, &QPushButton::clicked, this, &PluginController::onGoFanStartClick);
  connect(ui_->pbtnFanMotion, &QPushButton::clicked, this, &PluginController::onFanMotionClick);

  //update the state when the state changed! changed to use timer
//  connect(algorithm_, &PluginAlgorithm::poseChanged, this, &PluginController::updateUIToLastPose);
//  connect(algorithm_, &PluginAlgorithm::wrenchChanged, this, &PluginController::updateUIToLastWrench);

//  connect(ui_->cmbFanMotionType, &QComboBox::currentIndexChanged, this, &PluginController::onFanMotionTypeIndexChanged)
  //additional things
//  connect(ui_->cmbFanMotionType, &QPushButton::currentIndexChanged, this, &PluginController::onFanMotionTypeIndexChanged);

  connect(ui_->cmbFanMotionType, SIGNAL(currentIndexChanged(int)), this, SLOT(onFanMotionTypeIndexChanged(int)));

  //save the direction find by volunteer
  connect(ui_->pbtnSaveDirection, &QPushButton::clicked, this, &PluginController::onSaveDirectionClick);

  //receive the force measurement from robot via TCP
  connect(ui_->pbtnRecForce, &QPushButton::clicked, this, &PluginController::onRecForce);

  //Synchronous acquisition
  QObject::connect(m_recordTimer, &QTimer::timeout, this, &PluginController::onSynRecord);
  QObject::connect(ui_->chbRecordWrench, &QCheckBox::stateChanged, this, &PluginController::onchbRecordStateChanged);
  //update the robot state in UI
  QObject::connect(ui_->chbUpdateState, &QCheckBox::stateChanged, this, &PluginController::onChbUpdateStateChanged);
  QObject::connect(m_updateTimer, &QTimer::timeout, this, &PluginController::onUpdateState);

}

void PluginController::onExecuteMovementClick() {
  Eigen::Vector3d rotAngles(ui_->ledtAngRa->text().toDouble(), ui_->ledtAngRb->text().toDouble(),
                            ui_->ledtAngRc->text().toDouble());
  rotAngles *= M_PI / 180.;   //rad
  algorithm_->executeCartesianCommand(
          Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rotAngles[2], Eigen::Vector3d::UnitZ()),
          Eigen::Vector3d(ui_->ledtPosX->text().toDouble() / 1000, ui_->ledtPosY->text().toDouble() / 1000,
                          ui_->ledtPosZ->text().toDouble() / 1000), true);
}

void PluginController::onFanMotionTypeIndexChanged(int nCurrentIndex) {
    if (0 == nCurrentIndex) {
        ui_->cmbRotationStep->setEnabled(false);   //in continuous mode, disable step value
    }
    else if (1 == nCurrentIndex) {
        ui_->cmbRotationStep->setEnabled(true);
    }
    else {
        LOG_INFO("please check the rotation type combox");
    }
}



//////////////////////////// openCV Part beginning /////////////////////////////////
void PluginController::onReadImageClick()
{
    QString szFilePath = "/home/zhongliang/Project/Pro_OrientationAdj/";
    QString szFileName = "sweep_28_10_18_39_01.imf";

    DataList USImageData = m_confiMapConvex->readImage_IMF(szFilePath, szFileName);
//    setImageDir(&USImageData);
    m_main->dataModel()->add(USImageData.get(Data::UNKNOWN));
    m_disp->update();

    SharedImageSet* sharedImage = USImageData.getImage(Data::UNKNOWN);
    qDebug()<<"sharedImage kind:"<<sharedImage->kind();
    qDebug()<<"sharedImage Modality:"<<sharedImage->modality();
    MemImage* memImage = sharedImage->mem()->clone();

    int nImageHeight = sharedImage->mem()->height();
    int nImageWidth = sharedImage->mem()->width();
    qDebug()<<nImageHeight<<"       "<<nImageWidth;
    cv::Mat cv_imgIn(nImageHeight, nImageWidth, CV_8UC1);  //uchar
    m_confiMapConvex->setHeightRaw(nImageHeight);
    m_confiMapConvex->setWidthRaw(nImageWidth);   //save the size of the raw image

    memcpy(cv_imgIn.data, memImage->data(), memImage->byteSize());

    m_confiMapConvex->setCvMatRaw(cv_imgIn.clone());
    m_confiMapConvex->m_bFlagImgRaw = true;
    cv_imgIn.release();
//    delete(sharedImage);

//    cv::namedWindow("Example", cv::WINDOW_AUTOSIZE);
//    cv::imshow("Example", cv_imgIn);
//    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test.jpeg", cv_imgIn);


}

void PluginController::onSaveImageClick()
{
    //write data to disk
//    auto sharedImage_out = m_main->dataModel()->getImage(Data::UNKNOWN);
//    sharedImage_out->get()->sync();
//    MemImage* memImage = sharedImage_out->mem()->clone();
//    m_confiMapConvex->saveImageToDisc(memImage);
    auto shared_ = m_main->selectedData()[0];
    SharedImageSet* sharedImageSet = static_cast<SharedImageSet*>(shared_);
    Selection sel;
//    sel.setAll(sharedImageSet->size());   //select all images
    sel.setSingle(0);   //select only  one image
    sharedImageSet->setSelection(sel);
    MemImage* memImage = sharedImageSet->mem()->clone();
    m_confiMapConvex->saveImageToDisc(memImage);

}


void PluginController::onRefineFanClick()
{
    if (!m_confiMapConvex->m_bFlagImgRaw)
        {
        LOG_INFO("onRefineFanClick: there is no raw image");
        return;
    }

    std::vector<cv::Point2f> vecWholePosition = m_confiMapConvex->refineFan(pCenterPoint, nShortRadius, nLongRadius);
}


void PluginController::onGenCarFigClick()
{
    if (!m_confiMapConvex->m_bFlagImgRaw)
        {
        LOG_INFO("onRefineFanClick: there is no raw image");
        return;
    }

    cv::Mat cvImageCatesian = m_confiMapConvex->polarToCatesian(pCenterPoint, nShortRadius, nLongRadius);

    //update it into Imfusion
//    cv::Mat cvImageCatesian=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/test_cartesian.jpeg", 0);
//    qDebug()<<"cvImageCatesian->byteSize():"<<cvImageCatesian.total() * cvImageCatesian.elemSize();

    if(cvImageCatesian.empty())
        {
        qDebug()<<"cvImageCatesian is empty";
    }

    int nHeight = cvImageCatesian.rows;
    int nWidth = cvImageCatesian.cols;
    qDebug()<<"nHeight:"<<nHeight<<";   "<<"nWidth:"<<nWidth;
    MemImage* memImage = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
    qDebug()<<"nWidth*nHeight*sizeof(float):"<<nWidth*nHeight*sizeof(uchar);
    qDebug()<<"memImage->byteSize():"<<memImage->typeSize();
    memcpy(memImage->data(), cvImageCatesian.data, nWidth*nHeight*sizeof(uchar));

//    memImage->flip(1);

    Image imageTep(Image::UBYTE, nWidth, nHeight, 1, 1);   //same as previous defined memImage
    qDebug()<<"nHeight:"<<imageTep.height()<<";   "<<"nWidth:"<<imageTep.width();

    //transfer memimage to shareimage
    SharedImage* sharedImageTemp = new SharedImage(imageTep);
    sharedImageTemp->update(*memImage);

    //added sharedimage to sharedimageSet
    SharedImageSet* sharedImageSetTemp = new SharedImageSet();   //new is very important!!
    sharedImageSetTemp->add(sharedImageTemp);
    sharedImageSetTemp->setModality(Data::ULTRASOUND);
    sharedImageSetTemp->get()->setDirtyMem();   //don't know
    //repeating same image three times and then it can be used to transfer to ultradound sweep
    sharedImageSetTemp->add(sharedImageTemp);
    sharedImageSetTemp->setModality(Data::ULTRASOUND);
//    sharedImageSetTemp->add(sharedImageTemp);
//    sharedImageSetTemp->setModality(Data::ULTRASOUND);
    Selection sel;
    sel.setAll(sharedImageSetTemp->size());
    sharedImageSetTemp->setSelection(sel);

    qDebug()<<"sharedImage kind:"<<sharedImageSetTemp->kind();
    qDebug()<<"sharedImage Modality:"<<sharedImageSetTemp->modality();

    //added sharedimageSet to datalist
    DataList* dataListTemp = new DataList();
    dataListTemp->add(sharedImageSetTemp);
    qDebug()<<"dataListTemp:"<<dataListTemp->getImage()->mem()->height()<<";   "<<"nWidth:"<<dataListTemp->getImage()->mem()->width();



//    setImageDir(dataListTemp);
    //display it in imfusion
    m_main->dataModel()->add(*dataListTemp);
    m_disp->update();

//    //used for test how many images in sweep
//    auto sharedImage_out = m_main->dataModel()->getImage(Data::UNKNOWN);
//    sharedImage_out->get()->sync();
//    qDebug()<<"sharedImage_out->byteSize():"<<sharedImage_out->mem()->typeSize();

//    qDebug()<<"sharedImage_out:"<<sharedImage_out->mem()->height()<<";   "<<"nWidth:"<<sharedImage_out->mem()->width();

//ultrasound sweep
//    recordUSimages();

}

void PluginController::onComputeConfiMapClick()
{
    auto shared_ = m_main->selectedData()[0];
    SharedImageSet* sharedImageSet = static_cast<SharedImageSet*>(shared_);
    Selection sel;
//    sel.setAll(sharedImageSet->size());   //select all images
    sel.setSingle(0);   //select only  one image
    sharedImageSet->setSelection(sel);

    DataList* dataList =  m_confiMapConvex->computeConfiMap(sharedImageSet);

    //display
//    setImageDir(dataList);
    m_main->dataModel()->add(dataList->get(Data::UNKNOWN));
    m_disp->update();

    // save confidence map
    MemImage* memImageConfiMap = dataList->getImage()->mem();
    int nImageHeight = memImageConfiMap->height();
    int nImageWidth = memImageConfiMap->width();

    cv::Mat cvMatConfiMap(nImageHeight, nImageWidth, CV_32FC1, cv::Scalar(255));
//    cv::Mat cvMatConfiMap;  //this function doesn't need to give specific type
    ImFusion::OpenCV::convert(memImageConfiMap, cvMatConfiMap);
//    memcpy(cvMatConfiMap.data, memImageConfiMap->data(), memImageConfiMap->byteSize());
    //transfer the calculated confidence map from 32f to 8u, sametime [0,1] to [0,255]
    cv::Mat cvMatConfiMapUChar(nImageHeight, nImageWidth, CV_8UC1, cv::Scalar(255));
    cvMatConfiMap.convertTo(cvMatConfiMapUChar, CV_8UC1, 255);

    qDebug()<<"memImage->byteSize():"<<memImageConfiMap->byteSize();
    qDebug()<<"opencv->byteSize():"<<cvMatConfiMap.total() * cvMatConfiMap.elemSize();

    qDebug()<<"memImage->slices():"<<memImageConfiMap->slices();
    qDebug()<<"memImage->slices():"<<memImageConfiMap->type();
    qDebug()<<"memImage->channels():"<<memImageConfiMap->channels();
    qDebug()<<"memImage->scale():"<<memImageConfiMap->scale();
    qDebug()<<"memImage->dimension():"<<memImageConfiMap->dimension();

    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_confiMap.jpeg", cvMatConfiMapUChar);
}

void PluginController::onGenPolFigClick()
{
////    m_confiMapConvex->catesianToPolar();
//    QString szFilePath = "/home/zhongliang/Project/Pro_OrientationAdj/";
//    QString szFileName = "test_confidencemap.imf";
//    DataList USImageData = m_confiMapConvex->readImage_IMF(szFilePath, szFileName);

//    qDebug()<< "USImageData.size():"<<USImageData.size();
//    m_main->dataModel()->add(USImageData.get(Data::UNKNOWN));
//    m_disp->update();

//    std::vector<SharedImageSet*> sharedImageSets = USImageData.getImages(Data::UNKNOWN);
//    qDebug()<< "sharedImageSets.size():"<<sharedImageSets.size();



//    SharedImageSet* sharedImage = USImageData.getImage(Data::UNKNOWN);
//    qDebug()<<"sharedImage kind:"<<sharedImage->kind();
//    qDebug()<<"sharedImage Modality:"<<sharedImage->modality();
//    MemImage* memImage = sharedImage->mem()->clone();
//    qDebug()<<"memImage->channels():"<<memImage->channels();
//    memImage->convertToGray();
//    qDebug()<<"memImage->channels():"<<memImage->channels();

//    int nImageHeight, nImageWidth;
//    nImageHeight = memImage->height();
//    nImageWidth = memImage->width();
//    qDebug()<<nImageHeight<<"       "<<nImageWidth;
//    cv::Mat cvConfidenceMap;
//    cvConfidenceMap = cv::Mat(nImageHeight, nImageWidth, CV_8UC1);
//    qDebug()<<"memImage->byteSize():"<<memImage->byteSize();
//    qDebug()<<"memImage->byteSize():"<<cvConfidenceMap.total() * cvConfidenceMap.elemSize();
////    memImage->s
////    memcpy(cvConfidenceMap.data, memImage->data(), memImage->byteSize());


    ////////////because there type size of imf file is 4 time than previous one.
    cv::Mat cvImageConfiPolar = m_confiMapConvex->catesianToPolar(pCenterPoint, nShortRadius, nLongRadius);

    int nHeight = cvImageConfiPolar.rows;
    int nWidth = cvImageConfiPolar.cols;

    MemImage* memImage = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
    memcpy(memImage->data(), cvImageConfiPolar.data, nWidth*nHeight*sizeof(uchar));

    SharedImageSet* sharedImageSet = new SharedImageSet(static_cast<Image*>(memImage));
    sharedImageSet->add(static_cast<Image*>(memImage));
    DataList* dataList = new DataList();
    dataList->add(static_cast<Data*>(sharedImageSet));

//    setImageDir(dataList);
    //display it in imfusion
    m_main->dataModel()->add(*dataList);
    m_disp->update();

}

//calculate from live us Image
void PluginController::onConfiMapInterClick()
{
    //recorded 2 image from live us image
    DataList* dataListLive = new DataList();
    bool bRecoder = recordUSimages(dataListLive);
    if (false == bRecoder)
        {return;}
    
    //recorded the basic information of the raw image
    MemImage*  memImageLive = dataListLive->getImage()->mem();
    int nImageHeight = memImageLive->height();
    int nImageWidth = memImageLive->width();
    m_confiMapConvex->setHeightRaw(nImageHeight);
    m_confiMapConvex->setWidthRaw(nImageWidth);   //save the size of the raw image

    cv::Mat cvImageLive(nImageHeight, nImageWidth, CV_8UC1);  //uchar
    ImFusion::OpenCV::convert(memImageLive, cvImageLive);
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_live.jpeg", cvImageLive);

    m_confiMapConvex->setCvMatRaw(cvImageLive.clone());
    m_confiMapConvex->m_bFlagImgRaw = true;

//    cvImageLive.release();
////    delete(dataList);  //cause crash
////    delete(memImageLive);


    //generate cartesian image
    cv::Mat cvImageCatesian = m_confiMapConvex->polarToCatesian(pCenterPoint, nShortRadius, nLongRadius);
    cv::flip(cvImageCatesian, cvImageCatesian, 0);

    ///update it into Imfusion
    if(cvImageCatesian.empty())
        {qDebug()<<"cvImageCatesian is empty";}

    int nHeight = cvImageCatesian.rows;
    int nWidth = cvImageCatesian.cols;
    qDebug()<<"nHeight:"<<nHeight<<";   "<<"nWidth:"<<nWidth;
    MemImage* memImageCar = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
    qDebug()<<"nWidth*nHeight*sizeof(float):"<<nWidth*nHeight*sizeof(uchar);
    qDebug()<<"memImage->byteSize():"<<memImageCar->typeSize();
    memcpy(memImageCar->data(), cvImageCatesian.data, nWidth*nHeight*sizeof(uchar));

    ///display cartesian figure
    SharedImageSet* sharedImageSetCar = new SharedImageSet(static_cast<Image*>(memImageCar));
    sharedImageSetCar->add(static_cast<Image*>(memImageCar));

    Selection sel;
//    sel.setAll(sharedImageSet->size());   //select all images
    sel.setSingle(0);   //select only  one image
    sharedImageSetCar->setSelection(sel);
    sharedImageSetCar->setModality(Data::ULTRASOUND);

    DataList* dataListCar = new DataList();
    dataListCar->add(static_cast<Data*>(sharedImageSetCar));

//    setImageDir(dataListCar);
    m_main->dataModel()->add(*dataListCar);
    m_disp->update();

    //calculate the confidence map
//    Selection sel;
////    sel.setAll(sharedImageSet->size());   //select all images
//    sel.setSingle(0);   //select only  one image
//    sharedImageSetCar->setSelection(sel);

//    sharedImageSetCar->mem()->flip(1);
    DataList* dataListConfiMap =  m_confiMapConvex->computeConfiMap(sharedImageSetCar);

    //display
//    setImageDir(dataList);
    m_main->dataModel()->add(dataListConfiMap->get(Data::UNKNOWN));
    m_disp->update();

    // save confidence map
    MemImage* memImageConfiMap = dataListConfiMap->getImage()->mem();
    nImageHeight = memImageConfiMap->height();
    nImageWidth = memImageConfiMap->width();

    cv::Mat cvMatConfiMap(nImageHeight, nImageWidth, CV_32FC1, cv::Scalar(255));
//    cv::Mat cvMatConfiMap;  //this function doesn't need to give specific type
    ImFusion::OpenCV::convert(memImageConfiMap, cvMatConfiMap);
//    memcpy(cvMatConfiMap.data, memImageConfiMap->data(), memImageConfiMap->byteSize());
    //transfer the calculated confidence map from 32f to 8u, sametime [0,1] to [0,255]
    cv::Mat cvMatConfiMapUChar(nImageHeight, nImageWidth, CV_8UC1, cv::Scalar(255));
    cvMatConfiMap.convertTo(cvMatConfiMapUChar, CV_8UC1, 255);

    qDebug()<<"memImage->byteSize():"<<memImageConfiMap->byteSize();
    qDebug()<<"opencv->byteSize():"<<cvMatConfiMap.total() * cvMatConfiMap.elemSize();

    qDebug()<<"memImage->slices():"<<memImageConfiMap->slices();
    qDebug()<<"memImage->slices():"<<memImageConfiMap->type();
    qDebug()<<"memImage->channels():"<<memImageConfiMap->channels();
    qDebug()<<"memImage->scale():"<<memImageConfiMap->scale();
    qDebug()<<"memImage->dimension():"<<memImageConfiMap->dimension();

    cv::flip(cvMatConfiMapUChar, cvMatConfiMapUChar, 0);
    cv::imwrite("/home/zhongliang/Project/Pro_OrientationAdj/test_confiMap.jpeg", cvMatConfiMapUChar);


    //back to polar
    cv::Mat cvImageConfiPolar = m_confiMapConvex->catesianToPolar(pCenterPoint, nShortRadius, nLongRadius);

    nHeight = cvImageConfiPolar.rows;
    nWidth = cvImageConfiPolar.cols;

    MemImage* memImageConfiPolar = MemImage::create(Image::UBYTE, nWidth, nHeight, 1, 1);
    memcpy(memImageConfiPolar->data(), cvImageConfiPolar.data, nWidth*nHeight*sizeof(uchar));

    SharedImageSet* sharedImageSetConfiPolar = new SharedImageSet(static_cast<Image*>(memImageConfiPolar));
    sharedImageSetConfiPolar->add(static_cast<Image*>(memImageConfiPolar));
    DataList* dataListConfiPolar = new DataList();
    dataListConfiPolar->add(static_cast<Data*>(sharedImageSetConfiPolar));

//    setImageDir(dataList);
    //display it in imfusion
    m_main->dataModel()->add(*dataListConfiPolar);
    m_disp->update();



    cv::Mat cvImage=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/test_confiMap.jpeg", 0);
    m_confiMapConvex->computeBaryCenter(cvImage);
    ui_->pbtnGoResult->setEnabled(true);





    /////////////////////////////////////////////////////////////////

//    Selection sel;
////    sel.setAll(sharedImageSet->size());   //select all images
//    sel.setSingle(0);   //select only  one image
//    sharedImageSet->setSelection(sel);
}

void PluginController::onGoResultClick()
{
    double RotationAngle = -m_confiMapConvex->m_dConfidenceAng;
    //check the absolute rantationAngle no greater than 10 degree
    if(fabs(RotationAngle)>35.0) {
        LOG_ERROR("The rotation angle is larger than 35 degree");
        return;
    }
//    int nRotationAxis = ui_->cmbFanStartRotationAxis->currentIndex();   //0:X; 1:Y
//    if(0==nRotationAxis) {
//        algorithm_->RotateAroundTCP(RotationAngle, ROTATION_X);

//    } else if(1==nRotationAxis) {
//        algorithm_->RotateAroundTCP(RotationAngle, ROTATION_Y);
//    } else {
//        LOG_INFO("Whats wrong??? go the start point of fan motion");
//    }
    algorithm_->RotateAroundTCP(RotationAngle, ROTATION_X);
    ui_->pbtnGoResult->setEnabled(false);
}

//recorded the us images form live US scanning
bool PluginController::recordUSimages(DataList* dataList)
{
//    LiveTrackingStream* liveUsStream = static_cast<LiveTrackingStream*>(m_main->dataModel()->get("Robot Tracking"));
    ImageStream* usStream = static_cast<ImageStream*>(m_main->dataModel()->get("Ultrasound Stream - IGTL"));
    std::vector<Stream*> vecStream;
    vecStream.push_back(usStream);
    USSweepRecorderAlgorithm* usSweepRecorder = new USSweepRecorderAlgorithm(vecStream);
    usSweepRecorder->start();

    int nIteration = 0;
    //2 is because the confidence map is only worked for us sweep or multi us images
    while (usSweepRecorder->recordedFrames()<2)
    {
        nIteration ++;
        if (nIteration >30000000)    //3286847 is get from test
            {
            LOG_WARN("recordUSimages: cannot recorded image from live US system");
            return false;
        }
    }
    qDebug()<<"nIteration: "<<nIteration;

    usSweepRecorder->stop();   //if doesn't recorded anything, we still need to stop
    std::cout<<"recordedBytes:"<<usSweepRecorder->recordedBytes();

    dataList->deleteAll();   //make sure only this data
    usSweepRecorder->output(*dataList);
    dataList->get(Data::UNKNOWN)->setName("Recorded Ultrasound");

//    setImageDir(dataList);

    m_main->dataModel()->add(*dataList);
    m_disp->update();
    return true;
}

void PluginController::setImageDir(DataList* dataList)
{
    int nHeight = dataList->getImage(Data::UNKNOWN)->mem()->height();   //y
    int nWidth = dataList->getImage(Data::UNKNOWN)->mem()->width();   //x

    int x = floor(nWidth/2.0);
    double nAllValue(0);
    int temp(0);
    //calculate the up half part gray value (y(0,..))
    for(int i = 0; i < floor(nHeight/2.0); i++)
        {
        nAllValue += dataList->getImage(Data::UNKNOWN)->mem()->valueDouble(x,i);
        dataList->getImage(Data::UNKNOWN)->mem()->setValueDouble(255, x,i);
        temp = nHeight - i;
        nAllValue = nAllValue - dataList->getImage(Data::UNKNOWN)->mem()->valueDouble(x, temp);
        dataList->getImage(Data::UNKNOWN)->mem()->setValueDouble(100, x, temp);
    }

    if (nAllValue < 0)
    {
        qDebug()<<"nAllValue:"<<nAllValue;
        dataList->getImage(Data::UNKNOWN)->mem()->flip(1);   //1: up-right; 0: left-right
    }

}


//////////////////////////// openCV Part ending/////////////////////////////////



void PluginController::onGoFanStartClick() {

    double RotationAngle = ui_->ledtFanStartAngle->text().toDouble();
    //check the absolute rantationAngle no greater than 10 degree
    if(fabs(RotationAngle)>35.0) {
        LOG_ERROR("The rotation angle is larger than 20 degree");
        return;
    }
    int nRotationAxis = ui_->cmbFanStartRotationAxis->currentIndex();   //0:X; 1:Y
    if(0==nRotationAxis) {
        algorithm_->RotateAroundTCP(RotationAngle, ROTATION_X);

    } else if(1==nRotationAxis) {
        algorithm_->RotateAroundTCP(RotationAngle, ROTATION_Y);
    } else {
        LOG_INFO("Whats wrong??? go the start point of fan motion");
    }

}

void PluginController::onFanMotionClick() {
    //current position;
    if (algorithm_->isRobotConnected()) {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix();
        algorithm_->m_initialFrameFan = robot_pose;
        int nRoatationAxis = ui_->cmbRotationAxis->currentIndex();   //0:x; 1:Y
        int nRotationStep = ui_->cmbRotationStep->currentIndex()+1;
        QString szFanMotionType = ui_->cmbFanMotionType->currentText();
//        std::cout <<szFanMotionType.toUtf8().constData() <<std::endl;
        if(QString("Continous Fan Motion") == szFanMotionType) {
            algorithm_->fanShapMotion(10.0, nRotationStep, nRoatationAxis);
        }
        else if (QString("Step Fan Motion") == szFanMotionType) {
            algorithm_->fanShapMotion(10.0, nRotationStep, nRoatationAxis, FAN_MOTION_STYPE_STEP);
        }
        else {
            LOG_INFO("Please slect the motion type first!");
        }
    }
}

void PluginController::onPositionModeClick() {
  LOG_INFO("Applying position model!");
  algorithm_->applyPositionControlMode();
}

void PluginController::onForceModeClick() {
  LOG_INFO("Applying Force!");
  algorithm_->applyDesiredForce(iiwa_msgs::DOF::Z, ui_->ledtDesiredForce->text().toDouble(),
                                ui_->ledtStiffness->text().toDouble());
}

//this is temporary used as stop function
void PluginController::onStop() {
    if (algorithm_->isRobotConnected()) {
        int nStepFanMotionIteration = algorithm_->onGetStepFanMotionIteration();
        if (nStepFanMotionIteration > 0) {
            algorithm_->stopStepFanMotion();
        }
        else {
            algorithm_->executeCartesianCommand(algorithm_->getCurrentRobotPose().pose, false);
        }
    }
}

void PluginController::onSaveOut() {
    if (algorithm_->isRobotConnected()) { algorithm_->setRobotOutConfiguration(algorithm_->getCurrentRobotPose()); }
}

void PluginController::onSaveHome() {
    if (algorithm_->isRobotConnected()) {algorithm_->setRobotHomeConfiguration(algorithm_->getCurrentRobotPose());}
}

void PluginController::onGoHome() {
    LOG_INFO("enter onGoHome");
  algorithm_->executeCartesianCommand(algorithm_->getRobotHomeConfiguration().pose, true);
}

void PluginController::onGoOut() {
  algorithm_->executeCartesianCommand(algorithm_->getRobotOutConfiguration().pose, true);
}

void PluginController::onConnectToRobotClick() {
    algorithm_->connect("IFLConvex");
    algorithm_->addStreamToDataModel(m_main->dataModel());

    emit robotConnected();  //! Emit connection signal.
}

void PluginController::onDisconnectFromRobotClick() {
  algorithm_->disconnect();

  emit robotDisconnected();  //! Emit disconnection signal.
}



void PluginController::updateUIToLastPose(){
    if (algorithm_->isRobotConnected()) {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad
        //(0,1,2)表示分别绕XYZ轴顺序，即pitch yaw roll顺序，逆时针为正


        if (ui_->chbUpdateState->isChecked()) {
            ui_->ledtPosX->setText(QString::number(translation.x()));
            ui_->ledtPosY->setText(QString::number(translation.y()));
            ui_->ledtPosZ->setText(QString::number(translation.z()));
            ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
            ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
            ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));
        }
    }
}

void PluginController::updateUIToLastWrench() {
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();

        if (ui_->chbUpdateState->isChecked()) {
            ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
            ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
            ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
            ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
            ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
            ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));
        }
//        if (ui_->chbRecordWrench->isChecked()) {

//            auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

//            //calculate the reference angle position between real time frame and inital frame
//            Eigen::Matrix3d  initialFrameFan = algorithm_->m_initialFrameFan.block<3, 3>(0, 0);
//            Eigen::Matrix3d  currentFrameFan = robot_pose.block<3, 3>(0, 0);
//            Eigen::Vector3d reletiveAngle = algorithm_->calculateReleativeRatationAngle(initialFrameFan, currentFrameFan);

//            QString path = QDir::currentPath();
//            QDir dir;
//            path=path +QString("/ros/zhongliang/");
//            if (!dir.exists(path))
//                dir.mkpath(path); // create the directory if needed
//            QFile file(path + "ForceData.txt");
//            QString szData;
//            std::vector<float> forceData = m_forceData;
//            if (file.open(QIODevice::ReadWrite | QIODevice::Append)) {
//                QTextStream streamOut(&file);
//                streamOut /*<< szData.setNum(robot_wrench.wrench.force.x)
//                                  << "       "  //space of two TAB*/
//                        << QString::number(robot_wrench.force.x)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_wrench.force.y)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_wrench.force.z)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_wrench.torque.x)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_wrench.torque.y)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_wrench.torque.z)
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(0,0))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(1,0))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(2,0))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(0,1))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(1,1))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(2,1))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(0,2))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(1,2))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(2,2))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(0,3))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(1,3))
//                        << "       "  //space of two TAB
//                        << QString::number(robot_pose(2,3))
//                        << "       "  //space of two TAB
//                        << QString::number(reletiveAngle(0))
//                        << "       "  //space of two TAB
//                        << QString::number(reletiveAngle(1))
//                        << "       "  //space of two TAB
//                        << QString::number(reletiveAngle(2))
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[0])
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[1])
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[2])
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[3])
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[4])
//                        << "       "  //space of two TAB
//                        << QString::number(forceData[5])
//                        << "       "  //space of two TAB
//                        << endl;
//            }
//            file.close();
//        }

    }
}

//save the direction find by volunteer
void PluginController::onSaveDirectionClick()
{
    if (algorithm_->isRobotConnected()) {
        static int counter = 2;
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

        QString path = QDir::currentPath();
        QDir dir;
        path=path +QString("/ros/zhongliang/");
        if (!dir.exists(path))
            dir.mkpath(path); // create the directory if needed
        QFile file(path + "SavedDirection.txt");
        QString szData;
        if (file.open(QIODevice::ReadWrite | QIODevice::Append)) {
            QTextStream streamOut(&file);
            streamOut /*<< szData.setNum(robot_wrench.wrench.force.x)
                                          << "       "  //space of two TAB*/
                    << QString::number(robot_pose(0,2))
                    << "       "  //space of two TAB
                    << QString::number(robot_pose(1,2))
                    << "       "  //space of two TAB
                    << QString::number(robot_pose(2,2))
                    << endl;
        }
        file.close();
        ui_->pbtnSaveDirection->setText(QString("save direction %1").arg(counter));
        counter++;
    }
}

void PluginController::onTestClick() {
//    Eigen::Vector3d rotAngles(ui_->ledtAngRa->text().toDouble(), ui_->ledtAngRb->text().toDouble(),
//                              ui_->ledtAngRc->text().toDouble());
//    rotAngles *= M_PI / 180.;   //rad
////    LOG_INFO(Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()));
////    LOG_INFO(Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()));
////        Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()) *
////            Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()) *
////            Eigen::AngleAxisd(rotAngles[2], Eigen::Vector3d::UnitZ()),

//    Eigen::AngleAxisd m;
//    m = Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX()) *
//            Eigen::AngleAxisd(rotAngles[1], Eigen::Vector3d::UnitY()) *
//            Eigen::AngleAxisd(rotAngles[2], Eigen::Vector3d::UnitZ());

//    Eigen::AngleAxisd(rotAngles[0], Eigen::Vector3d::UnitX());


//    Eigen::AngleAxisd V1(rotAngles[0], Eigen::Vector3d(0, 0, 1));//以（0,0,1）为旋转轴，旋转45度
//        std::cout << "Rotation_vector1" << std::endl << m.matrix() << std::endl;

//    LOG_INFO(rotAngles);
//    Eigen::Matrix4d a;
//    a<<1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8;
//    std::cout << "a" << std::endl << a << std::endl;
////    Eigen::Matrix2i a; a << 1, 2, 3, 4;
//    QVector<Eigen::Matrix4d> vecPoses;
//    vecPoses.append(a);
//    std::cout << vecPoses.at(0) << std::endl;
//    algorithm_->calculateEndPointsTCP(10);


    cv::Mat cvImage=cv::imread("/home/zhongliang/Project/Pro_OrientationAdj/test_confiMap.jpeg", 0);

//    cv::Mat cvImage(2,2,CV_8UC1, cv::Scalar(255));
    m_confiMapConvex->computeBaryCenter(cvImage);
//    m_confiMapConvex->calFigureDerivation(cvImage);


}



//TCP
void PluginController::onRecForce()
{
     tcpServer->listen(QHostAddress::Any,9999);
     connect(tcpServer, SIGNAL(newConnection()), this, SLOT(acceptConnection()));
}

void PluginController::acceptConnection()
{
    serverSocket = tcpServer->nextPendingConnection();
    connect(serverSocket,SIGNAL(readyRead()),this,SLOT(replyToClient()));
}

void PluginController::replyToClient()
{
    qDebug() <<"here C1 bytes = " << serverSocket->bytesAvailable();
    if(serverSocket->bytesAvailable()>100)
    {
        qDebug() <<"MSG:" <<serverSocket->readAll();   //avoid crash
        serverSocket->flush();
        return;
    }
    std::string msg= std::string(serverSocket->readAll());
    std::string delimiter = ";";

    size_t pos = 0;
    int nindex(0); // represent the index in force

    while ((pos = msg.find(delimiter)) != std::string::npos) {
        m_forceData[nindex] = std::stof(msg.substr(0, pos));
        msg.erase(0, pos + delimiter.length());
        nindex++;
    }
    LOG_INFO(m_forceData.at(1));
}


//Synchronous acquisition
void PluginController::onSynRecord()
{
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));

        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad
        //(0,1,2)表示分别绕XYZ轴顺序，即pitch yaw roll顺序，逆时针为正


        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y()));
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));



        if (ui_->chbRecordWrench->isChecked()) {

            auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);

            //calculate the reference angle position between real time frame and inital frame
            Eigen::Matrix3d  initialFrameFan = algorithm_->m_initialFrameFan.block<3, 3>(0, 0);
            Eigen::Matrix3d  currentFrameFan = robot_pose.block<3, 3>(0, 0);
            Eigen::Vector3d reletiveAngle = algorithm_->calculateReleativeRatationAngle(initialFrameFan, currentFrameFan);

            QString path = QDir::currentPath();
            QDir dir;
            path=path +QString("/ros/zhongliang/");
            if (!dir.exists(path))
                dir.mkpath(path); // create the directory if needed
            QFile file(path + "ForceData.txt");
            QString szData;
            std::vector<float> forceData = m_forceData;
            if (file.open(QIODevice::ReadWrite | QIODevice::Append)) {
                QTextStream streamOut(&file);
                streamOut /*<< szData.setNum(robot_wrench.wrench.force.x)
                                                              << "       "  //space of two TAB*/
                        << QString::number(robot_wrench.force.x)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.force.y)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.force.z)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.x)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.y)
                        << "       "  //space of two TAB
                        << QString::number(robot_wrench.torque.z)
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,0))   //x
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,1))   //y
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,2))   //z
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(0,3))
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(1,3))
                        << "       "  //space of two TAB
                        << QString::number(robot_pose(2,3))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(0))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(1))
                        << "       "  //space of two TAB
                        << QString::number(reletiveAngle(2))
                        << "       "  //space of two TAB
                        << QString::number(forceData[0])
                        << "       "  //space of two TAB
                        << QString::number(forceData[1])
                        << "       "  //space of two TAB
                        << QString::number(forceData[2])
                        << "       "  //space of two TAB
                        << QString::number(forceData[3])
                        << "       "  //space of two TAB
                        << QString::number(forceData[4])
                        << "       "  //space of two TAB
                        << QString::number(forceData[5])
                        << "       "  //space of two TAB
                        << endl;
            }
            file.close();
        }
    }
}

void PluginController::onchbRecordStateChanged()
{
    if(true == ui_->chbRecordWrench->isChecked())
    {
        m_recordTimer->start(TIM_INTERVAL_REC);
    }
    else {
        m_recordTimer->stop();
    }

}

void PluginController::onUpdateState()
{
    if (algorithm_->isRobotConnected()) {

        auto robot_wrench = algorithm_->getCurrentRobotWrench();

        ui_->ledtForceX->setText(QString::number(robot_wrench.force.x));
        ui_->ledtForceY->setText(QString::number(robot_wrench.force.y));
        ui_->ledtForceZ->setText(QString::number(robot_wrench.force.z));
        ui_->ledtToqueX->setText(QString::number(robot_wrench.torque.x));
        ui_->ledtToqueY->setText(QString::number(robot_wrench.torque.y));
        ui_->ledtToqueZ->setText(QString::number(robot_wrench.torque.z));

    }

    if (algorithm_->isRobotConnected()) {
        auto robot_pose = algorithm_->getCurrentRobotTransformMatrix(true);
        Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);   //mm
        Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //rad
        //(0,1,2)表示分别绕XYZ轴顺序，即pitch yaw roll顺序，逆时针为正


        ui_->ledtPosX->setText(QString::number(translation.x()));
        ui_->ledtPosY->setText(QString::number(translation.y()));
        ui_->ledtPosZ->setText(QString::number(translation.z()));
        ui_->ledtAngRa->setText(QString::number(eulerAngles.x() * 180. / M_PI));
        ui_->ledtAngRb->setText(QString::number(eulerAngles.y() * 180. / M_PI));
        ui_->ledtAngRc->setText(QString::number(eulerAngles.z() * 180. / M_PI));

    }
}

void PluginController::onChbUpdateStateChanged()
{
    if(true == ui_->chbUpdateState->isChecked())
    {
        m_updateTimer->start(TIM_INTERVAL_UPDATE);
    }
    else {
        m_updateTimer->stop();
    }
}



}  // namespace OrienAdj
}  // namespace ImFusion

