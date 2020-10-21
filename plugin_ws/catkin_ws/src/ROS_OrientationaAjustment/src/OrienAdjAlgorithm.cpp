#include "orien_adj/OrienAdjAlgorithm.h"

#include <ImFusion/Base/Log.h>
#include <ImFusion/Stream/OpenIGTLinkConnection.h>
#include <ImFusion/Stream/TrackingStreamData.h>
#include <QDebug>

#include <fstream>
#include <QVector3D>

namespace ImFusion {
namespace OrienAdj {

PluginAlgorithm::PluginAlgorithm() {
  // TODO: why this?
  probe_rotation_.block<3, 3>(0, 0) = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).matrix() *
                                      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).matrix() *
                                      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).matrix();
}

void PluginAlgorithm::connect(const std::string &probe_name) {
    //innitialize Ros and iiwaRos object
    std::map<std::string, std::string> emptyArgs;
    if (!ros::isInitialized()) { ros::init(emptyArgs, "iiwaRos"); }
    ros_spinner_ = std::make_unique<ros::AsyncSpinner>(1);
    ros_spinner_->start();

    ros::NodeHandle node_handle;

    pose_state_.init("iiwa", std::bind(&PluginAlgorithm::poseCallback, this, std::placeholders::_1));
    wrench_state_.init("iiwa", std::bind(&PluginAlgorithm::wrenchCallback, this, std::placeholders::_1));
    torque_state_.init("iiwa");

    pose_command_.init("iiwa");
    linear_pose_command_.init("iiwa");

    control_mode_.init("iiwa");



    OpenIGTLinkConnection dummy_connection("Service robot connection");
    tracking_stream_ = new OpenIGTLinkTrackingStream(dummy_connection, "Robot");
    tracking_stream_->open();
    is_robot_connected_ = true;
    loadCalibrationFromFile("IFLUSCalibration.config", probe_name);

    emit robotConnected();
}

void PluginAlgorithm::disconnect() {
  if (ros::ok()) {
    if (ros_spinner_ != nullptr) {
      ros_spinner_->stop();
      ros_spinner_ = nullptr;
    }
  }

  //! If the stream was never passed to a DataModel, we have to dispose it.
  if (owning_stream_ && tracking_stream_ != nullptr) { delete tracking_stream_; }
  is_robot_connected_ = false;
  emit robotDisconnected();
}

PluginAlgorithm::~PluginAlgorithm() { disconnect(); }

bool PluginAlgorithm::createCompatible(const DataList& data, Algorithm** a) {
  if (data.size() != 0) { return false; }
  if (a) { *a = new PluginAlgorithm(); }
  return true;
}

void PluginAlgorithm::compute() {
}

void PluginAlgorithm::configure(const Properties* p) {
  if (p == nullptr) { return; }
  p->param("something", something_);
}

void PluginAlgorithm::configuration(Properties* p) const {
  if (p == nullptr) { return; }
  p->setParam("something", something_);
}

void PluginAlgorithm::doSomething() { emit somethingHappened(); }

void PluginAlgorithm::poseCallback(const iiwa_msgs::CartesianPose& pose){
    if (is_robot_connected_){
        std::lock_guard<std::mutex> lock{pose_mutex_};
        current_tip_pose_ = pose.poseStamped;   //this would be updated after connect automatically

        //! This object is disposed by the destructor of TrackingStreamData, or at least so it says its documentation.
        //! If you try to handle its lifetime -> CRASH, so leave it as it is.
        TrackingInstrument* tracking_instrument = new TrackingInstrument();
        tracking_instrument->active = true;
        tracking_instrument->name = "US";
        tracking_instrument->quality = 1;
        auto image_center_pose = poseToEigenMat4(pose.poseStamped.pose, 1000) * probe_rotation_ * ultrasound_calibration_;

        tracking_instrument->matrix = image_center_pose;

        current_image_center_pose_.pose = eigenMat4ToPose(image_center_pose);
        current_image_center_pose_.header = pose.poseStamped.header;

        std::vector<TrackingInstrument*> instrument_vector{tracking_instrument};
        TrackingStreamData datas(tracking_stream_, instrument_vector);

        std::chrono::system_clock::time_point arrivalTime = std::chrono::high_resolution_clock::now();
        datas.setTimestampArrival(arrivalTime);

        tracking_stream_->sendStreamData(datas);

        emit poseChanged();
      }
    }

void PluginAlgorithm::wrenchCallback(const iiwa_msgs::CartesianWrench &wench) {
    if (is_robot_connected_){
        std::lock_guard<std::mutex> lock{wrench_mutex_};
        current_tip_wrench_ = wench.wrench;   //this would be updated after connect automatically

        emit wrenchChanged();
      }
    }

void PluginAlgorithm::movementFinishCallback() {
    LOG_INFO("enter movementFinishCallback");
    switch (m_fanStatus) {
    case ON_CURRENT_POSE:
        LOG_INFO("go to initial pose");
        onGotoPose(m_initialPose);
        m_fanStatus = ON_INITIAL_POSE;
        break;

    case ON_INITIAL_POSE:
        LOG_INFO("go to final pose");
        onGotoPose(m_finalPose);
        m_fanStatus = ON_FINAL_POSE;
        break;

    case ON_FINAL_POSE:
        LOG_INFO("go to initial pose again");
        onGotoPose(m_initialPose);
        m_fanStatus = ON_BACK_INITIAL_POSE;
        break;

    case ON_BACK_INITIAL_POSE:
        LOG_INFO("Finished");
        m_fanStatus = -1;
        break;
    }
}

void PluginAlgorithm::stepMovementFinishCallback() {
    LOG_INFO("enter stepMovementFinishCallback");
    int n_poseNumber = m_vecPose.size();
    if (m_nIteration < n_poseNumber && m_nIteration >= 0) {
        LOG_INFO(m_nIteration);
        LOG_INFO(n_poseNumber);
        onStepGotoPose(m_vecPose.at(m_nIteration));
        m_nIteration++;
    }
    else {
        LOG_INFO("finished");
        m_nIteration = 0;
        m_vecPose.resize(0);
    }
}

//stop at current point and stop go into callback function
void PluginAlgorithm::stopStepFanMotion() {
    int n_poseNumber = m_vecPose.size();
    if (m_nIteration < n_poseNumber && m_nIteration >0) {
        m_nIteration = -1;   //to avoid move around become 1 after add 1
    }
}

void PluginAlgorithm::executeCartesianCommand(const geometry_msgs::Pose& pose, bool linear,
                                              const std::function<void()>& callback) {
    LOG_INFO("enter executeCartesianCommand");
    if (is_robot_connected_) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "iiwa_link_0";
    ps.pose = pose;

    if (linear == true) {
      if (callback == nullptr) {
        linear_pose_command_.setPose(ps);
      } else {
        linear_pose_command_.setPose(ps, callback);
      }
    } else {
      if (callback == nullptr) {
        pose_command_.setPose(ps);
      } else {
        pose_command_.setPose(ps, callback);
      }
    }
  } else {
    LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
  }
}

void PluginAlgorithm::executeCartesianCommand(const Eigen::Matrix4d& matrix, bool linear,
                                              const std::function<void()>& callback) {
  executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
}

void PluginAlgorithm::executeCartesianCommand(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool linear,
                                              std::function<void()> callback) {
  Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = t;
  executeCartesianCommand(eigenMat4ToPose(matrix), linear, callback);
}

// THIS IS IN METERS
geometry_msgs::PoseStamped PluginAlgorithm::getCurrentRobotPose() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  return current_tip_pose_;
}

// THIS IS IN METERS
Eigen::Matrix4d PluginAlgorithm::getCurrentRobotTransformMatrix(bool in_millimeters) {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  double scaling_factor{1};
  if (in_millimeters) { scaling_factor = 1000; }
  return poseToEigenMat4(current_tip_pose_.pose, scaling_factor);
}

geometry_msgs::PoseStamped PluginAlgorithm::getCurrentImageCenterPose() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{pose_mutex_};
  return current_image_center_pose_;
}

//wrench
geometry_msgs::Wrench PluginAlgorithm::getCurrentRobotWrench() {
  assert(is_robot_connected_ == true &&
         "The robot has to be connected before receiving any state. Call the 'connect()' method.");
  std::lock_guard<std::mutex> lock{wrench_mutex_};
  return current_tip_wrench_;
}


geometry_msgs::Pose PluginAlgorithm::eigenMat4ToPose(Eigen::Matrix4d matrix, double scaling_factor){
    Eigen::Quaterniond q{matrix.block<3, 3>(0, 0)};
    Eigen::Vector3d t{matrix.block<3, 1>(0,3)};  // block matrix:from (0,3) select a (3,1) matrix

    geometry_msgs::Pose pose;
    pose.position.x = t[0] * scaling_factor;
    pose.position.y = t[1] * scaling_factor;
    pose.position.z = t[2] * scaling_factor;

    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}

Eigen::Matrix4d PluginAlgorithm::poseToEigenMat4(const geometry_msgs::Pose &pose, double scaling_factor){
    Eigen::Matrix4d matrix{Eigen::Matrix4d::Identity()};
    matrix.block<3, 3>(0, 0) =
            Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            .toRotationMatrix();
    matrix.block<4, 1>(0, 3) = Eigen::Vector4d(pose.position.x * scaling_factor, pose.position.y * scaling_factor,
                                               pose.position.z * scaling_factor, 1);
    return matrix;
}

void PluginAlgorithm::loadCalibrationFromFile(const std::string& file_name, const std::string& desired_probe) {
  std::string config_file_path = configuration_dir + file_name;
  std::ifstream config_file(config_file_path);
  if (config_file.is_open()) {
    std::string probe_name;
    while (std::getline(config_file, probe_name)) {
      for (int x = 0; x < 4; x++) {
        std::string line;
        std::getline(config_file, line);
        std::istringstream iss(line);
        for (int y = 0; y < 4; y++) {
          std::string s;
          std::getline(iss, s, ';');
          ultrasound_calibration_(x, y) = std::stod(s);
        }
      }
      std::string line;
      std::getline(config_file, line);
      temporal_calibration_ = std::stof(line);
      if (probe_name.compare(desired_probe) == 0) {
        config_file.close();
        LOG_INFO("" << std::endl
                    << "Calibration found at " << config_file_path << " for probe " << desired_probe << ": "
                    << std::endl
                    << ultrasound_calibration_ << std::endl
                    << "Temporal Calibration: " << temporal_calibration_ << std::endl
                    << std::endl);
        return;
      }
    }
  } else {
    LOG_ERROR("Unable to open file");
  }
  LOG_ERROR("Couldn't find a calibration file at " << config_file_path << " for " << desired_probe
                                                   << ", I will load identity. " << std::endl);
  ultrasound_calibration_ = Eigen::MatrixXd::Identity(4, 4);
  LOG_INFO(ultrasound_calibration_ << std::endl);
}

////////////////////////////JZL OrienAdj task
void PluginAlgorithm::fanShapMotion(double doffsetAngle, double dStepAngle, int nRoationAxis, int nFanMotionType) {
///  just need to adjust the angle of Ra!
    LOG_INFO("Enter into FanMotion");
    if (is_robot_connected_) {
        LOG_INFO(nFanMotionType);
        if(FAN_MOTION_STYPE_CONTINOUS == nFanMotionType) {
            LOG_INFO("Enter into FanMotionC");
//            calculateEndPoints(doffsetAngle, nRoationAxis);   //rotate around base
            calculateEndPointsTCP(doffsetAngle, nRoationAxis);   //rotate around TCP
            movementFinishCallback();
        }
        else if(FAN_MOTION_STYPE_STEP == nFanMotionType) {
            LOG_INFO("Enter into FanMotionStep");
//            calculateEndPointsStep(doffsetAngle, dStepAngle, nRoationAxis);   //rotate around base
            calculateEndPointsStepTCP(doffsetAngle, dStepAngle, nRoationAxis);   //rotate around TCP
            m_nIteration = 0;
            stepMovementFinishCallback();
        }

    } else {
        LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
    }
}

//rotate around base frame
void PluginAlgorithm::calculateEndPoints(double fOffsetAngle, int nRotationAxis) {
     if (is_robot_connected_) {
         auto robot_pose = getCurrentRobotTransformMatrix(true);
         Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
         Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //

         Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
         if (ROTATION_X == nRotationAxis) {
             offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
             LOG_INFO(fOffsetAngle);
         }
         else if (ROTATION_Y == nRotationAxis) {
             offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
         }
         else {
             LOG_INFO("the rotation is wrong");
         }
         Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
         Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;

         LOG_INFO(initEulerAngle);
         LOG_INFO(finalEulerAngle);

         //back to pose(4*4 matrix)
         Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(initEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(initEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(initEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_initialPose.block<3, 1>(0, 3) = translation/1000.0;

         tempQuaternion = Eigen::AngleAxisd(finalEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(finalEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(finalEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_finalPose.block<3, 1>(0, 3) = translation/1000.0;
     } else {
         LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
     }
}

//calculate the final pose when the robot rotate about TCP
void PluginAlgorithm::calculateEndPointsTCP(double fOffsetAngle, int nRotationAxis) {
     if (is_robot_connected_) {
         auto robot_pose = getCurrentRobotTransformMatrix(true);
         Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
         Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //

         Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
         if (ROTATION_X == nRotationAxis) {
             offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
             LOG_INFO(fOffsetAngle);
         }
         else if (ROTATION_Y == nRotationAxis) {
             offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
         }
         else {
             LOG_INFO("the rotation is wrong");
         }

         //obtain the transformation between the target about TCP frame
         Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(-offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(-offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(-offsetAngleRad[2], Eigen::Vector3d::UnitZ());
         m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_initialPose.block<3, 1>(0, 3) << 0, 0, 0;

         tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
         m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_finalPose.block<3, 1>(0, 3) << 0, 0, 0;

         m_initialPose = robot_pose * m_initialPose;
         m_initialPose.block<3, 1>(0, 3) = translation/1000.0;
         m_finalPose = robot_pose * m_finalPose;
         m_finalPose.block<3, 1>(0, 3) = translation/1000.0;

//         //test give the posiiton and elular angle;
//         translation = m_initialPose.block<3, 1>(0, 3);
//         eulerAngles = (m_initialPose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //
//         std::cout << "translation"<< translation<< std::endl<<"eulerAngles"<<eulerAngles * 180 / M_PI<< std::endl;

//         translation = m_finalPose.block<3, 1>(0, 3);
//         eulerAngles = (m_finalPose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //
//         std::cout << "translation"<< translation<< std::endl<<"eulerAngles"<<eulerAngles * 180 / M_PI<< std::endl;

         //
     } else {
         LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
     }
}

// step by step rotate around base frame
void PluginAlgorithm::calculateEndPointsStep(double fOffsetAngle, double dStep, int nRotationAxis) {
    fOffsetAngle = fabs(fOffsetAngle);
    dStep = fabs(dStep);   //returnn the absolute value
    if(dStep > fOffsetAngle) {
        LOG_INFO("the step value is larger than offseAngle");
        return;
    }
     if (is_robot_connected_) {
         m_vecPose.resize(0);
         //calculte how many step should be included
         int nNumberofStep = int (floor(2 * fOffsetAngle / dStep));   //get round down

         //current pose
         auto robot_pose = getCurrentRobotTransformMatrix(true);
         Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
         Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //

         // initial pose and final pose
         Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
         if (ROTATION_X == nRotationAxis) {
             offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
             LOG_INFO(fOffsetAngle);
             LOG_INFO(dStep);
             LOG_INFO(nNumberofStep);
         }
         else if (ROTATION_Y == nRotationAxis) {
             offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
         }
         else {
             LOG_INFO("the rotation is wrong");
         }

         Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
         Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;
         LOG_INFO(initEulerAngle);
         LOG_INFO(finalEulerAngle);

         //back to pose(4*4 matrix)
         Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(initEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(initEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(initEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_initialPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_initialPose.block<3, 1>(0, 3) = translation/1000.0;

         tempQuaternion = Eigen::AngleAxisd(finalEulerAngle[0], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(finalEulerAngle[1], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(finalEulerAngle[2], Eigen::Vector3d::UnitZ());
         m_finalPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
         m_finalPose.block<3, 1>(0, 3) = translation/1000.0;

         //save the trarget pose in a row into a vector.
            //save EulerAngles in a row
         QVector<Eigen::Vector3d> vecEulerAngs;
         Eigen::Vector3d vecTempEulerAng(0.0, 0.0, 0.0);
              //save path to final position
         for(int i = 0; i<= nNumberofStep; i++)  {
             if (ROTATION_X == nRotationAxis) {
                 vecTempEulerAng = initEulerAngle;
                 vecTempEulerAng[0] += i * dStep / 180.0 * M_PI;
                 vecEulerAngs.append(vecTempEulerAng);
                 std::cout <<i <<"      "<<vecEulerAngs.at(i)<< "   "<<vecEulerAngs.size()<<std::endl;
             }
             else if (ROTATION_Y == nRotationAxis) {
                 LOG_INFO("ROTATION_Y");

                 vecTempEulerAng = initEulerAngle;
                 vecTempEulerAng[1] += i * dStep / 180.0 * M_PI;
                 vecEulerAngs.append(vecTempEulerAng);
                 std::cout <<i <<"      "<<vecEulerAngs.at(i)<< std::endl;
             }
         }
         vecEulerAngs.append(finalEulerAngle);
               //save path to inital pose back
         int nPoseNumber = vecEulerAngs.size();
         for (int i = 0; i< nPoseNumber; i++) {
             vecEulerAngs.append(vecEulerAngs.at(nPoseNumber-1-i));
             std::cout <<i <<"      "<<nPoseNumber<< std::endl;
         }

         //remove super close value
         nPoseNumber = vecEulerAngs.size();
         double dDifferenceX_Y = 0;
         QVector<int> vecIteration;
         for (int i = 1; i < nPoseNumber; i++) {
             dDifferenceX_Y = fabs(vecEulerAngs.at(i)[0] - vecEulerAngs.at(i-1)[0]);
             dDifferenceX_Y += fabs(vecEulerAngs.at(i)[1] - vecEulerAngs.at(i-1)[1]);
             if (dDifferenceX_Y < 0.0003) {
                 vecIteration.append(i);
             }
         }

         int nRemoveNumber = vecIteration.size();
         for (int i = 0; i < nRemoveNumber; i++) {
             vecEulerAngs.remove(vecIteration.at(nRemoveNumber-1-i));
         }

         //From eluar to matrix into vector
         nPoseNumber = vecEulerAngs.size();
         LOG_INFO(nPoseNumber);
         Eigen::Matrix4d matTempPose{Eigen::Matrix4d::Identity()};
         for (int i = 0; i < nPoseNumber; i++) {
             tempQuaternion = Eigen::AngleAxisd(vecEulerAngs.at(i)[0], Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[1], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[2], Eigen::Vector3d::UnitZ());
             matTempPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
             matTempPose.block<3, 1>(0, 3) = translation/1000.0;
             m_vecPose.append(matTempPose);
             std::cout <<i<<":      "<<vecEulerAngs.at(i)[0]<<"     "<<vecEulerAngs.at(i)[1]<<"     "<<vecEulerAngs.at(i)[2] <<std::endl;
         }

     } else {
         LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
     }
}

//setep by step rotate around TCP
void PluginAlgorithm::calculateEndPointsStepTCP(double fOffsetAngle, double dStep, int nRotationAxis) {
    fOffsetAngle = fabs(fOffsetAngle);
    dStep = fabs(dStep);   //returnn the absolute value
    if(dStep > fOffsetAngle) {
        LOG_INFO("the step value is larger than offseAngle");
        return;
    }
     if (is_robot_connected_) {
         m_vecPose.resize(0);
         //calculte how many step should be included
         int nNumberofStep = int (floor(2 * fOffsetAngle / dStep));   //get round down

         //current pose
         auto robot_pose = getCurrentRobotTransformMatrix(true);
         Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);
         Eigen::Vector3d eulerAngles = (robot_pose.block<3, 3>(0, 0)).eulerAngles(0, 1, 2);  //

         // initial pose and final pose
         Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
         if (ROTATION_X == nRotationAxis) {
             offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
             LOG_INFO(fOffsetAngle);
             LOG_INFO(dStep);
         }
         else if (ROTATION_Y == nRotationAxis) {
             offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
         }
         else {
             LOG_INFO("the rotation is wrong");
         }

//         Eigen::Vector3d initEulerAngle = eulerAngles - offsetAngleRad;
//         Eigen::Vector3d finalEulerAngle = eulerAngles + offsetAngleRad;
//         LOG_INFO(initEulerAngle);
//         LOG_INFO(finalEulerAngle);

         //save the trarget pose in a row into a vector.
            //save EulerAngles in a row
         QVector<Eigen::Vector3d> vecEulerAngs;
         Eigen::Vector3d vecTempEulerAng(0.0, 0.0, 0.0);
              //save path to final position
         for(int i = 0; i<= nNumberofStep; i++)  {
             if (ROTATION_X == nRotationAxis) {
                 vecTempEulerAng = -offsetAngleRad;
                 vecTempEulerAng[0] += i * dStep / 180.0 * M_PI;
                 vecEulerAngs.append(vecTempEulerAng);
                 std::cout <<i <<"      "<<vecEulerAngs.at(i)<< "   "<<vecEulerAngs.size()<<std::endl;
             }
             else if (ROTATION_Y == nRotationAxis) {
                 LOG_INFO("ROTATION_Y");

                 vecTempEulerAng = -offsetAngleRad;
                 vecTempEulerAng[1] += i * dStep / 180.0 * M_PI;
                 vecEulerAngs.append(vecTempEulerAng);
                 std::cout <<i <<"      "<<vecEulerAngs.at(i)<< std::endl;
             }
         }
         vecEulerAngs.append(offsetAngleRad);
               //save path to inital pose back
         int nPoseNumber = vecEulerAngs.size();
         for (int i = 0; i< nPoseNumber; i++) {
             vecEulerAngs.append(vecEulerAngs.at(nPoseNumber-1-i));
             std::cout <<i <<"      "<<nPoseNumber<< std::endl;
         }

         //remove super close value
         nPoseNumber = vecEulerAngs.size();
         double dDifferenceX_Y = 0;
         QVector<int> vecIteration;
         for (int i = 1; i < nPoseNumber; i++) {
             dDifferenceX_Y = fabs(vecEulerAngs.at(i)[0] - vecEulerAngs.at(i-1)[0]);
             dDifferenceX_Y += fabs(vecEulerAngs.at(i)[1] - vecEulerAngs.at(i-1)[1]);
             if (dDifferenceX_Y < 0.0003) {
                 vecIteration.append(i);
             }
         }

         int nRemoveNumber = vecIteration.size();
         for (int i = 0; i < nRemoveNumber; i++) {
             vecEulerAngs.remove(vecIteration.at(nRemoveNumber-1-i));
         }

         //From eluar to matrix into vector
         nPoseNumber = vecEulerAngs.size();
         LOG_INFO(nPoseNumber);
         Eigen::Matrix4d matTempPose{Eigen::Matrix4d::Identity()};
         for (int i = 0; i < nPoseNumber; i++) {
             Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(vecEulerAngs.at(i)[0], Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[1], Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(vecEulerAngs.at(i)[2], Eigen::Vector3d::UnitZ());
             matTempPose.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
             matTempPose.block<3, 1>(0, 3) << 0,0,0;
             matTempPose = robot_pose * matTempPose;
             matTempPose.block<3, 1>(0, 3) = translation / 1000.0;
             m_vecPose.append(matTempPose);
         }

     } else {
         LOG_ERROR("The robot has to be connected before sending any command. Call the 'connect()' method.");
     }
}

//calculate the relative angle between initial TCP frame and current frame
//Eigen::Matrix3d& initialFrame: reference frame, recorded when we start the fan motion
//Eigen::Matrix3d& currentFrame: real-time frame, recorded when we carry on the fan motion
Eigen::Vector3d PluginAlgorithm::calculateReleativeRatationAngle(Eigen::Matrix3d& initialFrame, Eigen::Matrix3d& currentFrame) {
    Eigen::Matrix3d matRelatePose = initialFrame.inverse() * currentFrame;
    Eigen::Vector3d relativeEularAngle = matRelatePose.eulerAngles(0, 1, 2);
    return relativeEularAngle;
}

void PluginAlgorithm::RotateAroundTCP(double fOffsetAngle, int nRotationAxis) {
    auto robot_pose = getCurrentRobotTransformMatrix(true);
    Eigen::Vector3d translation = robot_pose.block<3, 1>(0, 3);

    Eigen::Vector3d offsetAngleRad(0.0, 0.0, 0.0);
    if (ROTATION_X == nRotationAxis) {
        offsetAngleRad[0] = fOffsetAngle / 180.0 * M_PI;   //unit rad
        LOG_INFO(fOffsetAngle);
    }
    else if (ROTATION_Y == nRotationAxis) {
        offsetAngleRad[1] = fOffsetAngle / 180.0 * M_PI;   //unit rad
    }
    else {
        LOG_INFO("the rotation is wrong");
    }
    //obtain the transformation between the target about TCP frame
    Eigen::Quaterniond tempQuaternion = Eigen::AngleAxisd(offsetAngleRad[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(offsetAngleRad[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(offsetAngleRad[2], Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d PoseRotateTCP{Eigen::Matrix4d::Identity()};

    PoseRotateTCP.block<3, 3>(0, 0) = tempQuaternion.toRotationMatrix();
    PoseRotateTCP.block<3, 1>(0, 3) << 0, 0, 0;

    PoseRotateTCP = robot_pose * PoseRotateTCP;
    PoseRotateTCP.block<3, 1>(0, 3) = translation/1000.0;

    executeCartesianCommand(PoseRotateTCP, true);
}

void PluginAlgorithm::onGotoPose(const  Eigen::Matrix4d& pose, bool callback) {
    if (callback) {
        executeCartesianCommand(pose, true,
                                std::bind(&PluginAlgorithm::movementFinishCallback, this));
    } else {
        executeCartesianCommand(pose, true);
    }
}

void PluginAlgorithm::onStepGotoPose(const  Eigen::Matrix4d& pose, bool callback) {
    if (callback) {
        executeCartesianCommand(pose, true,
                                std::bind(&PluginAlgorithm::stepMovementFinishCallback, this));
    } else {
        executeCartesianCommand(pose, true);
    }
}

int PluginAlgorithm::onGetStepFanMotionIteration() {
    return m_nIteration;
}




}  // namespace OrienAdj
}  // namespace ImFusion
