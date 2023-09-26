/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file ekf_localization.h
 * @brief The class of 
 */

#ifndef MODULES_LOCALIZATION_RTK_EKF_LOCALIZATION_H_
#define MODULES_LOCALIZATION_RTK_EKF_LOCALIZATION_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <math.h>
#include <memory>
#include <tuple>
#include <chrono>
#include <mutex>
#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

class Tracking;
class NavigationState;
class ErrorState;
class ErrorStateCovariance;
/**
 * @class 
 *
 * @brief generate localization info based on RTK
 */
enum Type { ACCELEROMETER, GYRO };

class EKFEstimator {
public:
  EKFEstimator();
  void UpdateWithInertialMeasurement(Eigen::Vector3d data, enum Type type);
  void UpdateWithGPSMeasurements(std::vector<Eigen::Matrix<double, 6, 1>> gps_data);
  Eigen::VectorXd GetErrorState();
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> GetNavigationState();
  Eigen::MatrixXd GetErrorStateCovariance();
  Eigen::Vector3d GetPositionState();
  Eigen::Vector3d GetVelocityState();
  Eigen::Matrix3d GetOrientationState();
  double GetAzimuth();
  void SetQMatrix(Eigen::MatrixXd Q);
  void SetRMatrix(Eigen::MatrixXd R);
  void SetInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Matrix3d T_0);
  void Start();

private:
  Tracking *tracker_;

  Eigen::VectorXd fixed_error_state_, ins_error_state_, current_error_state_;
  Eigen::MatrixXd fixed_error_state_covariance_, ins_error_state_covariance_, current_state_covariance_;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> ins_navigation_state_, fixed_navigation_state_, current_navigation_state_;
  double azimuth_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd H_;

  bool is_running_;
  std::mutex state_mutex_;

  const double velocity_threshold_ = 5.0;
  double z_heading_;
  const bool use_azimuth_alignment_;
};

class Tracking {
  friend class EKFEstimator;
public:
  Tracking(bool use_azimuth_alignment);
  void updateTrackingWithAccelerometer(Eigen::Vector3d f_bi_b);
  void updateTrackingWithGyro(Eigen::Vector3d omega_bi_b);
  void setNavigationInitialState(const Eigen::Vector3d p,
                                 const Eigen::Vector3d v,
                                 const Eigen::Matrix3d T);
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> getNavigationState();
  Eigen::VectorXd getErrorState();
  Eigen::MatrixXd getErrorStateCovariance();

 protected:
  void resetClock();
  void setQMatrix(Eigen::MatrixXd Q);

private:
  void updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void checkAndUpdate();
  void updateDT();

  std::shared_ptr<NavigationState> navigation_state_ptr_;
  std::shared_ptr<ErrorState> error_state_ptr_;
  std::shared_ptr<ErrorStateCovariance> error_state_covariance_ptr_;

  std::vector<Eigen::Vector3d> f_measurments_, g_measurments_;
  std::chrono::duration<double> dt_;
  std::chrono::_V2::system_clock::time_point clock_;

  const bool use_azimuth_alignment_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_RTK_EKF_LOCALIZATION_H_