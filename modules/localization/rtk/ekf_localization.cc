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

#include "modules/localization/rtk/ekf_localization.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

class NavigationState {
public:
  NavigationState();
  void setState(const Eigen::Vector3d p, const Eigen::Vector3d v, const Eigen::Matrix3d T);
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> getState();
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void integrateState(double dt);

private:
  Eigen::Matrix3d D();
  Eigen::Matrix3d Omega_ne_n();
  Eigen::Matrix3d Omega_ei_n();

  Eigen::Vector3d p_dot_n_, p_n_;
  Eigen::Vector3d v_dot_n_, v_n_;
  Eigen::Matrix3d T_dot_bn_, T_bn_;

  const Eigen::Vector3d g_n_; // TODO: can be more precise

  enum Pose { phi, lambda, h };
  enum Vel { n, e, d };
};

class ErrorState {
public:
  ErrorState(std::shared_ptr<NavigationState> navigation_state_ptr, bool use_azimuth_alignment);
  Eigen::VectorXd getState();
  void updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b);
  void integrateState(double dt);
  void resetErrorState();
  void setOmegaA(double omega_a_max) { omega_a_max_ = omega_a_max; }
  void setOmegaG(double omega_g_max) { omega_g_max_ = omega_g_max; }
  void setOmegaAgm(double omega_a_gm_max) { omega_a_gm_max_ = omega_a_gm_max; }
  void setOmegaGgm(double omega_g_gm_max) { omega_g_gm_max_ = omega_g_gm_max; }
  Eigen::MatrixXd getTransitionMatrix(double dt);

private:
  Eigen::Matrix3d Frr();
  Eigen::Matrix3d Frv();
  Eigen::Matrix3d Fre();
  Eigen::Matrix3d Fvr();
  Eigen::Matrix3d Fvv();
  Eigen::Matrix3d Fve();
  Eigen::Matrix3d Fer();
  Eigen::Matrix3d Fev();
  Eigen::Matrix3d Fee();
  Eigen::Matrix3d Fa();
  Eigen::Matrix3d Fg();
  Eigen::Vector3d omega_a();
  Eigen::Vector3d omega_g();
  Eigen::Vector3d omega_a_gm();
  Eigen::Vector3d omega_g_gm();
  Eigen::MatrixXd F();
  void getNavigationState();

  Eigen::Vector3d delta_p_dot_n_, delta_p_n_;
  Eigen::Vector3d delta_v_dot_n_, delta_v_n_;
  Eigen::Vector3d epsilon_dot_n_, epsilon_n_;
  Eigen::Vector3d b_dot_a_, b_a_;
  Eigen::Vector3d b_dot_g_, b_g_;
  Eigen::VectorXd delta_x_;
  Eigen::MatrixXd F_;

  Eigen::Vector3d p_n_, v_n_;
  Eigen::Matrix3d T_bn_;
  Eigen::Vector3d f_bi_b_, omega_bi_b_;
  std::shared_ptr<NavigationState> navigation_state_ptr_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dis_;
  double omega_a_max_, omega_g_max_, omega_a_gm_max_, omega_g_gm_max_;

  const bool use_azimuth_alignment_;
  const double acceleration_threshold_ = 0.2;
  Eigen::Vector3d current_f_bi_n_, last_f_bi_n_;

  enum Pose { phi, lambda, h };
  enum Vel { n, e, d };
};

class ErrorStateCovariance {
public:
  ErrorStateCovariance(std::shared_ptr<ErrorState> error_state_ptr);
  ErrorStateCovariance(std::shared_ptr<ErrorState> error_state_ptr, Eigen::MatrixXd Q);
  void setQMatrix(Eigen::MatrixXd Q);
  void updateCovarianceMatrix(double dt);
  Eigen::MatrixXd getErrorStateCovariance();
  void resetPMatrix();

 private:
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;

  std::shared_ptr<ErrorState> error_state_ptr_;
};

class Utils {
public:
  static inline double Rm(double phi) {
    return (Re * (1 - e2) /
        std::pow(1 - e2 * std::pow(std::sin(phi), 2), 3 / 2));
  }

  static inline double Rn(double phi) {
    return (Re / std::pow(1 - e2 * std::pow(std::sin(phi), 2), 1 / 2));
  }

  static void toSkewSymmetricMatrix(Eigen::Matrix3d &M, Eigen::Vector3d &v) {
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  }

  static inline double constrainAngleDegree(double x) {
    x = std::fmod(x, 360);
    if (x < 0)
      x += 360;
    return x;
  }

  static inline double degreeToRadian(double x) {
    return constrainAngleDegree(x) * M_PI / 180.0;
  }

  static inline double radianToDegree(double x) {
    return x * 180.0 / M_PI;
  }

  static Eigen::Vector3d toEulerAngles(Eigen::Matrix3d T) {
    Eigen::Vector3d eulerAngles;
    eulerAngles = T.eulerAngles(0, 1, 2);
    return eulerAngles;
  }

  static Eigen::Matrix3d toRotationMatrix(Eigen::Vector3d eulerAngles) {
    Eigen::Matrix3d M;
    M = Eigen::AngleAxisd(eulerAngles(ROLL), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(eulerAngles(PITCH), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(eulerAngles(YAW), Eigen::Vector3d::UnitZ());
    return M;
  }

  static Eigen::Vector3d fromENUtoNED(Eigen::Vector3d enu) {
    Eigen::Matrix3d T;
    T << 1, 0, 0, 0, -1, 0, 0, 0, -1;

    Eigen::Vector3d ned;
    ned = T * enu;
    return ned;
  }

  template<class T>
  static T calcMeanVector(std::vector<T> measurments) {
    T mean;
    mean.setZero();
    for (int n = 0; n < measurments.size(); ++n) {
      for (int i = 0; i < mean.rows(); ++i) {
        mean(i) = mean(i) + 1 / (n + 1) * (measurments.at(n)(i) - mean(i));
      }
    }
    return mean;
  }

  static constexpr double Re = 6378.137E3; // Earth semi-major axis [m]
  static constexpr double e = 0.081819;    // Earth eccentricity
  static constexpr double e2 = std::pow(e, 2);
  static constexpr double omega_ei = 7.292E-05; // Earth turn rate [rad/s]
  static constexpr double g = 9.80665;          // Gravity of Earth [m/s^2]

  enum Euler { ROLL, PITCH, YAW };

  struct FullNavigationState {
    Eigen::Vector3d p_n;
    Eigen::Vector3d v_n;
    Eigen::Matrix3d T_bn;
  };
};

EKFEstimator::EKFEstimator()
  : Q_(15, 15), R_(6, 6), H_(6, 15), is_running_(false), 
    use_azimuth_alignment_(true) {

  tracker_ = new Tracking(use_azimuth_alignment_);

  // Initializing default Q matrix
  Eigen::VectorXd q_diag(15);
  q_diag << 0.01, 0.01, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;
  Q_ = q_diag.asDiagonal();
  tracker_->setQMatrix(Q_);

  // Initializing default R matrix
  Eigen::VectorXd r_diag(6);
  r_diag << 0.000001, 0.000001, 0.5, 0.000001, 0.000001, 0.5;
  R_ = r_diag.asDiagonal();

  H_.setZero();
  H_.topLeftCorner(6, 6).setIdentity();
}

void EKFEstimator::UpdateWithInertialMeasurement(Eigen::Vector3d data, Type type) {
  if (!is_running_) {
    AWARN << "EKFEstimator::updateWithInertialMeasurement - EKF not running yet";
    return;
  }
  switch (type) {
    case ACCELEROMETER:
      tracker_->updateTrackingWithAccelerometer(data);
      break;
    case GYRO:
      tracker_->updateTrackingWithGyro(data);
      break;
    default:
      AERROR << "EKF::updateWithInertialMeasurement - Can't accept this kind of measurement!";
  }
  ins_navigation_state_ = tracker_->getNavigationState();
  ins_error_state_ = tracker_->getErrorState();
  ins_error_state_covariance_ = tracker_->getErrorStateCovariance();
  AINFO << "EKF::updateWithInertialMeasurement - p_n: {} v_n: {} T_n: {}"
        << std::get<0>(ins_navigation_state_).transpose()
        << std::get<1>(ins_navigation_state_).transpose()
        << Utils::toEulerAngles(std::get<2>(ins_navigation_state_)).transpose();
  // Set the most update state to provide
  state_mutex_.lock();
  current_navigation_state_ = ins_navigation_state_;
  current_error_state_ = ins_error_state_;
  current_state_covariance_ = ins_error_state_covariance_;
  state_mutex_.unlock();
}

void EKFEstimator::UpdateWithGPSMeasurements(std::vector<Eigen::Matrix<double, 6, 1>> gps_data) {
  if (!is_running_) {
    AWARN << "EKF::updateWithGPSMeasurements - EKF not running yet";
    return;
  }
  if (gps_data.empty()) {
    AWARN << "EKF::updateWithGPSMeasurements - Got empty GPS data";
    return;
  }

  // Arrange gps data in error state manner
  auto gps_data_mean = Utils::calcMeanVector(gps_data);
  ADEBUG << "EKF::updateWithGPSMeasurements - gps_data_mean: " << gps_data_mean.transpose();
  Eigen::VectorXd pos_vel_state(6);
  pos_vel_state << std::get<0>(ins_navigation_state_), std::get<1>(ins_navigation_state_);
  ADEBUG << "EKF::updateWithGPSMeasurements - pos_vel_state: " << pos_vel_state.transpose();

  double vel = std::sqrt(pos_vel_state(3) * pos_vel_state(3) + pos_vel_state(4) * pos_vel_state(4));
  bool is_vel_higher_than_threshold = vel >= velocity_threshold_;

  // Calculate measurement vector
  Eigen::VectorXd z = pos_vel_state - gps_data_mean;
  if (is_vel_higher_than_threshold && use_azimuth_alignment_) {
    double gps_heading = std::atan2(pos_vel_state(4), pos_vel_state(3));
    double ins_heading = Utils::toEulerAngles(std::get<2>(ins_navigation_state_))(2);
    z_heading_ = ins_heading - gps_heading;
    ADEBUG  << "vel: " << vel << " gps_heading: " << gps_heading
            << " ins_heading: " << ins_heading << " z_heading_: " << z_heading_;
  }
  ADEBUG << "EKF::updateWithGPSMeasurements - z: " << z.transpose();

  // Calculate Kalman gain
  Eigen::MatrixXd P = ins_error_state_covariance_;
  ADEBUG << "EKF::updateWithGPSMeasurements - ins_error_state_covariance:\n"
         << ins_error_state_covariance_;
  Eigen::MatrixXd S = H_ * P * H_.transpose() + R_;
  Eigen::MatrixXd K = P * H_.transpose() * S.inverse();
  ADEBUG << "EKF::updateWithGPSMeasurements - K:\n" << K;

  // Correction step
  fixed_error_state_ = ins_error_state_ + K * (z - H_ * ins_error_state_);
  ADEBUG << "EKF::updateWithGPSMeasurements - fixed_error_state: "
         << fixed_error_state_.transpose();
  fixed_error_state_covariance_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H_) * P;
  // Position correction
  std::get<0>(fixed_navigation_state_) = std::get<0>(ins_navigation_state_) - fixed_error_state_.segment<3>(0);
  // Velocity correction
  std::get<1>(fixed_navigation_state_) = std::get<1>(ins_navigation_state_) - fixed_error_state_.segment<3>(3);
  // Orientation correction
  Eigen::Vector3d epsilon_n = fixed_error_state_.segment<3>(6);
  if (is_vel_higher_than_threshold && use_azimuth_alignment_) {
    epsilon_n(2) = z_heading_;
    Eigen::Vector3d ins_navigation_euler_angles = Utils::toEulerAngles(std::get<2>(ins_navigation_state_));
    std::get<2>(fixed_navigation_state_) = Utils::toRotationMatrix(ins_navigation_euler_angles - epsilon_n);
  }
  else {
    Eigen::Matrix3d E_n;
    Utils::toSkewSymmetricMatrix(E_n, epsilon_n);
    std::get<2>(fixed_navigation_state_) = (Eigen::Matrix3d::Identity() - E_n) * std::get<2>(ins_navigation_state_);
  }

  ADEBUG << "updateWithGPSMeasurements - error_state: \n" << fixed_error_state_.transpose();

  // Reset step
  tracker_->error_state_ptr_->resetErrorState();
  tracker_->navigation_state_ptr_->setState(std::get<0>(fixed_navigation_state_),
                                            std::get<1>(fixed_navigation_state_),
                                            std::get<2>(fixed_navigation_state_));
  tracker_->error_state_covariance_ptr_->resetPMatrix();
  AINFO << "updateWithGPSMeasurements - p_n: " << std::get<0>(fixed_navigation_state_).transpose()
        << " v_n: " << std::get<1>(fixed_navigation_state_).transpose()
        << " T_n: " << Utils::toEulerAngles(std::get<2>(fixed_navigation_state_)).transpose();
  // Set the most update state to provide
  state_mutex_.lock();
  current_navigation_state_ = fixed_navigation_state_;
  current_error_state_ = fixed_error_state_;
  current_state_covariance_ = fixed_error_state_covariance_;
  azimuth_ = Utils::toEulerAngles(std::get<2>(fixed_navigation_state_))(2);
  state_mutex_.unlock();
}

Eigen::VectorXd EKFEstimator::GetErrorState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_error_state_;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> EKFEstimator::GetNavigationState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_navigation_state_;
}

Eigen::MatrixXd EKFEstimator::GetErrorStateCovariance() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_covariance_;
}

Eigen::Vector3d EKFEstimator::GetPositionState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<0>(current_navigation_state_);
}

Eigen::Vector3d EKFEstimator::GetVelocityState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<1>(current_navigation_state_);
}

Eigen::Matrix3d EKFEstimator::GetOrientationState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return std::get<2>(current_navigation_state_);
}

double EKFEstimator::GetAzimuth() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return azimuth_;
}

void EKFEstimator::SetQMatrix(Eigen::MatrixXd Q) {
  Q_ = Q;
  tracker_->setQMatrix(Q_);
}

void EKFEstimator::SetRMatrix(Eigen::MatrixXd R) { R_ = R; }

void EKFEstimator::SetInitialState(Eigen::Vector3d p_0, Eigen::Vector3d v_0, Eigen::Matrix3d T_0) {
  tracker_->setNavigationInitialState(p_0, v_0, T_0);
  state_mutex_.lock();
  current_navigation_state_ = std::make_tuple(p_0, v_0, T_0);
  state_mutex_.unlock();
}

void EKFEstimator::Start() {
  tracker_->resetClock();
  is_running_ = true;
}

Tracking::Tracking(bool use_azimuth_alignment)
  : use_azimuth_alignment_(use_azimuth_alignment) {
  navigation_state_ptr_.reset(new NavigationState());
  error_state_ptr_.reset(new ErrorState(navigation_state_ptr_,use_azimuth_alignment_));
  error_state_covariance_ptr_.reset(new ErrorStateCovariance(error_state_ptr_));

  resetClock();
}

void Tracking::updateTrackingWithAccelerometer(Eigen::Vector3d f_bi_b) {
  f_measurments_.push_back(f_bi_b);
  checkAndUpdate();
}

void Tracking::updateTrackingWithGyro(Eigen::Vector3d omega_bi_b) {
  g_measurments_.push_back(omega_bi_b);
  checkAndUpdate();
}

void Tracking::updateTrackingWithMeasurements(Eigen::Vector3d f_bi_b,
                                              Eigen::Vector3d omega_bi_b) {
  navigation_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  navigation_state_ptr_->integrateState(dt_.count());
  error_state_ptr_->updateStateWithMeasurements(f_bi_b, omega_bi_b);
  error_state_ptr_->integrateState(dt_.count());
  error_state_covariance_ptr_->updateCovarianceMatrix(dt_.count());
}

void Tracking::setNavigationInitialState(const Eigen::Vector3d p,
                                         const Eigen::Vector3d v,
                                         const Eigen::Matrix3d T) {
  navigation_state_ptr_->setState(p, v, T);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d>
 Tracking::getNavigationState() { return navigation_state_ptr_->getState(); }

Eigen::VectorXd Tracking::getErrorState() { return error_state_ptr_->getState(); }

Eigen::MatrixXd Tracking::getErrorStateCovariance() {
  return error_state_covariance_ptr_->getErrorStateCovariance();
}

void Tracking::resetClock() {
  clock_ = std::chrono::high_resolution_clock::now();
}

void Tracking::checkAndUpdate() {
  if (!f_measurments_.empty() && !g_measurments_.empty()) {
    Eigen::Vector3d f_mean = Utils::calcMeanVector(f_measurments_);
    Eigen::Vector3d g_mean = Utils::calcMeanVector(g_measurments_);
    f_measurments_.clear();
    g_measurments_.clear();
    updateDT();
    updateTrackingWithMeasurements(f_mean, g_mean);
    resetClock();
  }
}

void Tracking::updateDT() {
  dt_ = std::chrono::high_resolution_clock::now() - clock_;
}

void Tracking::setQMatrix(Eigen::MatrixXd Q) {
  error_state_covariance_ptr_->setQMatrix(Q);
}


ErrorState::ErrorState(std::shared_ptr<NavigationState> navigation_state_ptr, bool use_azimuth_alignment)
    : navigation_state_ptr_(navigation_state_ptr),
      use_azimuth_alignment_(use_azimuth_alignment),
      gen_(rd_()),
      dis_(-1.0, 1.0),
      delta_x_(15),
      F_(15, 15),
      omega_a_max_(0.0),
      omega_g_max_(0.0),
      omega_a_gm_max_(0.1),
      omega_g_gm_max_(0.001) {
  resetErrorState();
}

Eigen::VectorXd ErrorState::getState() {
  delta_x_ << delta_p_n_, delta_v_n_, epsilon_n_, b_a_, b_g_;
  return delta_x_;
}

void ErrorState::updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b) {
  getNavigationState();
  f_bi_b_ = f_bi_b;
  omega_bi_b_ = omega_bi_b;

  // Azimuth correction
  if (use_azimuth_alignment_) {
    current_f_bi_n_ = T_bn_ * f_bi_b;
  }

  delta_p_dot_n_ = Frr() * delta_p_n_ + Frv() * delta_v_n_ + Fre() * epsilon_n_;
  delta_v_dot_n_ = Fvr() * delta_p_n_ + Fvv() * delta_v_n_ + Fve() * epsilon_n_ + T_bn_ * b_a_ + T_bn_ * omega_a();
  epsilon_dot_n_ = Fer() * delta_p_n_ + Fev() * delta_v_n_ + Fee() * epsilon_n_ + T_bn_ * b_g_ + T_bn_ * omega_g();
  b_dot_a_ = Fa() * b_a_ + omega_a_gm();
  b_dot_g_ = Fg() * b_g_ + omega_g_gm();
}

void ErrorState::integrateState(double dt) {
  delta_p_n_ += dt * delta_p_dot_n_;
  delta_v_n_ += dt * delta_v_dot_n_;
  epsilon_n_ += dt * epsilon_dot_n_;
  b_a_ += dt * b_dot_a_;
  b_g_ += dt * b_dot_g_;

  // Azimuth correction
  if (use_azimuth_alignment_) {
    if ((std::abs(current_f_bi_n_(n) - last_f_bi_n_(n)) >= acceleration_threshold_)
        || (std::abs(current_f_bi_n_(e) - last_f_bi_n_(e)) >= acceleration_threshold_)) {
      double delta_azimuth =
          -((delta_v_dot_n_(e) * current_f_bi_n_(n) - delta_v_dot_n_(n) * current_f_bi_n_(e) + Utils::g * epsilon_n_(n) * current_f_bi_n_(n)
              + Utils::g * epsilon_n_(e) * current_f_bi_n_(e))
              / (current_f_bi_n_(n) * current_f_bi_n_(n) + current_f_bi_n_(e) * current_f_bi_n_(e)));
//      std::cout << "Changing epsilon_n_(d) from : " << epsilon_n_(d) << " to: " << delta_azimuth << std::endl;
      epsilon_n_(d) = delta_azimuth;
    }
    last_f_bi_n_ = current_f_bi_n_;
  }
}

void ErrorState::resetErrorState() {
  delta_p_n_.setZero();
  delta_v_n_.setZero();
  epsilon_n_.setZero();
  b_a_.setZero();
  b_g_.setZero();
}

Eigen::Matrix3d ErrorState::Frr() {
  Eigen::Matrix3d Frr_ = Eigen::Matrix3d::Zero();
  Frr_(0, 2) = -v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2);
  Frr_(1, 0) = v_n_(e) * std::sin(p_n_(phi)) / (std::pow(std::cos(p_n_(phi)), 2) * (Utils::Rn(p_n_(phi) + p_n_(h))));
  Frr_(1, 2) = -v_n_(e) / (std::cos(p_n_(phi)) * std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2));

  return Frr_;
}

Eigen::Matrix3d ErrorState::Frv() {
  Eigen::Matrix3d Frv_ = Eigen::Matrix3d::Zero();
  Frv_(0, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Frv_(1, 1) = 1 / (std::cos(p_n_(phi)) * (Utils::Rn(p_n_(phi)) + p_n_(h)));
  Frv_(2, 2) = -1;

  return Frv_;
}

Eigen::Matrix3d ErrorState::Fre() { return Eigen::Matrix3d::Zero(); }

Eigen::Matrix3d ErrorState::Fvr() {
  Eigen::Matrix3d Fvr_ = Eigen::Matrix3d::Zero();
  Fvr_(0, 0) = -2 * v_n_(e) * Utils::omega_ei * std::cos(p_n_(phi))
      - std::pow(v_n_(e), 2) / ((Utils::Rn(p_n_(phi)) + p_n_(h)) * std::pow(std::cos(p_n_(phi)), 2));
  Fvr_(0, 2) = -v_n_(d) * v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2)
      + std::pow(v_n_(e), 2) * std::tan(p_n_(phi)) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fvr_(1, 0) = 2 * Utils::omega_ei * (-v_n_(d) * std::sin(p_n_(phi)) + v_n_(n) * std::cos(p_n_(phi)))
      + v_n_(n) * v_n_(e) / ((Utils::Rn(p_n_(phi)) + p_n_(h)) * std::pow(std::cos(p_n_(phi)), 2));
  Fvr_(1, 2) = -v_n_(d) * v_n_(e) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2)
      - v_n_(n) * v_n_(e) * std::tan(p_n_(phi)) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fvr_(2, 0) = 2 * v_n_(e) * Utils::omega_ei * std::sin(p_n_(phi));
  Fvr_(2, 2) = std::pow(v_n_(n), 2) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2) + std::pow(v_n_(e), 2) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2)
      - 2 * Utils::g / (Utils::Re + p_n_(h));

  return Fvr_;
}

Eigen::Matrix3d ErrorState::Fvv() {
  Eigen::Matrix3d Fvv_ = Eigen::Matrix3d::Zero();
  Fvv_(0, 0) = v_n_(d) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(0, 1) = -2 * v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h)) - 2 * Utils::omega_ei * std::sin(p_n_(phi));
  Fvv_(0, 2) = v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(1, 0) = 2 * Utils::omega_ei * std::sin(p_n_(phi)) + v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(1, 1) = v_n_(d) / (Utils::Rn(p_n_(phi)) + p_n_(h)) + v_n_(n) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(1, 2) = 2 * Utils::omega_ei * std::cos(p_n_(phi)) + v_n_(e) * std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fvv_(2, 0) = -2 * v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fvv_(2, 1) = -2 * Utils::omega_ei * std::cos(p_n_(phi)) - 2 * v_n_(e) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  return Fvv_;
}

Eigen::Matrix3d ErrorState::Fve() {
  Eigen::Vector3d f_ins_n = T_bn_ * f_bi_b_;
  Eigen::Matrix3d F_ins_n;
  Utils::toSkewSymmetricMatrix(F_ins_n, f_ins_n);

  return -1 * F_ins_n;
}

Eigen::Matrix3d ErrorState::Fer() {
  Eigen::Matrix3d Fer_ = Eigen::Matrix3d::Zero();
  Fer_(0, 0) = Utils::omega_ei * std::sin(p_n_(phi));
  Fer_(0, 2) = v_n_(e) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);
  Fer_(1, 2) = -v_n_(n) / std::pow(Utils::Rm(p_n_(phi)) + p_n_(h), 2);
  Fer_(2, 0) = Utils::omega_ei * std::cos(p_n_(phi)) + v_n_(e) / ((Utils::Rn(p_n_(phi) + p_n_(h))) * std::pow(std::cos(p_n_(phi)), 2));
  Fer_(2, 2) = -v_n_(e) * std::tan(p_n_(phi)) / std::pow(Utils::Rn(p_n_(phi)) + p_n_(h), 2);

  return Fer_;
}

Eigen::Matrix3d ErrorState::Fev() {
  Eigen::Matrix3d Fev_ = Eigen::Matrix3d::Zero();
  Fev_(0, 1) = -1 / (Utils::Rn(p_n_(phi)) + p_n_(h));
  Fev_(1, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  Fev_(2, 1) = std::tan(p_n_(phi)) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  return Fev_;
}

Eigen::Matrix3d ErrorState::Fee() {
  Eigen::Vector3d omega_ins_n = T_bn_ * omega_bi_b_;
  Eigen::Matrix3d Omega_ins_n;
  Utils::toSkewSymmetricMatrix(Omega_ins_n, omega_ins_n);

  return -1 * Omega_ins_n;
}

Eigen::Matrix3d ErrorState::Fa() { return Eigen::Matrix3d::Zero(); }

Eigen::Matrix3d ErrorState::Fg() { return Eigen::Matrix3d::Zero(); }

Eigen::Vector3d ErrorState::omega_a() {
  Eigen::Vector3d omega_a_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_a_max_ * omega_a_;
}

Eigen::Vector3d ErrorState::omega_g() {
  Eigen::Vector3d omega_g_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_g_max_ * omega_g_;
}

Eigen::Vector3d ErrorState::omega_a_gm() {
  Eigen::Vector3d omega_a_gm_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_a_gm_max_ * omega_a_gm_;
}

Eigen::Vector3d ErrorState::omega_g_gm() {
  Eigen::Vector3d omega_g_gm_(dis_(gen_), dis_(gen_), dis_(gen_));

  return omega_g_gm_max_ * omega_g_gm_;
}

void ErrorState::getNavigationState() {
  auto fullState = navigation_state_ptr_->getState();
  p_n_ = std::get<0>(fullState);
  v_n_ = std::get<1>(fullState);
  T_bn_ = std::get<2>(fullState);
}

Eigen::MatrixXd ErrorState::F() {
  F_ << Frr(), Frv(), Fre(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
      Fvr(), Fvv(), Fve(), T_bn_, Eigen::Matrix3d::Zero(),
      Fer(), Fev(), Fee(), Eigen::Matrix3d::Zero(), T_bn_,
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Fa(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Fg();

  return F_;
}

Eigen::MatrixXd ErrorState::getTransitionMatrix(double dt) {
  Eigen::MatrixXd coeff = F() * dt;

  return coeff.exp();
}


ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<ErrorState> error_state_ptr) : P_(15, 15), Q_(15, 15) {
  error_state_ptr_ = error_state_ptr;
  P_.setZero();
  Q_.setZero();
}

ErrorStateCovariance::ErrorStateCovariance(std::shared_ptr<ErrorState> error_state_ptr, Eigen::MatrixXd Q) : ErrorStateCovariance(
    error_state_ptr) {
  setQMatrix(Q);
}

void ErrorStateCovariance::setQMatrix(Eigen::MatrixXd Q) { Q_ = Q; }

void ErrorStateCovariance::updateCovarianceMatrix(double dt) {
  Eigen::MatrixXd phi = error_state_ptr_->getTransitionMatrix(dt);
  P_ = phi * P_ * phi.transpose() + Q_;
}

Eigen::MatrixXd ErrorStateCovariance::getErrorStateCovariance() { return P_; }

void ErrorStateCovariance::resetPMatrix() {
  P_.setZero();
}


NavigationState::NavigationState() : g_n_({0, 0, 1 * Utils::g}) {
  setState(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
}

void NavigationState::setState(const Eigen::Vector3d p, const Eigen::Vector3d v, const Eigen::Matrix3d T) {
  p_n_ = p;
  v_n_ = v;
  T_bn_ = T;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> NavigationState::getState() {
  return std::make_tuple(p_n_, v_n_, T_bn_);
}

void NavigationState::updateStateWithMeasurements(Eigen::Vector3d f_bi_b, Eigen::Vector3d omega_bi_b) {
  p_dot_n_ = D() * v_n_;
  v_dot_n_ = T_bn_ * f_bi_b + g_n_ - (Omega_ne_n() + 2 * Omega_ei_n()) * v_n_;

  Eigen::Matrix3d Omega_bi_b;
  Utils::toSkewSymmetricMatrix(Omega_bi_b, omega_bi_b);
  T_dot_bn_ = T_bn_ * Omega_bi_b - (Omega_ei_n() + Omega_ne_n()) * T_bn_;
}

void NavigationState::integrateState(double dt) {
  p_n_ += dt * p_dot_n_;
  v_n_ += dt * v_dot_n_;
  T_bn_ += dt * T_dot_bn_; // TODO: can be more precise with sin/cos method
}

Eigen::Matrix3d NavigationState::D() {
  Eigen::Matrix3d D_ = Eigen::Matrix3d::Zero();

  D_(0, 0) = 1 / (Utils::Rm(p_n_(phi)) + p_n_(h));
  D_(1, 1) = 1 / (std::cos(p_n_(phi)) * (Utils::Rn(p_n_(phi)) + p_n_(h)));
  D_(2, 2) = -1;

  return D_;
}

Eigen::Matrix3d NavigationState::Omega_ne_n() {
  Eigen::Vector3d omega_ne_n_;
  omega_ne_n_(0) = v_n_(e) / (Utils::Rn(p_n_(phi)) + p_n_(h));
  omega_ne_n_(1) = -v_n_(n) / (Utils::Rm(p_n_(phi)) + p_n_(h));
  omega_ne_n_(2) = -(v_n_(e) * std::tan(p_n_(phi))) / (Utils::Rn(p_n_(phi)) + p_n_(h));

  Eigen::Matrix3d Omega_ne_n_;
  Utils::toSkewSymmetricMatrix(Omega_ne_n_, omega_ne_n_);

  return Omega_ne_n_;
}

Eigen::Matrix3d NavigationState::Omega_ei_n() {
  Eigen::Vector3d omega_ei_n_;
  omega_ei_n_(0) = Utils::omega_ei * std::cos(p_n_(phi));
  omega_ei_n_(1) = 0;
  omega_ei_n_(2) = -Utils::omega_ei * std::sin(p_n_(phi));

  Eigen::Matrix3d Omega_ei_n_;
  Utils::toSkewSymmetricMatrix(Omega_ei_n_, omega_ei_n_);

  return Omega_ei_n_;
}

}  // namespace localization
}  // namespace apollo
