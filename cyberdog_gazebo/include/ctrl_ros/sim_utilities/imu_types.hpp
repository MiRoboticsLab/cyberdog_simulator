/*! @file imu_types.hpp
 *  @brief Data from IMUs
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cpp_types.hpp"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VectorNavData() {
    accelerometer = Vec3<float>(0, 0, 9.81);
    gyro.setZero();
    quat = Quat<float>(0, 0, 0, 1);
  }

  Vec3<float> accelerometer;
  Vec3<float> gyro;
  Quat<float> quat;
  // todo is there status for the vectornav?
};

/*!
 * "Cheater" state sent to the robot from simulator
 */
template <typename T>
struct CheaterState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Quat<T> orientation;
  Vec3<T> position;
  Vec3<T> omegaBody;
  Vec3<T> vBody;
  Vec3<T> acceleration;
};

#endif  // PROJECT_IMUTYPES_H
