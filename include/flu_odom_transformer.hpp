#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_lio
{

struct FluOdomKinematics
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d rotation_camera_init_flu_body_flu =
      Eigen::Matrix3d::Identity();
  Eigen::Vector3d position_camera_init_flu_body_flu =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity_camera_init_flu =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_body_flu =
      Eigen::Vector3d::Zero();
};

/**
 * Converts FAST-LIO's IMU state to the vehicle FLU frame.
 *
 * Configured transforms use the following coordinate convention:
 *
 *   p_camera_init_flu =
 *       R_camera_init_flu_camera_init * p_camera_init
 *   p_intermediate =
 *       R_intermediate_imu * p_imu + t_intermediate_imu
 *   p_body_flu =
 *       R_body_flu_intermediate * p_intermediate
 *       + t_body_flu_intermediate
 *
 * FAST-LIO estimates T_world_imu. The desired pose is therefore
 *
 *   R_camera_init_flu_body_flu =
 *       R_camera_init_flu_camera_init
 *       * R_camera_init_imu
 *       * R_imu_body_flu.
 */
class FluOdomTransformer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FluOdomTransformer(
      const Eigen::Matrix3d &rotation_camera_init_flu_camera_init,
      const Eigen::Matrix3d &rotation_intermediate_imu,
      const Eigen::Vector3d &translation_intermediate_imu,
      const Eigen::Matrix3d &rotation_body_flu_intermediate,
      const Eigen::Vector3d &translation_body_flu_intermediate)
  {
    rotation_camera_init_flu_camera_init_ =
        rotation_camera_init_flu_camera_init;

    const Eigen::Matrix3d rotation_body_flu_imu =
        rotation_body_flu_intermediate * rotation_intermediate_imu;
    const Eigen::Vector3d translation_body_flu_imu =
        rotation_body_flu_intermediate * translation_intermediate_imu +
        translation_body_flu_intermediate;

    rotation_imu_body_flu_ = rotation_body_flu_imu.transpose();
    translation_imu_body_flu_ =
        -rotation_imu_body_flu_ * translation_body_flu_imu;
    rotation_body_flu_imu_ = rotation_body_flu_imu;
  }

  FluOdomKinematics Transform(
      const Eigen::Matrix3d &rotation_camera_init_imu,
      const Eigen::Vector3d &position_camera_init_imu,
      const Eigen::Vector3d &linear_velocity_camera_init_imu,
      const Eigen::Vector3d &angular_velocity_imu) const
  {
    FluOdomKinematics output;
    output.rotation_camera_init_flu_body_flu =
        rotation_camera_init_flu_camera_init_ *
        rotation_camera_init_imu * rotation_imu_body_flu_;
    const Eigen::Vector3d position_camera_init_body_flu =
        position_camera_init_imu +
        rotation_camera_init_imu * translation_imu_body_flu_;
    output.position_camera_init_flu_body_flu =
        rotation_camera_init_flu_camera_init_ *
        position_camera_init_body_flu;

    // Velocity of the FLU origin, including the IMU-to-FLU lever arm.
    const Eigen::Vector3d linear_velocity_camera_init_body_flu =
        linear_velocity_camera_init_imu +
        rotation_camera_init_imu *
            angular_velocity_imu.cross(translation_imu_body_flu_);
    output.linear_velocity_camera_init_flu =
        rotation_camera_init_flu_camera_init_ *
        linear_velocity_camera_init_body_flu;
    output.angular_velocity_body_flu =
        rotation_body_flu_imu_ * angular_velocity_imu;
    return output;
  }

private:
  Eigen::Matrix3d rotation_camera_init_flu_camera_init_ =
      Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rotation_imu_body_flu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_imu_body_flu_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_body_flu_imu_ = Eigen::Matrix3d::Identity();
};

}  // namespace fast_lio
