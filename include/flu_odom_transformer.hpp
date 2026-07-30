#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_lio
{

struct FluOdomKinematics
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d rotation_world_flu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position_world_flu = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_velocity_world_flu = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_flu = Eigen::Vector3d::Zero();
};

/**
 * Converts FAST-LIO's IMU state to the vehicle FLU frame.
 *
 * Configured transforms use the following coordinate convention:
 *
 *   p_intermediate = R_intermediate_imu * p_imu + t_intermediate_imu
 *   p_flu = R_flu_intermediate * p_intermediate + t_flu_intermediate
 *
 * FAST-LIO estimates T_world_imu. The desired pose is therefore
 *
 *   T_world_flu = T_world_imu * inverse(T_flu_imu).
 */
class FluOdomTransformer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FluOdomTransformer(
      const Eigen::Matrix3d &rotation_intermediate_imu,
      const Eigen::Vector3d &translation_intermediate_imu,
      const Eigen::Matrix3d &rotation_flu_intermediate,
      const Eigen::Vector3d &translation_flu_intermediate)
  {
    const Eigen::Matrix3d rotation_flu_imu =
        rotation_flu_intermediate * rotation_intermediate_imu;
    const Eigen::Vector3d translation_flu_imu =
        rotation_flu_intermediate * translation_intermediate_imu +
        translation_flu_intermediate;

    rotation_imu_flu_ = rotation_flu_imu.transpose();
    translation_imu_flu_ =
        -rotation_imu_flu_ * translation_flu_imu;
    rotation_flu_imu_ = rotation_flu_imu;
  }

  FluOdomKinematics Transform(
      const Eigen::Matrix3d &rotation_world_imu,
      const Eigen::Vector3d &position_world_imu,
      const Eigen::Vector3d &linear_velocity_world_imu,
      const Eigen::Vector3d &angular_velocity_imu) const
  {
    FluOdomKinematics output;
    output.rotation_world_flu =
        rotation_world_imu * rotation_imu_flu_;
    output.position_world_flu =
        position_world_imu +
        rotation_world_imu * translation_imu_flu_;

    // Velocity of the FLU origin, including the IMU-to-FLU lever arm.
    output.linear_velocity_world_flu =
        linear_velocity_world_imu +
        rotation_world_imu *
            angular_velocity_imu.cross(translation_imu_flu_);
    output.angular_velocity_flu =
        rotation_flu_imu_ * angular_velocity_imu;
    return output;
  }

private:
  Eigen::Matrix3d rotation_imu_flu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_imu_flu_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_flu_imu_ = Eigen::Matrix3d::Identity();
};

}  // namespace fast_lio
