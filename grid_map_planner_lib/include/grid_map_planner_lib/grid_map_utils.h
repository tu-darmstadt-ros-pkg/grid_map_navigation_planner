#pragma once

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace grid_map_utils
{

/**
 * @brief Get the yaw angle from a quaternion.
 */
double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quaternion, tf_quat);
  tf2::Matrix3x3 matrix(tf_quat);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  return yaw;
}

/**
 * @brief Compute the orientation in a 90 degree angle to the given vector from start to end.
 * @return The orientation of the perpendicular line.
 */
geometry_msgs::Quaternion computePerpendicularOrientation(const geometry_msgs::Point& start,
                                                          const geometry_msgs::Point& end)
{
  tf2::Vector3 start_v;
  tf2::fromMsg(start, start_v);

  tf2::Vector3 end_v;
  tf2::fromMsg(end, end_v);

  tf2::Vector3 diff = end_v - start_v;
  diff.setZ(0.0);

  tf2::Vector3 normal = diff.cross(tf2::Vector3(0, 0, 1));

  tf2::Quaternion quat;
  quat.setRPY(0, 0, atan2(normal.y(), normal.x()));

  geometry_msgs::Quaternion orientation = tf2::toMsg(quat);

  return orientation;
}

}  // namespace grid_map_utils
