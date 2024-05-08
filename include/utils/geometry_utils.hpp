#ifndef search_map_GEOMETRY_UTILS__HPP__
#define search_map_GEOMETRY_UTILS__HPP__

#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/utils.h"

namespace search_map{
namespace utils{

/**
 * @brief 
 * 
 * @param pose 
 * @return geometry_msgs::msg::Pose2D 
 */
inline geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose& pose){
    tf2::Quaternion q(pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.theta = yaw;
    pose_2d.x = pose.position.x;
    pose_2d.y = pose.position.y;
    return pose_2d;
}

/**
 * @brief 
 * 
 * @param pose_2d 
 * @return geometry_msgs::msg::Pose 
 */
inline geometry_msgs::msg::Pose fromPose2D(const geometry_msgs::msg::Pose2D& pose_2d){
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_2d.x;
    pose.position.y = pose_2d.y;
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(pose_2d.theta);
    return pose;
}


} // end namespace utils
} // end namespace search_map


#endif