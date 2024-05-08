#ifndef SEARCH_MAP_MAP_UTILS__HPP__
#define SEARCH_MAP_MAP_UTILS__HPP__

#include "utils/geometry_utils.hpp"

namespace search_map{
namespace utils{

/**
 * @brief 
 * 
 * @param index 
 * @param width 
 * @return std::pair<uint32_t, uint32_t> 
 */
inline std::pair<uint32_t, uint32_t> indexToPointInt(const size_t& index, const uint32_t& width){
    return std::make_pair(
        static_cast<uint32_t>(index)%width,
        static_cast<uint32_t>(index)/width
    );
}

/**
 * @brief 
 * 
 * @param cell 
 * @param pre_pose 
 * @param meta_data 
 * @return geometry_msgs::msg::Pose 
 */
inline geometry_msgs::msg::Pose pointIntToPose(const std::pair<uint32_t, uint32_t>& cell,
    const geometry_msgs::msg::Pose2D& pre_pose,
    const nav_msgs::msg::MapMetaData& meta_data){
    auto col = cell.first;
    auto row = cell.second;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.x = col*meta_data.resolution + meta_data.origin.position.x + meta_data.resolution/2.0;
    pose_2d.y = row*meta_data.resolution + meta_data.origin.position.y + meta_data.resolution/2.0;
    pose_2d.theta = atan2(pose_2d.y - pre_pose.y,
        pose_2d.x - pre_pose.x);
    return fromPose2D(pose_2d);
}

/**
 * @brief 
 * 
 * @param pose_2d 
 * @param meta_data 
 * @return std::pair<uint32_t, uint32_t> 
 */
inline std::pair<uint32_t, uint32_t> poseToPointInt(const geometry_msgs::msg::Pose2D& pose_2d,
    const nav_msgs::msg::MapMetaData& meta_data){
    return std::make_pair(
        static_cast<uint32_t>((pose_2d.x - meta_data.origin.position.x)/meta_data.resolution),
        static_cast<uint32_t>((pose_2d.y - meta_data.origin.position.y)/meta_data.resolution)
    );
}
inline size_t pointIntToIndex(
        const uint32_t& x_i, const uint32_t& y_i, const uint32_t& width){
    return static_cast<size_t>(x_i)
        + static_cast<size_t>(y_i)*width;
}

inline size_t poseToIndex(const geometry_msgs::msg::Pose2D& pose_2d,
    const nav_msgs::msg::MapMetaData& meta_data){
    auto [col, row] = poseToPointInt(pose_2d, meta_data);
    return pointIntToIndex(col, row, meta_data.width);
}
inline double distanceBetweenCell(const std::pair<uint32_t, uint32_t>& start,
    const std::pair<uint32_t, uint32_t>& goal, const double& resolution){
    double delta_x = static_cast<double>(goal.first) - static_cast<double>(start.first);
    double delta_y = static_cast<double>(goal.second) - static_cast<double>(start.second);
    return sqrt(delta_x*delta_x + delta_y*delta_y)*resolution;
}

inline double distanceBetweenCell(const size_t& start_index,
    const size_t& goal_index, const nav_msgs::msg::MapMetaData& meta_data){
    auto start = indexToPointInt(start_index, meta_data.width);
    auto goal = indexToPointInt(goal_index, meta_data.width);
    return distanceBetweenCell(
        start, goal, meta_data.resolution
    );;
}
} // end namespace utils
} // end namespace search_map

#endif