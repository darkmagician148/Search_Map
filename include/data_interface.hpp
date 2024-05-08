#ifndef search_map_DATA_INTERFACE__HPP__
#define search_map_DATA_INTERFACE__HPP__

#include "algorithm_base.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "pluginlib/class_loader.hpp"

namespace search_map{

/**
 * @brief 
 * 
 */
class DataInterface: public rclcpp::Node
{
protected:
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr re_pub_map_;
AlgorithmBase::SharedPtr algorithm_;

nav_msgs::msg::OccupancyGrid::SharedPtr grid_;

pluginlib::ClassLoader<AlgorithmBase> algorithm_loader_;
public:
DataInterface(/* args */);
~DataInterface();

/**
 * @brief 
 * 
 * @param msg 
 */
void goalSub(geometry_msgs::msg::PoseStamped::SharedPtr msg);

/**
 * @brief 
 * 
 * @param msg 
 */
void startSub(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

/**
 * @brief 
 * 
 * @param msg 
 */
void rePublishMap(std_msgs::msg::Empty::SharedPtr msg);

/**
 * @brief 
 * 
 * @param file_name 
 */
void mapLoader(const std::string file_name);

/**
 * @brief 
 * 
 */
void declareParam();

/**
 * @brief Get the Param object
 * 
 */
void getParam();

/**
 * @brief 
 * 
 */
void start();
};

} // end namespace search_map


#endif