#include "data_interface.hpp"
#include "glog/logging.h"
#include "nav2_map_server/map_io.hpp"
#include "utils/search_utils.hpp"

using std::placeholders::_1;

namespace search_map{

DataInterface::DataInterface(): rclcpp::Node("search_algorithm_node"),
    algorithm_loader_("search_map", "search_map::AlgorithmBase"){
    grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    declareParam();
    getParam();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", rclcpp::SystemDefaultsQoS());
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS());
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::SystemDefaultsQoS(),
        std::bind(&DataInterface::goalSub, this, _1));
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::SystemDefaultsQoS(),
        std::bind(&DataInterface::startSub, this, _1));
    re_pub_map_ = this->create_subscription<std_msgs::msg::Empty>("re_pub", rclcpp::SystemDefaultsQoS(),
        std::bind(&DataInterface::rePublishMap, this, _1));

    map_pub_->publish(*grid_);
}

DataInterface::~DataInterface(){}

void DataInterface::mapLoader(const std::string file_name){
    auto result = nav2_map_server::loadMapFromYaml(file_name, *grid_);
    CHECK_EQ(result, nav2_map_server::LOAD_MAP_SUCCESS);
}

void DataInterface::goalSub(geometry_msgs::msg::PoseStamped::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Set Goal Pose %f %f",
        msg->pose.position.x, msg->pose.position.y);
    algorithm_->setGoal(utils::toPose2D(msg->pose));
    auto path = algorithm_->getPath();
    if(path != nullptr){
        RCLCPP_INFO(get_logger(), "Create path success");
        path_pub_->publish(*path);
        delete path;
    }
    else{
        RCLCPP_INFO(get_logger(), "Failed when search path");
    }
}

void DataInterface::startSub(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Set Start Pose %f %f",
        msg->pose.pose.position.x, msg->pose.pose.position.y);
    algorithm_->setStart(utils::toPose2D(msg->pose.pose));
}

void DataInterface::rePublishMap(std_msgs::msg::Empty::SharedPtr){
    map_pub_->publish(*grid_);
}

void DataInterface::declareParam(){
    this->declare_parameter<std::string>("map_file", "");
    this->declare_parameter<std::string>("algorithm", "");
}

void DataInterface::getParam(){
    auto path_file = this->get_parameter("map_file").value_to_string();
    mapLoader(path_file);
    auto algorithm_name = this->get_parameter("algorithm").value_to_string();
    algorithm_ = algorithm_loader_.createSharedInstance(algorithm_name);
    algorithm_->setMap(grid_);
}

void DataInterface::start(){
    algorithm_->initialize(this->shared_from_this());
}

} // end namespace search_map