#ifndef SEARCH_MAP_RRT_START__HPP__
#define SEARCH_MAP_RRT_START__HPP__

#include "algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <limits>
#include <map>
#include <random>
#include "visualization_msgs/msg/marker.hpp"

namespace search_map{
namespace rrt{

struct Node
{
    Node() = default;
    Node(const double x, const double y, const int parent_index) :
        x(x), y(y), cost(0.0), parent_index(parent_index)
    {}

    Node(const double x, const double y, double cost, const int parent_index) :
            x(x), y(y), cost(cost), parent_index(parent_index)
    {}

    double x;
    double y;
    double cost;
    int parent_index;
};

class RRTStart: public AlgorithmBase{

protected:

std::mt19937 gen;
std::uniform_real_distribution<> x_dist;
std::uniform_real_distribution<> y_dist;
nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
geometry_msgs::msg::Pose2D start_, goal_;
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

double max_expansion_distance_, goal_tolerance_, search_radius_, delta_x_, delta_y_;
int max_rrt_iters_, collision_checking_points_;

/**
 * @brief 
 * 
 * @param tree 
 */
void publishTree(std::vector<Node> tree) const;

/**
 * @brief 
 * 
 * @param nearest_node 
 * @param new_node 
 * @return true 
 * @return false 
 */
bool isCollided(const Node &nearest_node, const Node &new_node) const;

/**
 * @brief 
 * 
 * @param x_map 
 * @param y_map 
 * @return true 
 * @return false 
 */
bool isCollided(const double x_map, const double y_map) const;

/**
 * @brief 
 * 
 * @param latest_added_node 
 * @return true 
 * @return false 
 */
bool isGoal(const Node &latest_added_node) const;

/**
 * @brief 
 * 
 * @param tree 
 * @param new_node
 * @return double 
 */
double cost(const std::vector<Node> &tree, const Node &new_node) const;

/**
 * @brief 
 * 
 * @param n1 
 * @param n2 
 * @return double 
 */
double lineCost(const Node &n1, const Node &n2) const;

/**
 * @brief 
 * 
 * @param tree 
 * @param node 
 * @return std::vector<int> 
 */
std::vector<int> near(const std::vector<Node> &tree, const Node &node) const;

/**
 * @brief 
 * 
 * @param tree 
 * @param sampled_point 
 * @return int 
 */
int nearest(const std::vector<Node> &tree, const std::array<double, 2> &sampled_point) const;

/**
 * @brief 
 * 
 * @return std::array<double, 2> 
 */
std::array<double, 2> sample();

/**
 * @brief 
 * 
 * @param nearest_node 
 * @param nearest_node_index 
 * @param sampled_point 
 * @return Node 
 */
Node steer(const Node &nearest_node, const int nearest_node_index, const std::array<double, 2> &sampled_point) const;

/**
 * @brief 
 * 
 * @param tree 
 * @param node 
 * @return nav_msgs::msg::Path* 
 */
nav_msgs::msg::Path* fromGoalToPath(const std::vector<Node>& tree, Node node) const;

public:
explicit RRTStart();

~RRTStart();
void initialize(const rclcpp::Node::WeakPtr& node) override final;
void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override final;
void setStart(const geometry_msgs::msg::Pose2D start) override final;
void setGoal(const geometry_msgs::msg::Pose2D goal) override final;
nav_msgs::msg::Path* getPath() override final;

};

} // end namespace algorithm
} // end namespace search_map

#endif