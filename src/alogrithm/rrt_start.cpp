#include "algorithm/rrt_start.hpp"
#include "utils/map_utils.hpp"

namespace search_map{
namespace rrt{

RRTStart::RRTStart(){

}

RRTStart::~RRTStart(){

}

void RRTStart::initialize(const rclcpp::Node::WeakPtr& node){
    node_ = node.lock();

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_tree", rclcpp::SystemDefaultsQoS());

    node_->declare_parameter<int>("rrt_start.max_rrt_iters", 1000);
    node_->declare_parameter<double>("rrt_start.max_expansion_distance", 10);
    node_->declare_parameter<int>("rrt_start.collision_checking_points", 100);
    node_->declare_parameter<double>("rrt_start.goal_tolerance", 0.1);
    node_->declare_parameter<double>("rrt_start.search_radius", 5.0);

    max_rrt_iters_ = node_->get_parameter("rrt_start.max_rrt_iters").as_int();
    collision_checking_points_ = node_->get_parameter("rrt_start.collision_checking_points").as_int();
    max_expansion_distance_ = node_->get_parameter("rrt_start.max_expansion_distance").as_double();
    goal_tolerance_ = node_->get_parameter("rrt_start.goal_tolerance").as_double();
    search_radius_ = node_->get_parameter("rrt_start.search_radius").as_double();
}

void RRTStart::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid){
    grid_ = grid;
}

void RRTStart::setStart(const geometry_msgs::msg::Pose2D start){
    start_ = start;
}

void RRTStart::setGoal(const geometry_msgs::msg::Pose2D goal){
    goal_ = goal;
    delta_x_ = abs(goal_.x - start_.x);
    delta_y_ = abs(goal_.y - start_.y);
}

nav_msgs::msg::Path* RRTStart::getPath(){
    std::vector<Node> tree;
    tree.emplace_back(Node(start_.x, start_.y, 0.0, -1));
    for(int i = 0; i < max_rrt_iters_; i++){
        const auto sample_node = sample();
        if(isCollided(sample_node[0], sample_node[1]))
        {
            RCLCPP_DEBUG(node_->get_logger(),"Sample Node Colliding");
            continue;
        }
        const int nearest_node_id = nearest(tree, sample_node);

        Node new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);

        const auto current_node_index = tree.size();

        if(isCollided(tree[nearest_node_id], new_node))
        {
            RCLCPP_DEBUG(node_->get_logger(),"Sample Node Edge Colliding");
            continue;
        }

        new_node.cost = cost(tree, new_node);
        const auto near_neighbour_indices = near(tree, new_node);
        std::vector<bool> is_near_neighbor_collided;
        int best_neighbor = new_node.parent_index;

        for(const int near_node_index: near_neighbour_indices)
        {
            if(isCollided(tree[near_node_index], new_node))
            {
                is_near_neighbor_collided.push_back(true);
                continue;
            }
            is_near_neighbor_collided.push_back(false);

            double cost = tree[near_node_index].cost + lineCost(tree[near_node_index], new_node);

            if(cost < new_node.cost)
            {
                new_node.cost = cost;
                new_node.parent_index = near_node_index;
                best_neighbor = near_node_index;
            }
        }

        for(size_t i=0; i<near_neighbour_indices.size(); i++)
        {
            if(is_near_neighbor_collided[i] || i == static_cast<size_t>(best_neighbor))
            {
                continue;
            }
            if(tree[near_neighbour_indices[i]].cost > new_node.cost + lineCost(
                    new_node, tree[near_neighbour_indices[i]]))
            {
                RCLCPP_DEBUG(node_->get_logger(),"Rewiring Parents");
                tree[near_neighbour_indices[i]].parent_index = current_node_index;
            }
        }
        tree.emplace_back(new_node);

        RCLCPP_DEBUG(node_->get_logger(),"Sample Node Edge Found");
        if(isGoal(new_node))
        {
            RCLCPP_INFO(node_->get_logger(),"Goal reached. Backtracking ...");
            RCLCPP_INFO(node_->get_logger(),"Path Found");
            return fromGoalToPath(tree, new_node);
        }
    }
    publishTree(tree);
    return nullptr;
}

std::array<double, 2> RRTStart::sample(){
    std::uniform_real_distribution<>::param_type x_param(-delta_x_, delta_y_);
    std::uniform_real_distribution<>::param_type y_param(-delta_y_, delta_y_);
    x_dist.param(x_param);
    y_dist.param(y_param);
    return { x_dist(gen), y_dist(gen) };
}

bool RRTStart::isCollided(const double x_map, const double y_map) const {
    geometry_msgs::msg::Pose2D pose = geometry_msgs::msg::Pose2D();
    pose.x = x_map;
    pose.y = y_map;
    auto index = utils::poseToIndex(pose, grid_->info);
    return (grid_->data[index] != 0);
}

bool RRTStart::isCollided(const Node &nearest_node, const Node &new_node) const{
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(isCollided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}

Node RRTStart::steer(const Node &nearest_node, const int nearest_node_index,
    const std::array<double, 2> &sampled_point) const{
    const double x = sampled_point[0] - nearest_node.x;
    const double y = sampled_point[1] - nearest_node.y;
    const double distance = pow(x, 2) + pow(y, 2);

    Node new_node{};
    if(distance < max_expansion_distance_)
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        const double theta = atan2(y, x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}

double RRTStart::cost(const std::vector<Node> &tree, const Node &new_node) const{
    return tree[new_node.parent_index].cost + lineCost(tree[new_node.parent_index], new_node);
}

double RRTStart::lineCost(const Node &n1, const Node &n2) const{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

bool RRTStart::isGoal(const Node &latest_added_node) const{
    const double distance = sqrt(pow(latest_added_node.x - goal_.x,2)
        +pow(latest_added_node.y - goal_.y,2));
    return distance < goal_tolerance_;
}

std::vector<int> RRTStart::near(const std::vector<Node> &tree, const Node &node) const{
    std::vector<int> near_neighbor_indices;
    for(size_t i=0; i<tree.size(); i++)
    {
        const double distance = sqrt(pow(node.x - tree[i].x, 2) + pow(node.y - tree[i].y, 2));
        if(distance < search_radius_)
        {
            near_neighbor_indices.push_back(i);
        }
    }
    return near_neighbor_indices;
}

int RRTStart::nearest(const std::vector<Node> &tree, const std::array<double, 2> &sampled_point) const{
    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();
    for(size_t i=0; i< tree.size(); i++)
    {
        const auto distance_sqr = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if(distance_sqr < nearest_node_distance)
        {
            nearest_node = i;
            nearest_node_distance = distance_sqr;
        }
    }
    return nearest_node;
}

nav_msgs::msg::Path* RRTStart::fromGoalToPath(
    const std::vector<Node>& tree, Node node) const{
    nav_msgs::msg::Path* path = new nav_msgs::msg::Path();
    auto pre_pose_2d = goal_;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Clock().now();
    // start pose
    pose_stamped.pose = utils::fromPose2D(goal_);
    path->poses.push_back(pose_stamped);
    // link list to path
    while(node.parent_index != -1){
        geometry_msgs::msg::Pose2D pose_2d;
        auto parent_pose = tree[node.parent_index];
        pose_2d.x = node.x;
        pose_2d.y = node.y;
        pose_2d.theta = atan2(node.y - parent_pose.y,
        node.x - parent_pose.x);
        auto pose = utils::fromPose2D(pose_2d);
        pre_pose_2d = utils::toPose2D(pose);
        pose_stamped.pose = pose;
        path->poses.push_back(pose_stamped);
        node = tree[node.parent_index];
    }
    // goal pose
    pose_stamped.pose = utils::fromPose2D(start_);
    path->poses.push_back(pose_stamped);
    path->header.frame_id = "map";
    path->header.stamp = rclcpp::Clock().now();
    publishTree(tree);
    return path;
}

void RRTStart::publishTree(std::vector<Node> tree) const{
    visualization_msgs::msg::Marker marker;
    marker.color.a = 0.5f;
    marker.color.r = 1.f;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "rrt_start";
    marker.id = 0;
    marker.scale.x = 0.01f;
    marker.pose.position.x = 0.f;
    marker.pose.position.y = 0.f;
    marker.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.f);
    for(const auto& node: tree){
        if(node.parent_index != -1){
            geometry_msgs::msg::Point start_point, end_point;
            start_point.x = node.x;
            start_point.y = node.y;
            end_point.x = tree[node.parent_index].x;
            end_point.y = tree[node.parent_index].y;
            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
        }
    }
    marker_pub_->publish(marker);
}

} // end namespace algorithm
} // end namespace search_map

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(search_map::rrt::RRTStart, 
    search_map::AlgorithmBase);