#include "algorithm/dijkstra.hpp"
#include "utils/search_utils.hpp"
#include <limits>
#include "rclcpp/logger.hpp"

namespace search_map{
namespace dijkstra{

    DijkstraCell::DijkstraCell(uint32_t col,uint32_t row): 
    col_(col),row_(row),pre_cell_(nullptr){
        cost_ = std::numeric_limits<double>::max();
    }
    DijkstraCell::~DijkstraCell(){}

    void DijkstraCell::setCost(double cost){
        cost_ = cost;
    }
    double DijkstraCell::getCost() const{
        return cost_;
    }

    uint64_t DijkstraCell::getHash(){
        return utils::pointIntToKey(col_,row_);
    }
    std::vector<uint64_t> DijkstraCell::getHashAroud(){
        std::vector<uint64_t> HashAroud;
        HashAroud.push_back(utils::pointIntToKey(row_ -1, col_-1));
        HashAroud.push_back(utils::pointIntToKey(row_ -1, col_));
        HashAroud.push_back(utils::pointIntToKey(row_ -1, col_+1));
        HashAroud.push_back(utils::pointIntToKey(row_ , col_-1));
        HashAroud.push_back(utils::pointIntToKey(row_ , col_+1));
        HashAroud.push_back(utils::pointIntToKey(row_ +1, col_-1));
        HashAroud.push_back(utils::pointIntToKey(row_ +1, col_ ));
        HashAroud.push_back(utils::pointIntToKey(row_ +1, col_+1));
        return HashAroud;

    }
    
    DijkstraCell* DijkstraCell::getPreCell() const{
        return pre_cell_;
    }
    std::pair<uint32_t, uint32_t> DijkstraCell::getPoint() const{
            return std::make_pair(col_,row_);
    }

    void DijkstraCell::setPreCell(DijkstraCell *pre_cell){
        pre_cell_ = pre_cell;
    }



    /**
     * @brief  Implement Dijksta about alogrithm start point to goal point
     * 
     */

    Dijkstra::Dijkstra(){}
    Dijkstra::~Dijkstra(){}
    void Dijkstra::initialize(const rclcpp::Node::WeakPtr& node){
        node_ = node.lock();
    }

    void Dijkstra::setStart(geometry_msgs::msg::Pose2D start){
        start_ = start;
    }
    void Dijkstra::setGoal(geometry_msgs::msg::Pose2D goal){
        goal_ = goal;
    }

    void Dijkstra::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid){
        for(size_t i = 0; i < grid->data.size(); i ++){
            if(grid->data[i] == 0){
                auto[col,row]= utils::indexToPointInt(i,grid->info.width);
                uint64_t key = utils::pointIntToKey(row,col);
                DijkstraCell::SharedPtr newCell = std::make_shared<DijkstraCell>(col,row);
                map_.emplace(std::make_pair(key,newCell));
            }
        }
        meta_data_.reset(&grid->info);
    }

    void Dijkstra::resetMap(){
        for(auto& cell:map_){
            cell.second->setCost(std::numeric_limits<double>::max());
            cell.second->setPreCell(nullptr);
        }
    }
    void Dijkstra::updatePointAroud(DijkstraCell::SharedPtr& cell, std::vector<uint64_t>& hash_cell, uint64_t start_key){
        auto aroud_cell = cell->getHashAroud();
        for (const auto& cell_key : aroud_cell){
            auto it = map_.find(cell_key);
            if(it == map_.end()){
                continue;
            }
            if (it->second->getPreCell() == nullptr && it->second->getHash() != start_key){
                RCLCPP_ERROR(node_->get_logger(), "%ld",start_key);
                hash_cell.push_back(it->second->getHash());        
            }
            auto cell_point = cell->getPoint();
            auto check_poit = it->second->getPoint();
            auto delta_col = (double) (cell_point.first - check_poit.first);
            auto delta_row = (double) (cell_point.second - check_poit.second);
            double cost = sqrt((delta_col*delta_col) + (delta_row*delta_row)) * meta_data_->resolution;
            if(it->second->getCost() > cell->getCost()+ cost){
                it->second->setCost(cell->getCost()+ cost);
                it->second->setPreCell(cell.get());
            }
        }
        std::sort(hash_cell.begin(), hash_cell.end());
        auto last = std::unique(hash_cell.begin(), hash_cell.end());
        hash_cell.erase(last, hash_cell.end());
        RCLCPP_ERROR(node_->get_logger(), "%ld", hash_cell.size());
    }
    void Dijkstra::finishTable(const uint64_t& start_key){
        auto start_cell = map_.find(start_key)->second;
        start_cell->setCost(0.0);
        
        std::vector<uint64_t> update_cell;
        updatePointAroud(start_cell,update_cell,start_key);

        std::vector<uint64_t> new_cell;
        while (update_cell.size() != 0)
        {
            new_cell.clear();
            for (const auto& cell_key : update_cell){
                auto it = map_.find(cell_key);
                RCLCPP_ERROR(node_->get_logger(), "%ld",cell_key);
                if(it != map_.end()){
                    updatePointAroud(it->second,new_cell,start_key);
                }
            }
            update_cell = new_cell;
        }
           
    }
    nav_msgs::msg::Path* Dijkstra::fromGoalToPath(const uint64_t& goal_key){
        auto cell = map_.find(goal_key)->second.get();
        cell->getPreCell();
        if (cell->getPreCell() == nullptr){
            return nullptr;
        }
        nav_msgs::msg::Path* path = new nav_msgs::msg::Path();
        auto pre_pose_2d = goal_;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = rclcpp::Clock().now();
        pose_stamped.pose = utils::fromPose2D(goal_);
        path->poses.push_back(pose_stamped);
        while(cell != nullptr){
            auto pose = utils::pointIntToPose(cell->getPoint(),pre_pose_2d,*meta_data_);
            pre_pose_2d = utils::toPose2D(pose);
            pose_stamped.pose = pose;
            path->poses.push_back(pose_stamped);

            cell = cell->getPreCell();
        }
        // goal pose
        pose_stamped.pose = utils::fromPose2D(start_);
        path->poses.push_back(pose_stamped);
        path->header.frame_id = "map";
        path->header.stamp = rclcpp::Clock().now();
        return path;
        
    }

    nav_msgs::msg::Path* Dijkstra::getPath(){
        this->resetMap();
        
        auto [start_col, start_row] = utils::poseToPointInt(start_, *meta_data_);
        auto start_key = utils::pointIntToKey(start_row, start_col);
        auto [goal_col, goal_row] = utils::poseToPointInt(goal_, *meta_data_);
        auto goal_key = utils::pointIntToKey(goal_row, goal_col);

        if(map_.find(start_key) == map_.end() || map_.find(goal_key) == map_.end())
            return nullptr;
        finishTable(start_key);
        // get goal hash value
        return fromGoalToPath(goal_key);
    }


}


}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(search_map::dijkstra::Dijkstra, 
    search_map::AlgorithmBase);