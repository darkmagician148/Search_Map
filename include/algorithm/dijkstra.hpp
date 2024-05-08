#ifndef MAP_CELL_HPP
#define MAP_CELL_HPP
#include <iostream>
#include <map>
#include <limits>
#include <memory>
#include "nav_msgs/msg/path.hpp"
#include "algorithm_base.hpp"


namespace search_map{
namespace dijkstra{

class DijkstraCell{
    private:
        double cost_;
        uint32_t col_,row_;
        DijkstraCell* pre_cell_;

    public:
        DijkstraCell(uint32_t col, uint32_t row);
        ~DijkstraCell();
        void setCost(double cost);
        double getCost() const;
        uint64_t getHash();
        void setPreCell(DijkstraCell *pre_cell);
        DijkstraCell* getPreCell() const;
        std::vector<uint64_t> getHashAroud();
        std::pair<uint32_t, uint32_t> getPoint() const;

        using SharedPtr = std::shared_ptr<DijkstraCell>; 
};

class Dijkstra : public AlgorithmBase{
    private:
        rclcpp::Node::SharedPtr node_;
        std::map<uint64_t, DijkstraCell::SharedPtr> map_;
        geometry_msgs::msg::Pose2D goal_, start_;
        nav_msgs::msg::MapMetaData::SharedPtr meta_data_;
    
    public:

    explicit Dijkstra();
    ~Dijkstra();
    void resetMap();
    void updatePointAroud(DijkstraCell::SharedPtr& cell,std::vector<uint64_t>& hash_cell,uint64_t start_key);
    void finishTable(const uint64_t& start_key);
    nav_msgs::msg::Path* fromGoalToPath(const uint64_t& goal_key);


    void initialize(const rclcpp::Node::WeakPtr& node) override final;
    void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid)override final;
    void setStart(geometry_msgs::msg::Pose2D start)override final;
    void setGoal(geometry_msgs::msg::Pose2D goal)override final;
    nav_msgs::msg::Path* getPath() override final;
    
};

}
}
#endif // _MAP_CELL_HPP