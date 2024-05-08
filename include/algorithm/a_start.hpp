#ifndef AStart
#define Astart
#include <iostream>
#include <map>
#include <limits>
#include <memory>
#include "nav_msgs/msg/path.hpp"
#include "algorithm_base.hpp"

namespace search_map{
namespace a_start{

    enum class CellStatus {unread,close,open, constant};

    struct CellCost{
        double cost = std::numeric_limits<uint64_t>::max();
        double g_cost = std::numeric_limits<uint64_t>::max();
        CellCost *pre_cell = nullptr;
        CellStatus state = CellStatus::unread;
        const size_t index;

        void reset(){
            cost = std::numeric_limits<uint64_t>::max();
            g_cost = std::numeric_limits<uint64_t>::max();
            pre_cell = nullptr;
            state = CellStatus::unread;
        }

        CellCost(const size_t& index):index(index){}
        using SharedPtr = std::shared_ptr<CellCost>;
    };

    class AStart: public AlgorithmBase{
        private:
            static bool instance;
            explicit AStart();

        public:
            ~AStart();
            using SharedPtr = std::shared_ptr<AStart>;
            SharedPtr getInstance();
            rclcpp::Node::SharedPtr node_;
            geometry_msgs::msg::Pose2D goal_, start_;
            std::vector<CellCost::SharedPtr> table_;
            nav_msgs::msg::MapMetaData::SharedPtr meta_data_;

            void resetMap();
            void updatePointAround(const size_t& index, const size_t& goal_index,std::vector<size_t>& open_list);
            void finishTable(const size_t& start_index, const size_t& goal_index);
            double gScore(const size_t& index, const size_t& pre_cell_index);
            double hScore(const size_t& index, const size_t& goal_index);
            double fScore(const size_t& index, const size_t& pre_cell_index, const size_t& goal_index);
            nav_msgs::msg::Path* fromGoalToPath(const size_t& goal_index);
            bool cellAvailable(const size_t& index);
            std::vector<size_t> getPointAround(const size_t& index);

            void initialize(const rclcpp::Node::WeakPtr& node) override final;
            void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override final;
            void setStart(const geometry_msgs::msg::Pose2D start) override final;
            void setGoal(const geometry_msgs::msg::Pose2D goal) override final;
            nav_msgs::msg::Path* getPath() override final;
    };

}
} // namespace search_map

#endif