#include "algorithm/a_start.hpp"
#include "utils/search_utils.hpp"

namespace search_map{
namespace a_start{

    AStart::SharedPtr AStart::getInstance(){
        instance = true;
        return std::make_shared<AStart>();
    }
    AStart::~AStart(){
        instance = false;
    }
    void AStart::initialize(const rclcpp::Node::WeakPtr&){
    }

    void AStart::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid){
        table_.resize(grid->data.size());
        for(size_t i = 0; i < grid->data.size(); i++){
            if(grid->data[i] == 0){
                this->table_[i] = std::make_shared<CellCost>(i);
            }
        }
        meta_data_.reset(&grid->info);
    }

    void AStart::setStart(const geometry_msgs::msg::Pose2D start){
        start_ = start;
    }

    void AStart::setGoal(const geometry_msgs::msg::Pose2D goal){
        goal_ = goal;
    }

    void AStart::resetMap(){
        for(auto& cell: table_){
            if(cell.get() != nullptr)
                cell->reset();
        }
    }

    std::vector<size_t> AStart::getPointAround(const size_t& index){
        auto pose_i = utils::indexToPointInt(index, meta_data_->width);
        std::vector<size_t> point_around = {
            utils::pointIntToIndex(pose_i.first-1, pose_i.second-1, meta_data_->width),
            utils::pointIntToIndex(pose_i.first-1, pose_i.second, meta_data_->width),
            utils::pointIntToIndex(pose_i.first-1, pose_i.second+1, meta_data_->width),
            utils::pointIntToIndex(pose_i.first, pose_i.second-1, meta_data_->width),
            utils::pointIntToIndex(pose_i.first, pose_i.second+1, meta_data_->width),
            utils::pointIntToIndex(pose_i.first+1, pose_i.second-1, meta_data_->width),
            utils::pointIntToIndex(pose_i.first+1, pose_i.second, meta_data_->width),
            utils::pointIntToIndex(pose_i.first+1, pose_i.second+1, meta_data_->width)
        };
        std::vector<size_t> available_points;
        for(const auto& index: point_around){
            if(cellAvailable(index)){
                if(table_[index]->state != CellStatus::constant)
                    available_points.push_back(index);                
            }
        }
        return available_points;
    }

    void AStart::updatePointAround(const size_t& index,
        const size_t& goal_index,
        std::vector<size_t>& open_list){
        auto available_points = getPointAround(index);
        auto addToOpenList = [&](const size_t&check_point, const size_t& ref_point,
            const double& successor_current_cost){
            table_[check_point]->state = CellStatus::open;
            open_list.push_back(check_point);
            table_[check_point]->cost = successor_current_cost;
            table_[check_point]->g_cost = gScore(check_point, ref_point);
            table_[check_point]->pre_cell = table_[index].get();
        };
        for(auto point_index: available_points){
            auto node_successor = table_[point_index];
            auto successor_current_cost = fScore(point_index, index, goal_index);
            switch (table_[point_index]->state)
            {
                case CellStatus::open:
                    if(table_[point_index]->cost <= successor_current_cost){
                        continue;
                    }
                    else{
                        table_[point_index]->cost = successor_current_cost;
                        table_[point_index]->g_cost = gScore(point_index, index);
                    }
                    break;
                case CellStatus::close:
                    if(table_[point_index]->cost <= successor_current_cost){
                        continue;
                    }
                    else{
                        addToOpenList(point_index, index, successor_current_cost);
                    }
                    break;
                case CellStatus::unread:
                    addToOpenList(point_index, index, successor_current_cost);
                    break;
                case CellStatus::constant:
                    break;
            }
        }
        table_[index]->state = CellStatus::close;
        sort(open_list.begin(), open_list.end(), [&](const size_t& lhs,const size_t& rhs)
        {
            return table_[lhs]->cost < table_[rhs]->cost;
        });
    }

    void AStart::finishTable(const size_t& start_index, const size_t& goal_index){
        std::vector<size_t> open_list;
        table_[start_index]->cost = 0.0;
        table_[start_index]->g_cost = 0.0;
        table_[start_index]->state = CellStatus::constant;
        open_list.push_back(start_index);
        updatePointAround(start_index, goal_index, open_list);
        while (open_list.size() > 0)
        {
            auto it = *open_list.begin();
            open_list.erase(open_list.begin());
            if(goal_index == it){
                break;
            }
            updatePointAround(it, goal_index, open_list);
        }
        
    }

    double AStart::gScore(const size_t& index, const size_t& pre_cell_index){
        return utils::distanceBetweenCell(index, pre_cell_index, *meta_data_)
            + table_[pre_cell_index]->g_cost;
    }

    double AStart::hScore(const size_t& index, const size_t& goal_index){
        return utils::distanceBetweenCell(index, goal_index, *meta_data_);

    }

    double AStart::fScore(const size_t& index, const size_t& pre_cell_index, const size_t& goal_index){
        return gScore(index, pre_cell_index)
            + hScore(index, goal_index);
    }

    nav_msgs::msg::Path* AStart::fromGoalToPath(const size_t& goal_index){
        auto cell = table_[goal_index].get();
        if(table_[goal_index]->pre_cell == nullptr){
            return nullptr;
        }
        nav_msgs::msg::Path* path = new nav_msgs::msg::Path();
        auto pre_pose_2d = goal_;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = rclcpp::Clock().now();
        // start pose
        pose_stamped.pose = utils::fromPose2D(goal_);
        path->poses.push_back(pose_stamped);
        // link list to path
        while(cell != nullptr){
            auto pointInt = utils::indexToPointInt(cell->index, meta_data_->width);
            auto pose = utils::pointIntToPose(pointInt,
                pre_pose_2d,
                *meta_data_);
            pre_pose_2d = utils::toPose2D(pose);
            pose_stamped.pose = pose;
            path->poses.push_back(pose_stamped);
            cell = cell->pre_cell;
        }
        // goal pose
        pose_stamped.pose = utils::fromPose2D(start_);
        path->poses.push_back(pose_stamped);
        path->header.frame_id = "map";
        path->header.stamp = rclcpp::Clock().now();
        return path;
    }

    bool AStart::cellAvailable(const size_t& index){
        bool available = index < table_.size();
        available = available && (index > 0);
        available = available && (table_[index].get() != nullptr);
        return available;
    }

    nav_msgs::msg::Path* AStart::getPath(){
        resetMap();
        auto start_index = utils::poseToIndex(start_, *meta_data_);
        auto goal_index = utils::poseToIndex(goal_, *meta_data_);
        if (!(cellAvailable(start_index) && cellAvailable(goal_index))){
            return nullptr;
        }
        else if(start_index == goal_index){
            RCLCPP_INFO(node_->get_logger(), "Goal Reach");
            return nullptr;
        }
        finishTable(start_index, goal_index);
        return fromGoalToPath(goal_index);
    }

    } // end namespace algorithm
    } // end namespace search_map

    #include "pluginlib/class_list_macros.hpp"
    PLUGINLIB_EXPORT_CLASS(search_map::a_start::AStart, 
        search_map::AlgorithmBase);
