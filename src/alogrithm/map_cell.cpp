#include "map_cell.hpp"
namespace dijkstras_pkg{
namespace data_cell{

    Cell::Cell( uint32_t col,  uint32_t row): 
    prev_node_(nullptr), row_(row), col_(col) {
        cost_ = std::numeric_limits<double>::max();
    }

    Cell::~Cell(){};

    uint64_t Cell::getHash(){
        return (static_cast<uint64_t>(col_)<<32) + (static_cast<uint64_t>(row_));           
    }
    Cell* Cell::getPrev(){
        return prev_node_;
    }

    void Cell::setPrev(Cell *pre_node){
        prev_node_ = pre_node;
    }

    double Cell::getCost(){
        return cost_;
    }

    void Cell::setCost(double cost){
        cost_ = cost;
    }

    std::pair<uint32_t,uint32_t> Cell::getPosition(){
        return std::make_pair(col_,row_);
    }

}

namespace data_cell {
namespace algorithm {
nav_msgs::msg::Path Graph::calculate(){
            if(mp_.find(start_.getHash()) == mp_.end()){
                return nav_msgs::msg::Path();
            }
            std::vector<uint64_t> prev_list = {start_.getHash()};
            while (prev_list.size() != 0)
            {
                for(auto value : prev_list){
                    auto poision = mp_.find(value);
                    auto [col,row] = poision->second->getPosition();
                }
            }
            if (mp_.find(goal_.getHash())->second->getPrev() == nullptr){
                std::cout<< "not found";
                return;
            }
            auto path = mp_.find(goal_.getHash());
            while (path->second->getPrev() != nullptr){
                path = mp_.find(path->second->getPrev()->getHash());
            }
            
}
}
}
}