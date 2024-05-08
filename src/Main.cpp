#include "data_interface.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<search_map::DataInterface> node = 
        std::make_shared<search_map::DataInterface>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
}