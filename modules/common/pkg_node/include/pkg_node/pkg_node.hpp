#ifndef COMMON__PKG_NODE__PKG_NODE_HPP
#define COMMON__PKG_NODE__PKG_NODE_HPP
#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "rclcpp/rclcpp.hpp"

class PkgNode : public rclcpp::Node
{
private:
    /* data */
public:
    PkgNode(/* args */);
    ~PkgNode();
};




#endif // COMMON__PKG_NODE__PKG_NODE_HPP