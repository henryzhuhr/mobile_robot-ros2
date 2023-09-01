#include <rclcpp/rclcpp.hpp>
#include "base_node/base_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseNode>("base_node"); // 设置节点名称

    /**
     * 初始化节点状态，设置分组和ID。
     * 例如当前传感器节点为手柄节点，
     * 那么分组为传感器，ID为手柄
     * 参考 `SystemState::StateGroup` 和 `SystemState::StateID`
     */
    auto init_error = node->InitNode(
        static_cast<uint8_t>(SystemState::StateGroup::SENSOR),  // 分组为传感器
        static_cast<uint8_t>(SystemState::StateID::Sensor::JOY) // ID为手柄
    );

    if (init_error == 0)
    {
        RCLCPP_INFO(node->get_logger(), "%s[INIT NODE] Successfully init:%s %s", //
                    SystemState::Color::LGREEN, SystemState::Color::DEFAULT,       //
                    node->get_name());
        rclcpp::spin(node);
    }
    else
    {
        RCLCPP_FATAL(node->get_logger(), "%s[INIT NODE] Init Node Error (%ld)%s. Please Check Group and id in \"InitNode()\"", //
                     SystemState::Color::LRED,                                                                                 //
                     init_error,
                     SystemState::Color::DEFAULT);
    }
    rclcpp::shutdown();
    return 0;
}
