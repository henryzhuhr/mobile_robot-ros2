#include <fstream>
#include <vector>
#include <string>
#include <json/json.h>

#include "system_manager/system_manager.hpp"

#define set_bit(x, y) (x) |= (1 << (y))     // 将 X 的第 Y 位 置1
#define reset_bit(x, y) (x) &= ~(1 << (y))  // 将 X 的第 Y 位 清0
#define reverse_bit(x, y) (x) ^= (1 << (y)) // 将 X 的第 Y 位 取反
namespace manager
{
    void SystemManager::parse_config()
    {
        std::ifstream ifile;
        ifile.open(this->system_config);
        if (!ifile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "config file\"%s\" open error", this->system_config.c_str());
            return;
        }
        Json::CharReaderBuilder ReaderBuilder;
        ReaderBuilder["emitUTF8"] = true; // enable UTF8 decoding

        Json::Value root; // starts as "null"; will contain the root value after parsing
        std::string strerr;
        bool ok = Json::parseFromStream(ReaderBuilder, ifile, &root, &strerr);
        if (!ok)
        {
            RCLCPP_ERROR(this->get_logger(), "json parse error");
            return;
        }

        if (!root.isMember("system_state"))
        {
            RCLCPP_WARN(this->get_logger(), "config file warning: no key \"state\"");
            return;
        }

        Json::Value jr_system_state = root["system_state"]; // system_state json root

        auto parse_root = [&](const std::string &node_name, SystemState::StateGroup group) -> uint64_t
        {
            for (const auto &item : jr_system_state[node_name])
            {
                std::vector<std::string> key_list = {"id", "enable", "name"};

                for (const auto &key : key_list)
                {
                    if (!(item.isMember(key)))
                    {
                        RCLCPP_ERROR(this->get_logger(), "[ERROR] key system_state.%s.%s not found in config file:%s",
                                     node_name.c_str(),
                                     key.c_str(),
                                     this->system_config.c_str());
                        return static_cast<uint64_t>(SystemState::ErrorCode::CONFIG__KEY_NOT_FOUND);
                    }
                }

                if (item["id"].asInt() > SystemState::ID_NUM_MAX)
                {
                    RCLCPP_WARN(this->get_logger(), "[CONFIG] state [%s] id overflow", node_name.c_str());
                    return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_ID_OVERFLOW);
                }

                bool enable = item["enable"].asBool();
                if (enable)
                {
                    if (this->state_pointer_map.find(static_cast<uint8_t>(group)) == this->state_pointer_map.end())
                    {
                        return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_GROUP_NOT_FOUND);
                    }
                    set_bit(
                        *(this->state_pointer_map.at(static_cast<uint8_t>(group)).first),
                        item["id"].asInt());
                }
                RCLCPP_INFO(this->get_logger(), "%s[INIT]%s %s%7s%s %-6s [%02d] %s%s%s",
                            SystemState::Color::LGREEN,                                   //
                            SystemState::Color::DEFAULT,                                  //
                                                                                          //
                            enable ? SystemState::Color::GREEN : SystemState::Color::RED, //
                            enable ? " enable" : "disable",                               //
                            SystemState::Color::DEFAULT,                                  //
                                                                                          //
                            node_name.c_str(),
                            item["id"].asInt(),
                            SystemState::Color::CYAN,
                            item["name"].asCString(),
                            SystemState::Color::DEFAULT //
                );
            };
            return static_cast<uint64_t>(SystemState::ErrorCode::NO_ERROR);
        };

        if (jr_system_state.isMember("task"))
            parse_root("task", SystemState::StateGroup::TASK);
        if (jr_system_state.isMember("sensor"))
            parse_root("sensor", SystemState::StateGroup::SENSOR);
        if (jr_system_state.isMember("vision"))
            parse_root("vision", SystemState::StateGroup::VISION);

        ifile.close();
    }

} // namespace manager