#include "motion_controller/motion_controller.hpp"

#include <string>
#include <vector>
namespace controller {
/**
 * @brief 循环读取串口数据
 */
void MotionController::LoopReadSerial() {
    uint8_t       state           = 0; // 读取状态机
    uint32_t      cnt             = 0;
    uint32_t      frame_len       = 0;
    const uint8_t MAX_BUFFER_SIZE = 100;
    uint8_t       buffer[MAX_BUFFER_SIZE];
    uint8_t       one_byte = 0;
    // 循环读取串口数据
    while (true) {

        // 判断是否允许读取串口数据
        if (!this->read_serial_enable || !this->serial_.isOpen()) {
            // // 判断串口是否打开
            // while (!this->serial_.isOpen())
            // {
            //     auto open_state = this->OpenSerial();
            //     if (open_state == 0)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "serial open success: %s", this->serial_.getPort().c_str());
            //     }
            // }

            continue;
        }
        try {
            // 读取串口数据
            auto read_bytes = this->serial_.readline();
            this->serial_.flushInput();
            this->serial_.flush();
            if (read_bytes.size() == 0)
                continue;

            for (auto i = 0; i < read_bytes.size(); i++) {
                one_byte = read_bytes[i];

                if (state == static_cast<uint8_t>(frame::StateMachine::ZERO) // 当前空状态
                    && one_byte == frame::HEADER_1                           // 校验 帧头1
                ) {
                    state++;
                    buffer[cnt++] = one_byte;
                } else if (state == static_cast<uint8_t>(frame::StateMachine::HEADER_1) // 帧头1校验通过状态
                           && one_byte == frame::HEADER_2                               // 校验 帧头2
                ) {
                    state++;
                    buffer[cnt++] = one_byte;
                } else if (state == static_cast<uint8_t>(frame::StateMachine::HEADER_2) // 帧头2校验通过状态
                ) {
                    state++;
                    buffer[cnt++] = one_byte;

                    frame_len = DATA_FLAG_LENGTH[one_byte];
                } else if (state == static_cast<uint8_t>(frame::StateMachine::DATA) // 数据校验通过状态，持续读取数据
                ) {
                    buffer[cnt++] = one_byte;

                    if (one_byte == frame::TAIL && cnt == frame_len) // 帧尾校验通过状态
                    {

                        uint8_t sum = 0; // 计算校验和
                        for (uint8_t i = 2; i < cnt - 2; i++) {
                            sum += buffer[i];
                        }
                        if (sum == buffer[cnt - 2]) {
                            {
                                std::string hex_frame_str = "";
                                for (uint32_t i = 0; i < cnt; i++) {
                                    std::ostringstream ss;
                                    ss << std::hex << static_cast<int>(buffer[i]);
                                    hex_frame_str += ss.str();
                                    hex_frame_str += " ";
                                }
#ifdef DEBUG_SERIAL_READ
                                RCLCPP_INFO(this->get_logger(), "[收] [%d/%d]: %s (sum = %d)", frame_len, cnt,
                                            hex_frame_str.c_str(), sum);
#endif
                            }
                        }
                        state = static_cast<uint8_t>(frame::StateMachine::ZERO);
                        cnt   = 0;
                    } else if (cnt >= MAX_BUFFER_SIZE) {
                        state = static_cast<uint8_t>(frame::StateMachine::ZERO);
                        cnt   = 0;
                    }
                } else {
                    state = static_cast<uint8_t>(frame::StateMachine::ZERO);
                    cnt   = 0;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_INFO(this->get_logger(), "serial read error: %s", e.what());
        }
    }
}

void MotionController::SetSpeed(float x, float y, float z) {
    this->speed.x = x * 100;
    this->speed.y = y * 100;
    this->speed.z = z;
    if (this->serial_.isOpen()) {
        this->speed_buffer[static_cast<uint8_t>(frame::speed_frame_index::X)] = this->speed.x;
        this->speed_buffer[static_cast<uint8_t>(frame::speed_frame_index::Y)] = this->speed.y;
        {
            frame::u_float_byte float_byte;
            float_byte.f = this->speed.z;
            for (uint8_t i = 0; i < sizeof(frame::u_float_byte::f); i++) {
                this->speed_buffer[static_cast<uint8_t>(frame::speed_frame_index::z) + i] = float_byte.b[i];
            }
        }
        int sum = 0;
        for (auto i = 2; i < SPEED_BUFFER_LEN - 2; i++) {
            sum += this->speed_buffer[i];
        }
        this->speed_buffer[SPEED_BUFFER_LEN - 2] = sum & 0xFF;
        this->serial_.write(this->speed_buffer, this->SPEED_BUFFER_LEN);

        std::string hex_frame_str = "";
        for (auto i = 0; i < SPEED_BUFFER_LEN; i++) {
            std::ostringstream ss;
            ss << std::hex << static_cast<int>(this->speed_buffer[i]);
            hex_frame_str += ss.str();
            hex_frame_str += " ";
        }

#ifdef DEBUG_SERIAL_WRITE
        RCLCPP_INFO(this->get_logger(), "x=%d, y=%d, z=%.2f (%d)%s", this->speed.x, this->speed.y, this->speed.z,
                    this->SPEED_BUFFER_LEN, hex_frame_str.c_str());
#endif
    }
}

} // namespace controller