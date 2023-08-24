#include "motion_manager/motion_manager.hpp"

#include <string>
#include <vector>
namespace controller
{

    MotionManager::MotionManager()
    {
        for (uint8_t i = 0; i < static_cast<uint8_t>(frame::DataFlag::END); i++)
        {
            this->DATA_FLAG_LENGTH[frame::DF_LUT[i].flag] = frame::DF_LUT[i].length;
// #define DEBUG
#ifdef DEBUG
            RCLCPP_INFO(this->get_logger(), "(%d) DATA_FLAG_LENGTH[%d] = %d",
                        i,
                        frame::DATA_FLAG[i].flag,
                        this->DATA_FLAG_LENGTH[frame::DATA_FLAG[i].flag] //
            );
#endif
        }

        RCLCPP_INFO(this->get_logger(), "motion manager init");

        this->serial_send_timer_ = this->create_wall_timer(
            this->serial_send_timer_interval,
            std::bind(&MotionManager::serial_send_timer_callback, this));

        /**
         * 初始化速度控制帧，并且设置速度为0
         */
        {
            this->speed_buffer = new uint8_t[SPEED_BUFFER_LEN];
            for (auto i = 0; i < SPEED_BUFFER_LEN; i++)
            {
                this->speed_buffer[i] = 0;
            }
            this->speed_buffer[0] = frame::HEADER_1;
            this->speed_buffer[1] = frame::HEADER_2;
            this->speed_buffer[2] = frame::SPEED_CONTROL_FLAG;
            this->speed_buffer[SPEED_BUFFER_LEN - 1] = frame::TAIL;
            this->SetSpeed(0, 0, 0);
        }
    }

    MotionManager::~MotionManager()
    {
        if (this->serial_.isOpen())
        {
            uint8_t send_cnt = 20;
            while ((send_cnt--) > 0)
            {
                this->SetSpeed(0, 0, 0);
            }
        }
        this->serial_.close();
        delete[] this->speed_buffer;
    }
    void MotionManager::StartReadSerial()
    {
        // this->read_serial_thread = std::thread(std::bind(&MotionManager::LoopReadSerial, this));
        // this->read_serial_thread.detach();
        // 判断串口是否打开
        while (!this->serial_.isOpen())
        {
            auto open_state = this->OpenSerial();
            if (open_state == 0)
            {
                RCLCPP_INFO(this->get_logger(), "serial open success: %s", this->serial_.getPort().c_str());
            }
        }
    }

    namespace SM_TEST // 测试状态机 state machine test
    {
        static uint8_t cnt = 0;
        static uint8_t state = 0;
        static float speed_list[][3] = {
            {0.3, 0, 0},
            {0, 0, 0},
            {0, 0.2, 0},
            {0, 0, 0},
            {0, 0, 1.0},
            {0, 0, 0},
        };
    }
    void MotionManager::serial_send_timer_callback()
    {
        this->SetSpeed(
            SM_TEST::speed_list[SM_TEST::state][0],
            SM_TEST::speed_list[SM_TEST::state][1],
            SM_TEST::speed_list[SM_TEST::state][2]);

        if (SM_TEST::cnt++ >= 10)
        {
            SM_TEST::cnt = 0;
            SM_TEST::state++;
            SM_TEST::state %= 6;
        }
    }

} // namespace controll