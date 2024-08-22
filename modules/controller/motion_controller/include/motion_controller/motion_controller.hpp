#ifndef MOTION_MANAGER__MOTION_MANAGER_HPP
#define MOTION_MANAGER__MOTION_MANAGER_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "system_state/types.hpp"
#include "system_state/state.hpp"
#include "state_interfaces/msg/speed.hpp"

#include "serial/serial.h"

#include "motion_controller/motion_serial.hpp"

#define DEBUG
#ifdef DEBUG
#define DEBUG_SERIAL_READ
// #define DEBUG_SERIAL_WRITE
#endif

namespace controller {
namespace SM_TEST // 测试状态机 state machine test
{
static uint8_t cnt             = 0;
static uint8_t state           = 0;
static float   speed_list[][3] = {
      {0.3, 0, 0}, {0, 0, 0}, {0, 0.2, 0}, {0, 0, 0}, {0, 0, 1.0}, {0, 0, 0},
};
} // namespace SM_TEST
namespace frame {
namespace v1 {}
inline namespace CAR {
const static uint8_t HEADER_1           = 0xAA;
const static uint8_t HEADER_2           = 0x55;
const static uint8_t SPEED_CONTROL_FLAG = 0x11;
const static uint8_t TAIL               = 0xFF;

/**
 * @brief 串口读取校验状态机
 */
enum class StateMachine : uint8_t { ZERO = 0, HEADER_1, HEADER_2, DATA };

/**
 * @brief 帧数据标志
 */
enum class DataFlag : uint8_t {
    // 控制指令
    SET_SPEED,
    // 数据
    IMU,
    ODOMETER,
    END,
};

// 数据标志查找表 (Data Flag LookUp Table)
const struct _data_flag {
    DataFlag id;
    uint8_t  flag;
    uint8_t  length;

} DF_LUT[] = {
    {DataFlag::SET_SPEED, 0x11, 18},
    {DataFlag::IMU, 0x81, 21},
    {DataFlag::ODOMETER, 0x82, 21},
};
union u_float_byte {
    float   f;
    uint8_t b[sizeof(float)];
};

enum class speed_frame_index : uint8_t {
    FLAG = 2,
    NONE,
    X,
    Y,
    z,
};

} // namespace CAR

} // namespace frame

typedef struct {
    int8_t x = 0;
    int8_t y = 0;
    float  z = 0;
} speed_t;

/**
 * 运动控制管理模块
 */
class MotionController : public MotionSerial {
  private:
    std::thread   read_serial_thread;
    void          LoopReadSerial();
    uint8_t       DATA_FLAG_LENGTH[sizeof(uint8_t) << 8] = {UINT8_MAX};
    const uint8_t SPEED_BUFFER_LEN =
        frame::DF_LUT[static_cast<uint8_t>(frame::DataFlag::SET_SPEED)].length; // 速度控制帧长度
    uint8_t* speed_buffer;

    const TIME_MS                serial_send_timer_interval = TIME_MS(50);
    rclcpp::TimerBase::SharedPtr serial_send_timer_; // 串口发布定时器
    void                         serial_send_timer_callback();

    speed_t speed;

  public:
    explicit MotionController();
    ~MotionController();
    /**
     * @brief 开启循环读取串口数据 的线程
     */
    void StartReadSerial();
    void SetSpeed(float x, float y, float z);
};

} // namespace controller
#endif // MOTION_MANAGER__MOTION_MANAGER_HPP