# 运动控制管理器


## 调试记录

### 2023.08.24 

串口有时候读写不稳定，有可能是串口连接的问题，提供如下调试方案:

1. 使用 python 脚本测试。 在 `sources/controller/motion_manager/scripts` 目录下

- `serial-data.py`: 生成测试串口数据，可以用与串口调试助手
- `serial-read.py`: 读取串口数据测试，可以用于测试串口读取是否正常
- `serial-write.py`: 写入串口数据测试，采用状态机的方式，可以让小车按照状态机的状态运动，已确认串口数据写入是否正常

1. 使用 `MotionManager::serial_send_timer_callback` 函数测试。修改 `speed_list` 的状态机，可以让小车按照状态机的状态运动，已确认串口数据写入是否正常

```cpp
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
```