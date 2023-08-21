# SystemState : 小车的状态信息
# task : 位标识，系统当前执行的任务，至多预留的64个任务
# task_enable : 位标识，系统任务列表可用性，0表示禁用
# sensor : 位标识，系统当前使用的传感器，至多预留的64个传感器
# sensor_enable : 位标识，系统传感器可用性，0表示禁用


std_msgs/Header header


uint64 task_state 0
uint64 task_enable 0

uint64 sensor_state 0
uint64 sensor_enable 0

uint64 vision_state 0
uint64 vision_enable 0
