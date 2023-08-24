# 状态信息
# mode : 当前小车的控制状态
# modules : 当前小车的模块(是否启用)状态，最长64个模块，预留

std_msgs/Header header

Speed speed
uint8 mode
uint64 modules
