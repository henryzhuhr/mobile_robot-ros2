


# MQTT 客户端功能

## 技术栈

- [EMQX](https://www.emqx.io): MQTT 服务器
- [paho.mqtt](https://eclipse.dev/paho): MQTT 客户端




## MQTT 主题设计原则

- **确保 MQTT 主题级别仅使用小写字母、数字和破折号。** 由于 MQTT 主题区分大小写，设计 MQTT 主题时，一定要使用一组标准的命名约定。因此，客户在创建每个主题级别时，应仅使用小写字母、数字和破折号。客户应避免使用驼峰式大小写和空格等难以调试的字符。
- **确保 MQTT 主题级别结构遵循从一般到具体的模式。** 主题架构从左到右，主题级别从一般到具体。
- **为 MQTT 主题添加前缀，以区分数据主题与命令主题。** 确保 MQTT 主题在命令和数据消息之间不存在
重叠。通过预留第一个主题级别来表示数据和命令主题，您可以更轻松地使用 IoT 策略创建细粒度权
限，分开监控命令和命令响应的状态与被动遥测命令。


## MQTT 主题

```text
/<message_type>/<device_type>/<application>/<client_id>
```

- `message_type`: 消息类型，包括 `cmd` 和 `data`    
- `device_type`: 设备类型
- `application`: 应用名称
- `client_id`: 客户端 ID

### 状态数据上报

```text
/data/car/state/<client_id>
```

数据格式

```json
{
    "client_id": "mqtt_123456789",
    "mac": "aa:aa:aa:aa:aa:aa",
    "nid": "car",
    "time": "2023-08-22 02:19:14",
    "data": {
        "location": {
            "x": 12,
            "y": 23,
            "z": 34
        },
        "norm_location": {
            "x": 0.1,
            "y": 0.4,
            "z": 0.4
        }
    }
}
```

### 控制指令下发与响应

控制指令下发
```text
/cmd/car/control/<client_id>
```

控制指令下发时携带一个随机生成的命令编号 `id` ，用于标识该条指令，以及一个 `control` 数组，数组中每个元素为一个指令（便于指令的组合），指令包括 `action` 和 `args` 两个字段，`action` 为指令名称，`args` 为指令参数。
```json
{
    "client_id": "mqttjs_123456789",
    "time": "2023-08-22 02:19:14",
    "cmd": {
        "id": "123456789",
        "control": [
            {
                "action": "move",
                "args": {
                    "x": 0.5,
                    "y": 0.5
                }
            }
        ]
    }
}
```


控制指令响应
```text
/cmd/car/response/<client_id>
```

响应指令中，包含 `id` 用于与控制指令对应，以区分多条指令，`response` 数组中每个元素为一个指令的响应的错误码，`0` 表示成功，其他值表示失败。
```json
{
    "client_id": "mqttjs_123456789",
    "time": "2023-08-22 02:19:14",
    "cmd": {
        "id": "123456789",
        "response": [
            0
        ]
    }
}
```
```json
{
    "client_id": "mqttjs_123456789",
    "time": "2023-08-22 02:19:14",
    "cmd": {
        "id": "123456789",
        "response": [
            0
        ]
    }
}
```



## 参考

- [车联网 TSP 平台场景中的 MQTT 主题设计](https://www.emqx.com/zh/blog/mqtt-topic-design-for-internet-of-vehicles)
- [MQTT协议史上最全解析(纯干货分享)](https://blog.csdn.net/weixin_44788542/article/details/129690265)