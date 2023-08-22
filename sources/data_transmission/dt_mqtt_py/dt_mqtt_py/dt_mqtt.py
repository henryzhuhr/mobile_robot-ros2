import datetime
import json
import os
import time
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from paho.mqtt import client as mqtt_client
import uuid

def get_mac_address():
    mac=uuid.UUID(int = uuid.getnode()).hex[-12:]
    return ":".join([mac[e:e+2] for e in range(0,11,2)])


"""
rtmp 推流 https://zhuanlan.zhihu.com/p/74260950

"""
class DataTransmission(Node):
    def __init__(self, node_name: str="data_transmission") -> None:
        super().__init__(node_name)
        self.declare_parameter("config_file", "configs/data_transmission.json")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value


        # MQTT
        self.mqtt_client:mqtt_client.Client=None
        self.system_state_data_transmision_topic="/data/car/state"
        self.system_state_data_transmision_topic="/data/car/state/<client_id>"
        self.mac_address = get_mac_address()

        self.parse_config(config_file)

        # ======================
        self.timer_100ms=self.create_timer(0.1,self.timer_100ms_callback)

        

    def parse_config(self, config_file: str):
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
            
            if "mqtt" in config:
                self.get_logger().info("mqtt config: {}".format(config["mqtt"]))
                def on_connect(client, userdata, flags, rc):
                    if rc == 0:
                        self.get_logger().info("Connected to MQTT Broker")
                    elif rc == 1:
                        self.get_logger().error("Connection refused - incorrect protocol version")
                    elif rc == 2:
                        self.get_logger().error("Connection refused - invalid client identifier")
                    elif rc == 3:
                        self.get_logger().error("Connection refused - server unavailable")
                    elif rc == 4:
                        self.get_logger().error("Connection refused - bad username or password")
                    elif rc == 5:
                        self.get_logger().error("Connection refused - not authorised")
                    else:
                        self.get_logger().error("Failed to connect, return code %d\n", rc)
                mqtt_config = config["mqtt"]
                protocol = mqtt_config["protocol"]
                client_id=mqtt_config["client_id"]
                host=mqtt_config["host"]
                port=mqtt_config["port"]
                username=mqtt_config["username"]
                password=mqtt_config["password"]
                client = mqtt_client.Client(client_id)
                client.username_pw_set(username, password)
                client.on_connect = on_connect
                client.connect(host, port, keepalive=60)
                time.sleep(1)
                client.subscribe([
                    (f"{self.system_state_data_transmision_topic}/{client_id}", 0), #  
                    
                    ])
                self.mqtt_client=client
                self.mqtt_client.loop_start()
  
        except Exception as e:
            self.get_logger().error("parse config file error: {}".format(e))
            return

    def timer_100ms_callback(self):
        
        if self.mqtt_client is not None and self.mqtt_client.is_connected():
            data={
                "client_id":self.mqtt_client._client_id.decode(),
                "mac":self.mac_address,
                "nid":"car",
                "time":datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                "data":{
                    "location":{"x":12,"y":23,"z":34},
                    "norm_location":{"x":0.1,"y":0.4,"z":0.4},
                }
            }
            data_msg=json.dumps(data)
            # self.get_logger().info(f"timer 100ms callback {data_msg}")
            self.mqtt_client.publish(self.system_state_data_transmision_topic, data_msg)

def main(args=None):
    rclpy.init(args=args)

    node = DataTransmission()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

