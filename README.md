# 移动机器人

## 项目简介

涉及技术栈: 
- 语言层: Python3, C++
- 框架: ROS2, PyTorch,  TensorRT
- 库: OpenCV, TensorRT, MQTT, RTMP
- 软件包管理: CMake
- 软件开发相关: Git, Github Actions, VSCode, Docker


## 环境配置
```shell
git clone git@github.com:HenryZhuHR/mobile_robot-ros2.git
cd mobile_robot-ros2
git checkout -b dev origin/dev
```
> 使用 SSH 协议 clone，如果没有配置 SSH 密钥，参考 [Github 配置 SSH 密钥](https://henryzhuhr.github.io/program/git/github-ssh.html)

如果有核心代码的开发权限，则获取核心代码
```shell
git submodule init
git submodule update core
```


配置 python 环境，确保 `python3-venv` 已安装

```shell
# sudo apt install -y python3-venv
bash scripts/env/init-py-env.sh
```

## 项目文档

拥有开发权限的开发者可以在本地查看完整文档: 
```shell
yarn
yarn docs:dev
```

对于公开文档，查看 [文档](https://henryzhuhr.github.io/mobile_robot-ros2/)


## 开发贡献
<!-- readme: contributors -start -->
<table>
<tr>
    <td align="center">
        <a href="https://github.com/HenryZhuHR">
            <img src="https://avatars.githubusercontent.com/u/38650110?v=4" width="100;" alt="HenryZhuHR"/>
            <br />
            <sub><b>HenryZhuHR</b></sub>
        </a>
    </td></tr>
</table>
<!-- readme: contributors -end -->
