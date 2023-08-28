# 移动机器人


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