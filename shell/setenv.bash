#!/bin/bash

# 设置GPS_DRIVER_PATH环境变量为当前脚本路径
# 如果在交互模式下运行，使用$0获取脚本路径
if [ -z "$PS1" ]; then
  echo "交互模式"
  SCRIPT_DIR="$( cd "$( dirname "$0" )" && pwd )"
else
  # 在非交互模式下使用$BASH_SOURCE获取脚本路径
  echo "非交互模式"
  SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
fi

export GPS_DRIVER_PATH=$SCRIPT_DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GPS_DRIVER_PATH/lib:$GPS_DRIVER_PATH/lib/robo_slam_3d

# 打印当前脚本路径
echo "当前gps_driver安装路径为: $GPS_DRIVER_PATH"