#!/bin/bash

killProcess(){
    echo "收到 Ctrl+C 信号，正在结束进程..."

    kill -9 $pid_gps_driver_node
    exit 0
}

source ./setenv.bash

echo $GPS_DRIVER_PATH

# every process wait time 0.2s
wait_time=0.2s

# if [ -f "$GPS_DRIVER_PATH/../config/robo_slam_3d.json" ]; then
#     echo "config/robo_slam_3d.json 文件已存在."
# else
#     echo "config/robo_slam_3d.json 文件不存在."
#     # 复制当前目录下的backup_config/config目录到上一层目录并重命名为config
#     cp -r $GPS_DRIVER_PATH/backup_config/config $GPS_DRIVER_PATH/..
#     echo "已将 backup_config/config 目录复制到 $GPS_DRIVER_PATH/../config"
# fi

gps_driver_node=$GPS_DRIVER_PATH/bin/gps_driver_node
$gps_driver_node &
pid_gps_driver_node=$!
echo "gps_driver_node PID:$pid_gps_driver_node"
sleep $wait_time

trap killProcess SIGINT SIGTERM

while true; do
    sleep 1
done
