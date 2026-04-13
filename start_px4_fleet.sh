#!/bin/bash

# 脚本用于同时启动10个PX4 SITL实例
# 所有实例共享一个Gazebo窗口

# 检查px4可执行文件是否存在
if [ ! -f "./build/px4_sitl_default/bin/px4" ]; then
    echo "错误: 找不到 ./build/px4_sitl_default/bin/px4"
    echo "请确保在PX4项目根目录下运行此脚本"
    exit 1
fi

# 清理可能残留的端口占用
echo "清理残留进程..."
pkill -9 px4 2>/dev/null
pkill -9 gz 2>/dev/null
sleep 2

# 设置Gazebo只启动一个窗口（通过环境变量）
export PX4_GZ_WORLD=default
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}

# 存储所有后台进程的PID
PIDS=()

# 启动函数
start_instance() {
    local instance_id=$1
    local pose=$2

    echo "启动实例 $instance_id，位置: $pose"

    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE="$pose" \
    PX4_SIM_MODEL=gz_x500 \
    ./build/px4_sitl_default/bin/px4 -i "$instance_id" > /dev/null 2>&1 &

    PIDS+=($!)

    # 等待MAVLink端口就绪
    local mavlink_port=$((18570 + instance_id))
    local timeout=15
    local elapsed=0
    
    echo -n "  等待实例 $instance_id MAVLink端口 $mavlink_port 就绪..."
    while [ $elapsed -lt $timeout ]; do
        if netstat -tuln 2>/dev/null | grep -q ":$mavlink_port " || ss -tuln 2>/dev/null | grep -q ":$mavlink_port "; then
            echo " 完成 (${elapsed}秒)"
            sleep 3  # 额外等待传感器校准
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
        echo -n "."
    done
    
    echo " 超时！"
    return 1
}

# 启动所有10个实例
start_instance 1 "0,2"
start_instance 2 "0,4"
start_instance 3 "0,6"
start_instance 4 "0,8"


echo ""
echo "所有实例已启动！"
echo "实例ID: 1-10"
echo "后台进程PID: ${PIDS[@]}"
echo ""
echo "按 Ctrl+C 将停止所有实例"
echo "或者运行: kill ${PIDS[@]}"
echo ""

# 等待所有后台进程
wait

