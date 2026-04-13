#!/bin/sh
#
# DJ_MK ASMG-MD Servo Driver - Example Startup Configuration
#
# 这个文件展示了如何在 PX4 启动脚本中集成 DJ_MK 驱动
# 将相关部分添加到你的板级启动脚本中
#

#=============================================================================
# 第一步: 初始化 CAN 总线
#=============================================================================

echo "Initializing CAN bus for DJ_MK servo..."

# 启动 CAN 驱动，波特率 250kbps
# 注意: 某些板子可能使用不同的命令
if [ -c /dev/can0 ]
then
    echo "CAN0 device found"
else
    echo "Starting CAN driver..."
    # NuttX 系统上启动 CAN
    can start 250000

    # 或者在某些系统上可能需要:
    # ifconfig can0 up
    # ip link set can0 type can bitrate 250000
    # ip link set can0 up
fi

#=============================================================================
# 第二步: 确保 DroneCAN/UAVCAN 已禁用
#=============================================================================

# 检查并禁用 UAVCAN
if param compare UAVCAN_ENABLE 1
then
    echo "WARNING: UAVCAN is enabled, disabling it for DJ_MK..."
    param set UAVCAN_ENABLE 0
fi

# 检查并禁用 Cyphal
if param compare CYPHAL_ENABLE 1
then
    echo "WARNING: Cyphal is enabled, disabling it for DJ_MK..."
    param set CYPHAL_ENABLE 0
fi

#=============================================================================
# 第三步: 设置默认参数（如果未设置）
#=============================================================================

# 检查参数是否已设置，如果没有则设置默认值
if ! param compare DJ_MK_ENABLE -1
then
    echo "Setting default DJ_MK parameters..."
    param set DJ_MK_ENABLE 0
    param set DJ_MK_SERVO_ID 1
    param set DJ_MK_SPEED 256
fi

#=============================================================================
# 第四步: 启动 DJ_MK 驱动
#=============================================================================

# 检查驱动是否启用
if param compare DJ_MK_ENABLE 1
then
    echo "Starting DJ_MK servo driver..."

    # 启动驱动
    if dj_mk start
    then
        echo "✓ DJ_MK driver started successfully"

        # 等待驱动初始化
        usleep 100000

        # 检查驱动状态
        if dj_mk status
        then
            echo "✓ DJ_MK driver is running"
        else
            echo "✗ DJ_MK driver status check failed"
        fi
    else
        echo "✗ Failed to start DJ_MK driver"
        echo "  Check /dev/can0 device and CAN bus configuration"
    fi
else
    echo "DJ_MK driver is disabled (set DJ_MK_ENABLE=1 to enable)"
fi

#=============================================================================
# 第五步: 配置混控器（可选）
#=============================================================================

# 如果需要使用混控器，取消下面的注释
# if param compare DJ_MK_ENABLE 1
# then
#     echo "Loading DJ_MK mixer..."
#     if [ -f /etc/mixers/dj_mk_servo.main.mix ]
#     then
#         mixer load /dev/pwm_output0 /etc/mixers/dj_mk_servo.main.mix
#         echo "✓ Mixer loaded"
#     else
#         echo "✗ Mixer file not found"
#     fi
# fi

#=============================================================================
# 调试信息
#=============================================================================

# 显示当前配置
echo ""
echo "=========================================="
echo "DJ_MK Configuration:"
echo "=========================================="
param show DJ_MK_ENABLE
param show DJ_MK_SERVO_ID
param show DJ_MK_SPEED
echo "=========================================="
echo ""
