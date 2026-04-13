#!/bin/bash

# QGC连接诊断脚本

echo "=========================================="
echo "QGroundControl 连接诊断工具"
echo "=========================================="
echo ""

# 1. 检查PX4进程
echo "1. 检查PX4进程状态："
px4_count=$(pgrep -c px4)
if [ $px4_count -gt 0 ]; then
    echo "   ✓ 发现 $px4_count 个PX4进程运行中"
    pgrep -a px4 | head -5
else
    echo "   ✗ 未发现PX4进程！请先启动仿真"
fi
echo ""

# 2. 检查MAVLink端口
echo "2. 检查MAVLink端口状态："
for port in 18570 18571 18572 18573 18574; do
    if netstat -tuln 2>/dev/null | grep -q ":$port " || ss -tuln 2>/dev/null | grep -q ":$port "; then
        echo "   ✓ 端口 $port 已监听"
    else
        echo "   ✗ 端口 $port 未监听"
    fi
done
echo ""

# 3. 检查系统资源
echo "3. 系统资源使用："
echo "   CPU负载: $(uptime | awk -F'load average:' '{print $2}')"
echo "   内存使用: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
echo ""

# 4. 测试MAVLink连接
echo "4. 测试MAVLink心跳包（端口18570）："
timeout 3 nc -u -l 18570 2>/dev/null && echo "   ✓ 端口可访问" || echo "   - 无法测试（需要nc工具）"
echo ""

# 5. 建议
echo "=========================================="
echo "建议操作："
echo "=========================================="
echo "如果进度条卡住："
echo "  1. 关闭QGC，运行: pkill -9 QGroundControl"
echo "  2. 清理PX4进程: pkill -9 px4"
echo "  3. 等待5秒后重新启动仿真"
echo "  4. 确保PX4完全启动后（约15秒）再打开QGC"
echo ""
echo "如果问题持续："
echo "  - 检查防火墙: sudo ufw status"
echo "  - 检查网络管理器: sudo systemctl status NetworkManager"
echo "  - 降低仿真实例数量（从4个减少到2个）"
echo ""
