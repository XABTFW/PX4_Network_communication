#!/usr/bin/env python3
"""
俯冲轰炸演示脚本
使用MAVSDK控制无人机执行俯冲轰炸任务
"""

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
from mavsdk.shell import ShellError

class DiveBombingDemo:
    def __init__(self, connection_url="udp://:14540"):
        self.drone = System()
        self.connection_url = connection_url
        
    async def connect(self):
        """连接到飞机"""
        print(f"正在连接到 {self.connection_url}...")
        await self.drone.connect(system_address=self.connection_url)
        
        print("等待飞机连接...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✓ 飞机已连接")
                break
    
    async def wait_for_position(self):
        """等待GPS定位"""
        print("等待GPS定位...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                print("✓ GPS定位正常")
                break
    
    async def arm_and_takeoff(self, altitude=100.0):
        """解锁并起飞"""
        print("解锁飞机...")
        await self.drone.action.arm()
        print("✓ 飞机已解锁")
        
        print(f"起飞到 {altitude} 米...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        
        # 等待到达目标高度
        async for position in self.drone.telemetry.position():
            altitude_reached = abs(position.relative_altitude_m - altitude) < 1.0
            if altitude_reached:
                print(f"✓ 已到达高度 {position.relative_altitude_m:.1f} 米")
                break
            await asyncio.sleep(1)
    
    async def enable_offboard(self):
        """启用Offboard模式"""
        print("启用Offboard模式...")
        
        # 获取当前位置
        async for position in self.drone.telemetry.position():
            current_altitude = -position.relative_altitude_m
            break
        
        # 设置初始位置（悬停）
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, current_altitude, 0.0)
        )
        
        try:
            await self.drone.offboard.start()
            print("✓ Offboard模式已启用")
        except OffboardError as error:
            print(f"✗ 启动Offboard失败: {error}")
            raise
    
    async def start_dive_bombing_module(self):
        """启动俯冲轰炸模块"""
        print("启动俯冲轰炸模块...")
        try:
            response = await self.drone.shell.send("dive_bombing_control start")
            print(f"模块响应: {response}")
            print("✓ 俯冲轰炸模块已启动")
        except ShellError as error:
            print(f"✗ 启动模块失败: {error}")
            print("提示：确保模块已编译并包含在固件中")
    
    async def execute_dive_bombing(self, target_x, target_y):
        """执行俯冲轰炸任务"""
        print(f"执行俯冲轰炸任务...")
        print(f"目标位置: 北 {target_x}m, 东 {target_y}m")
        
        try:
            command = f"dive_bombing_control execute {target_x} {target_y}"
            response = await self.drone.shell.send(command)
            print(f"任务响应: {response}")
            print("✓ 俯冲轰炸任务已启动")
        except ShellError as error:
            print(f"✗ 执行任务失败: {error}")
            raise
    
    async def monitor_mission(self, duration=30):
        """监控任务执行"""
        print(f"监控任务执行 ({duration}秒)...")
        start_time = asyncio.get_event_loop().time()
        
        while asyncio.get_event_loop().time() - start_time < duration:
            async for position in self.drone.telemetry.position():
                async for velocity in self.drone.telemetry.velocity_ned():
                    altitude = position.relative_altitude_m
                    speed = (velocity.north_m_s**2 + velocity.east_m_s**2 + velocity.down_m_s**2)**0.5
                    
                    print(f"高度: {altitude:6.1f}m | 速度: {speed:5.1f}m/s | "
                          f"位置: N{position.latitude_deg:.6f} E{position.longitude_deg:.6f}")
                    
                    # 检查是否接近地面（任务可能完成）
                    if altitude < 10.0:
                        print("⚠ 接近地面，准备降落")
                        return
                    
                    await asyncio.sleep(1)
                    break
                break
    
    async def land(self):
        """降落"""
        print("停止Offboard模式...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
        
        print("降落...")
        await self.drone.action.land()
        
        # 等待降落完成
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print("✓ 已安全降落")
                break
    
    async def run_mission(self, target_x=50.0, target_y=30.0, altitude=100.0):
        """运行完整任务"""
        try:
            # 1. 连接
            await self.connect()
            await self.wait_for_position()
            
            # 2. 起飞
            await self.arm_and_takeoff(altitude)
            await asyncio.sleep(2)
            
            # 3. 启用Offboard
            await self.enable_offboard()
            await asyncio.sleep(2)
            
            # 4. 启动俯冲轰炸模块
            await self.start_dive_bombing_module()
            await asyncio.sleep(1)
            
            # 5. 执行任务
            await self.execute_dive_bombing(target_x, target_y)
            
            # 6. 监控任务
            await self.monitor_mission(duration=30)
            
            # 7. 降落
            await self.land()
            
            print("\n✓ 任务完成！")
            
        except KeyboardInterrupt:
            print("\n⚠ 用户中断，紧急降落...")
            await self.land()
        except Exception as e:
            print(f"\n✗ 任务失败: {e}")
            print("尝试紧急降落...")
            try:
                await self.land()
            except:
                print("✗ 无法降落，请手动接管！")

async def main():
    """主函数"""
    print("=" * 60)
    print("俯冲轰炸演示程序")
    print("=" * 60)
    
    # 解析命令行参数
    if len(sys.argv) >= 3:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
    else:
        target_x = 50.0
        target_y = 30.0
    
    if len(sys.argv) >= 4:
        altitude = float(sys.argv[3])
    else:
        altitude = 100.0
    
    if len(sys.argv) >= 5:
        connection_url = sys.argv[4]
    else:
        connection_url = "udp://:14540"  # 默认仿真地址
    
    print(f"配置:")
    print(f"  目标位置: 北 {target_x}m, 东 {target_y}m")
    print(f"  起飞高度: {altitude}m")
    print(f"  连接地址: {connection_url}")
    print("=" * 60)
    print()
    
    # 运行任务
    demo = DiveBombingDemo(connection_url)
    await demo.run_mission(target_x, target_y, altitude)

if __name__ == "__main__":
    # 使用说明
    if len(sys.argv) > 1 and sys.argv[1] in ["-h", "--help"]:
        print("使用方法:")
        print("  python3 dive_bombing_demo.py [target_x] [target_y] [altitude] [connection_url]")
        print()
        print("参数:")
        print("  target_x        目标北向位置 (米), 默认: 50.0")
        print("  target_y        目标东向位置 (米), 默认: 30.0")
        print("  altitude        起飞高度 (米), 默认: 100.0")
        print("  connection_url  连接地址, 默认: udp://:14540")
        print()
        print("示例:")
        print("  # 使用默认参数")
        print("  python3 dive_bombing_demo.py")
        print()
        print("  # 指定目标位置")
        print("  python3 dive_bombing_demo.py 80.0 -40.0")
        print()
        print("  # 指定所有参数")
        print("  python3 dive_bombing_demo.py 80.0 -40.0 120.0 udp://:14540")
        print()
        print("  # 连接真实飞机")
        print("  python3 dive_bombing_demo.py 50.0 30.0 100.0 serial:///dev/ttyUSB0:57600")
        sys.exit(0)
    
    # 运行
    asyncio.run(main())
