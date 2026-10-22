# 整车整合版（历史目录名 left_edge_fix）

包含视觉处理、舵机控制与双电机驱动；固件标记 fw103-preview42，编码器速度闭环当前默认关闭。

## 入口

- [Keil C251 工程](mdk/seekfree.uvproj)
- [主程序与接线/配置说明](user/main.c)
- [中断相关源文件](user/isr.c)
- [共享库](../libraries/)
- [全仓库快速开始](../../../docs/getting-started.md)

## 使用方法

1. 保留本目录与同级 `libraries`。
2. 阅读 `user/main.c` 的硬件连接说明和宏定义，按实际主板接线。
3. 用带 MCS-251 / C251 工具链的 Keil 打开工程并重新编译。
4. 确认接线、供电和输出状态后再烧录验证。

Wi-Fi 例程中的 SSID/密码均为占位值，使用前在本地填写；不要提交真实连接信息。运行结果需在自己的硬件上验证。

## 整车版入口

- `user/main.c`：初始化、图像处理与控制调用。
- `user/car_config.h`：引脚和控制参数。
- `user/car_control.c`：视觉结果到转向/速度目标。
- `user/car_motor.c`、`car_servo.c`、`car_drive.c`：电机、舵机和差速输出。
- `code/`：二值化、寻线、十字和圆环等视觉处理。
- `_track_test/`：已有的主机侧测试源码与桩文件；不是板上验收结论。
- `docs/reference/` 与 `code/*.md`：本地已有的历史说明，可能与当前代码存在差异。

当前 `CAR_ENABLE_ENCODER=0`，不要把带有“闭环”字样的旧标题理解为当前已开启速度闭环。

[返回例程索引](../../../docs/examples.md)
