# 配置与硬件核对

## 先区分配置层次

| 配置位置 | 作用范围 |
|---|---|
| 每个例程的 `user/main.c` | 当前例程的网络信息、外设选择与测试流程 |
| 共享 `libraries/zf_device/*.h` | 所有引用该文件的例程；修改会影响多个工程 |
| 整车版 `user/car_config.h` | 仅整车整合版的驱动、控制、启动与调试参数 |
| 整车版 `code/vision_geometry.h` | 图像处理尺寸与有效区域 |

## Wi-Fi 和上位机

所有 `WIFI_SSID_TEST` / `WIFI_PASSWORD_TEST` 已替换为 `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`，需要在本机填写后重新编译。目标 IP/端口见 [共享 Wi-Fi 驱动头文件](../firmware/Motherboard_Demo/libraries/zf_device/zf_device_wifi_spi.h)，它们是示例配置，不保证适用于你的网络。提交前恢复占位值。

Wi-Fi SPI 图像链路与无线串口是不同的链路，不应将“无线串口能回消息”直接当作“图像 TCP 已连通”的证据。原例程也注明高速 Wi-Fi SPI 与部分屏幕引脚共用，不能默认同时启用。

## 整车默认状态（以当前源码为依据）

来源：[car_config.h](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/user/car_config.h)。这些是配置值，不是实测性能保证。

| 宏 | 当前值 | 含义 |
|---|---|---|
| CAR_FW_TAG | fw103-preview42 | 当前源码版本标记 |
| CAR_ENABLE_SERVO | 1 | 允许舵机输出 |
| CAR_ENABLE_MOTOR | 1 | 允许电机模块工作，仍受启动/硬关等条件限制 |
| CAR_ENABLE_ENCODER | 0 | 默认不启用编码器速度闭环 |
| CAR_BOOT_OUTPUT_OFF | 1 | 保留上电输出硬关设置 |
| CAR_USE_WIFI_LINK | 1 | 启用 Wi-Fi 链路 |
| CAR_WIFI_TCP_CONNECT | 1 | 主动连接上位机 TCP |
| CAR_BRINGUP_TEST | 0 | 默认不执行上电台架自检 |
| CAR_WL_BAUD | 115200 | 无线调试串口配置波特率 |

电机口交换、编码器左右交换、转向正反、舵机中位及打角限幅都有条件配置。请直接阅读 `car_config.h` 的 `#if` 分支，不要只复制某一条宏值。原车标定不能代替对本车的机械核对。

整车图像几何见 [vision_geometry.h](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/code/vision_geometry.h)：当前配置使用二倍下采样，并裁去一行处理图像底部区域。DMA 原始图像和算法处理图像不是同一个尺寸；改尺寸时必须同时核对协议、边线数组和控制使用的坐标。

## 上板前顺序

1. 断开电机动力，检查电源和核心板方向。
2. 确认摄像头和通信能独立工作。
3. 架空车轮，验证左右轮及编码器方向。
4. 解开舵机连杆或限制负载，确认中位与机械极限。
5. 确认启动、停车和失联处理逻辑，再进行低速整车测试。

硬件型号、电池电压、减速比、轮径和赛道光照均未由本次整理确认，需团队补录。

## 目标电脑地址

`libraries/zf_device/zf_device_wifi_spi.h` 的 `WIFI_SPI_TARGET_IP` 已用 `YOUR_PC_IP` 占位。使用图像上传或网络通信前，将其改为你自己电脑在当前网络中的 IPv4 地址；同时填写本地 Wi-Fi 名称和密码。占位字符串可编译，但不对应可连接的网络目标。
