# 快速开始

## 1. 准备完整工程

从仓库主页选择 Code → Download ZIP，解压后使用；也可以运行 `git clone https://github.com/tvtuh/smart-car.git`。请保留整个目录结构，尤其是 `firmware/Motherboard_Demo/libraries`。

## 2. 准备软件和硬件

- 平台：STC32G144K246 100Pin 主板/核心板，具体外设按例程准备。
- 软件：带 MCS-251 / C251 编译器的 Keil µVision。原上游 README 写作「MDK FOR C251，推荐 V5.60」，这里只保留该来源信息，不将它等同于普通 ARM 工具链。
- 工程当前 Target 为 `stc32g144k246`；工程文件中设备数据库名称为 `AI8052U-32Bit Series`，本次未擅自更改该原配置。若本机提示找不到设备，先核对 C251 安装和对应设备支持。
- 原始 C/H/A51 文件保留已有编码；有的文件使用 GB18030/GBK，有的使用 UTF-8。编辑器乱码时先选择“以编码重新打开”，不要批量改编码。新增中文 Markdown 统一使用 UTF-8。

## 3. 先验证简单外设

仓库根目录提供 [build.ps1](../build.ps1)：`-CheckOnly` 检查所有工程的依赖，`-List` 列出可选例程，`-Example` 指定要编译的目录名称，`-KeilRoot` 指定本机 Keil 安装目录。默认编译整车整合版。命令示例见 [首页](../README.md)。脚本会检查共享库、启动隐藏窗口编译、检查错误/警告和 HEX 是否生成；它不包含编译器，也不自动烧录。

建议顺序：蜂鸣器 → 按键/显示 → 舵机 → 电机 → 编码器 → 摄像头 → 整车。按键例程默认初始化 IPS200，没有该屏幕时不能用“屏幕没显示”单独判断程序未运行。

1. 在 [例程索引](examples.md) 中选一个功能。
2. 打开其 `mdk/seekfree.uvproj`。
3. 先阅读该工程 `user/main.c` 顶部的接线说明及宏定义。
4. 在 µVision 中执行 Rebuild（重新编译）。
5. 确认编译结果，生成文件默认在该工程 `mdk/out_file` 下；HEX 文件不随仓库提交。
6. 用与你的芯片及主板匹配的 STC 下载流程烧录。原上游说明提到按住 P32 上电进入 USB 下载模式，具体接线与下载参数请以实际板卡资料为准。

## 4. 运行团队整车版

打开 [整车整合版工程](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/mdk/seekfree.uvproj)，先阅读 [配置说明](configuration.md)。最少核对：

- `user/main.c` 中的 `WIFI_SSID_TEST` 和 `WIFI_PASSWORD_TEST`（目前为占位值）；
- 共享库 `zf_device_wifi_spi.h` 中的目标 IP、端口与引脚；
- `user/car_config.h` 中电机、舵机、编码器和启动开关；
- `CAR_ENABLE_ENCODER=0` 表示当前没有启用编码器速度闭环；
- `CAR_BOOT_OUTPUT_OFF=1` 表示保留上电输出硬关设置，不能以“电机没动”直接判定烧录失败。

## 5. 常见问题

| 现象 | 先查什么 |
|---|---|
| 找不到 C251/目标设备 | 检查 C251 工具链与设备数据库，而不是直接改成 ARM 器件 |
| 找不到 `zf_*.h` | 是否下载了完整的 `libraries`，是否只复制了一个例程目录 |
| Wi-Fi 一直重试 | 是否替换占位 SSID/密码，热点和模块是否匹配 |
| 图像传不上位机 | 上位机 IP、端口、同一网络、监听状态及防火墙设置 |
| 中文注释乱码 | 以 GB18030/GBK 或 UTF-8 重新打开相应文件 |
| 电机方向/舵机中位不对 | 核对 `car_config.h`、实际电机口与机械标定 |

编译能通过不等于接线正确，也不等于整车能够安全运行；板上验证状态见 [验证记录](verification.md)。
