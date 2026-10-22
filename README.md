# SZU team · Smart Car

STC32G144K246（100 引脚）智能车团队工程与主板例程。这里包含逐飞基础例程、共享驱动库，以及本地已有的视觉/整车开发版本。

> 当前为工程整理版，不代表已经完成整车验收。整车整合版的固件标记是 `fw103-preview42`；编码器速度闭环当前默认关闭。所有 Wi-Fi 名称与密码已替换为占位值，使用前按说明配置。

## 从这里开始

| 你的目标 | 入口 |
|---|---|
| 第一次接手工程，准备编译/烧录 | [快速开始](docs/getting-started.md) |
| 根据外设查找例程 | [全部例程索引](docs/examples.md) |
| 阅读团队整车代码 | [整车整合版](./firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/README.md) |
| 对比另一份视觉/控制开发版本 | [fw62 开发版](./firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo/README.md) |
| 修改 Wi-Fi、引脚和控制参数 | [配置与硬件核对](docs/configuration.md) |
| 了解模块分工和数据流 | [整车代码导读](docs/architecture.md) |
| 查看本次实际检查结果 | [验证记录](docs/verification.md) |

## 目录结构

```text
smart-car/
├── README.md
├── docs/                         # 中文入门、例程索引与配置说明
├── THIRD_PARTY_NOTICES.md          # 来源和许可证说明
└── firmware/Motherboard_Demo/
    ├── E01_... ～ E09_...          # 40 个主例程/开发版本，各有独立 Keil 工程
    ├── E07_09_..._left_edge_fix/   # 整车整合版，已展开重复目录
    ├── archive/                   # 1 份保留用于比对的摄像头历史副本
    ├── libraries/                 # 全部工程共用的逐飞驱动与库
    └── LICENSE                    # 随原始资料保留的 GPL v3 许可证
```

## 开发环境与使用原则

本目录有三份独立的视觉/控制版本：整车整合版为 `fw103-preview42`，原 `E07_09_wifi_spi_mt9v03x_demo` 为 `fw62`，归档副本为 `fw61`。三份均包含电机/舵机控制调用，不要当成仅显示图像的程序。构建脚本默认选择 `fw103-preview42`，这只是明确入口，不代表已经通过赛道验收。

安装好 Keil C251 后，可以在仓库根目录用 [build.ps1](build.ps1) 检查依赖并编译：

```powershell
# 检查完整下载后的全部工程依赖，不需要编译器
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 -CheckOnly

# 编译整车整合版；Keil 路径按本机安装位置填写
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 -KeilRoot "D:\Keil_v5"

# 列出例程，或先编译一个简单蜂鸣器例程
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 -List
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1 -Example E01_02_buzzer_demo -KeilRoot "D:\Keil_v5"
```

脚本会显示实际生成的 HEX 路径和编译日志位置，不会自动烧录硬件。警告数量会如实显示。

- 使用包含 **MCS-251 / C251** 工具链的 Keil µVision；不能仅凭安装了 MDK-ARM 就认为环境完整。
- 每个例程独立编译：打开对应 `mdk/seekfree.uvproj`，不要把多个例程的 `main.c` 同时加入一个目标。
- 共享 `libraries` 必须随工程一起克隆或下载，不能只取某个例程文件夹。
- 外设、电机方向、编码器接法、舵机中位都要按实物核对；原车参数不能直接当成其他车的标定值。
- 首次测试电机或舵机时先解除机械负载、架空车轮，并确认有可用的断电方式。

## 资料来源

基础代码来源于本地的「逐飞科技 STC32G144K 100Pin 开源库」资料；共享库内的版本记录为 **V3.2.0**。原始作者声明保留，具体见 [来源说明](THIRD_PARTY_NOTICES.md) 和 [原上游 README](docs/reference/upstream-README.md)。已有的算法/控制说明作为历史参考保留，若与代码不一致，以当前代码及实测结果为准。

本次整理所做的移动、路径调整、凭据替换和排除项见 [整理记录](docs/import-notes.md)。
