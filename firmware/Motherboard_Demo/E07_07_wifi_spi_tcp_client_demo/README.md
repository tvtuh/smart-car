# Wi-Fi SPI TCP 客户端

通过 Wi-Fi SPI 模块连接 TCP 服务端。

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

[返回例程索引](../../../docs/examples.md)
