# 共享库

所有例程共用本目录。

| 目录 | 职责 |
|---|---|
| zf_common | 时钟、基础类型、启动文件、调试与通用函数 |
| zf_driver | GPIO、PWM、ADC、串口、定时器、编码器等底层驱动 |
| zf_device | 摄像头、屏幕、Wi-Fi、惯性传感器、测距等设备驱动 |
| zf_components | USB CDC、逐飞助手通信组件 |
| doc | [原始版本记录](doc/version.txt) |

版本记录首项为 V3.2.0。`.LIB` 文件是原工程引用的必要输入，已保留；`.LST` 和 `.obj` 等编译输出已排除。

源码头部的作者与授权声明保留。随原资料携带的许可证见 [LICENSE](LICENSE)。
