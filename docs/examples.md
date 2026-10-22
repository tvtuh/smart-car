# 例程索引

每个例程是独立程序，入口均为其 `mdk/seekfree.uvproj`。以下功能说明根据目录、源码和当前配置整理，不将旧文档中的性能描述当作已验证结论。

## E01 · 基础输入输出

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E01_01_button_demo](../firmware/Motherboard_Demo/E01_01_button_demo/README.md) | **按键与拨码开关**：读取按键状态并显示计数；默认代码使用 IPS200 屏幕。 | [打开工程](../firmware/Motherboard_Demo/E01_01_button_demo/mdk/seekfree.uvproj) |
| [E01_02_buzzer_demo](../firmware/Motherboard_Demo/E01_02_buzzer_demo/README.md) | **蜂鸣器**：验证蜂鸣器 GPIO 控制，适合作为基础上电检查。 | [打开工程](../firmware/Motherboard_Demo/E01_02_buzzer_demo/mdk/seekfree.uvproj) |
| [E01_03_hall_stopline_detection_demo](../firmware/Motherboard_Demo/E01_03_hall_stopline_detection_demo/README.md) | **霍尔停车线检测**：读取霍尔传感器，用于停车线检测实验。 | [打开工程](../firmware/Motherboard_Demo/E01_03_hall_stopline_detection_demo/mdk/seekfree.uvproj) |
| [E01_04_laser_head_demo](../firmware/Motherboard_Demo/E01_04_laser_head_demo/README.md) | **激光头**：演示激光头相关的接口控制。 | [打开工程](../firmware/Motherboard_Demo/E01_04_laser_head_demo/mdk/seekfree.uvproj) |

## E02 · 电机与舵机

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E02_01_hip4082_double_motor_contro_demo](../firmware/Motherboard_Demo/E02_01_hip4082_double_motor_contro_demo/README.md) | **HIP4082 双电机**：验证 HIP4082 驱动板上的双电机输出。 | [打开工程](../firmware/Motherboard_Demo/E02_01_hip4082_double_motor_contro_demo/mdk/seekfree.uvproj) |
| [E02_02_drv8701e_double_motor_contro_demo](../firmware/Motherboard_Demo/E02_02_drv8701e_double_motor_contro_demo/README.md) | **DRV8701E 双电机**：验证 DRV8701E 驱动板上的双电机输出。 | [打开工程](../firmware/Motherboard_Demo/E02_02_drv8701e_double_motor_contro_demo/mdk/seekfree.uvproj) |
| [E02_03_servo_control_demo](../firmware/Motherboard_Demo/E02_03_servo_control_demo/README.md) | **舵机 PWM**：演示 PWM 舵机控制；原例程会往返转动，请先解除机械负载。 | [打开工程](../firmware/Motherboard_Demo/E02_03_servo_control_demo/mdk/seekfree.uvproj) |
| [E02_04_brushless_electric_regulating_demo](../firmware/Motherboard_Demo/E02_04_brushless_electric_regulating_demo/README.md) | **无刷电调**：演示无刷电调输出，接入前核对电调协议与供电。 | [打开工程](../firmware/Motherboard_Demo/E02_04_brushless_electric_regulating_demo/mdk/seekfree.uvproj) |

## E03 · 编码器

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E03_01_encoder_dir_demo](../firmware/Motherboard_Demo/E03_01_encoder_dir_demo/README.md) | **带方向编码器**：读取脉冲加方向类型的编码器。 | [打开工程](../firmware/Motherboard_Demo/E03_01_encoder_dir_demo/mdk/seekfree.uvproj) |
| [E03_02_encoder_quad_demo](../firmware/Motherboard_Demo/E03_02_encoder_quad_demo/README.md) | **正交编码器**：读取 A/B 两相正交编码器。 | [打开工程](../firmware/Motherboard_Demo/E03_02_encoder_quad_demo/mdk/seekfree.uvproj) |

## E04 · 模拟采集

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E04_01_adc_capture_demo](../firmware/Motherboard_Demo/E04_01_adc_capture_demo/README.md) | **ADC 采样**：演示模拟输入采样。 | [打开工程](../firmware/Motherboard_Demo/E04_01_adc_capture_demo/mdk/seekfree.uvproj) |
| [E04_02_battery_voltage_detection_demo](../firmware/Motherboard_Demo/E04_02_battery_voltage_detection_demo/README.md) | **电池电压检测**：读取电池电压检测通道；分压比例以实际主板和源码为准。 | [打开工程](../firmware/Motherboard_Demo/E04_02_battery_voltage_detection_demo/mdk/seekfree.uvproj) |

## E05 · 惯性传感器

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E05_03_imu660ra_demo](../firmware/Motherboard_Demo/E05_03_imu660ra_demo/README.md) | **IMU660RA**：读取 IMU660RA 传感器数据。 | [打开工程](../firmware/Motherboard_Demo/E05_03_imu660ra_demo/mdk/seekfree.uvproj) |
| [E05_04_imu660rb_demo](../firmware/Motherboard_Demo/E05_04_imu660rb_demo/README.md) | **IMU660RB**：读取 IMU660RB 传感器数据。 | [打开工程](../firmware/Motherboard_Demo/E05_04_imu660rb_demo/mdk/seekfree.uvproj) |
| [E05_05_imu963ra_demo](../firmware/Motherboard_Demo/E05_05_imu963ra_demo/README.md) | **IMU963RA**：读取 IMU963RA 传感器数据。 | [打开工程](../firmware/Motherboard_Demo/E05_05_imu963ra_demo/mdk/seekfree.uvproj) |
| [E05_06_imu660rc_demo](../firmware/Motherboard_Demo/E05_06_imu660rc_demo/README.md) | **IMU660RC**：读取 IMU660RC 传感器数据。 | [打开工程](../firmware/Motherboard_Demo/E05_06_imu660rc_demo/mdk/seekfree.uvproj) |

## E06 · 屏幕与图形控件

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E06_02_tft180_display_demo](../firmware/Motherboard_Demo/E06_02_tft180_display_demo/README.md) | **TFT180 显示**：验证 TFT180 显示接口。 | [打开工程](../firmware/Motherboard_Demo/E06_02_tft180_display_demo/mdk/seekfree.uvproj) |
| [E06_03_ips114_display_demo](../firmware/Motherboard_Demo/E06_03_ips114_display_demo/README.md) | **IPS114 显示**：验证 IPS114 显示接口。 | [打开工程](../firmware/Motherboard_Demo/E06_03_ips114_display_demo/mdk/seekfree.uvproj) |
| [E06_04_ips200_display_demo](../firmware/Motherboard_Demo/E06_04_ips200_display_demo/README.md) | **IPS200 显示**：验证 IPS200 显示接口。 | [打开工程](../firmware/Motherboard_Demo/E06_04_ips200_display_demo/mdk/seekfree.uvproj) |
| [E06_05_ips200pro_page](../firmware/Motherboard_Demo/E06_05_ips200pro_page/README.md) | **IPS200PRO 页面**：演示页面控件。 | [打开工程](../firmware/Motherboard_Demo/E06_05_ips200pro_page/mdk/seekfree.uvproj) |
| [E06_06_ips200pro_label](../firmware/Motherboard_Demo/E06_06_ips200pro_label/README.md) | **IPS200PRO 标签**：演示文字标签控件；原目录误放的摄像头副本已单独归档。 | [打开工程](../firmware/Motherboard_Demo/E06_06_ips200pro_label/mdk/seekfree.uvproj) |
| [E06_07_ips200pro_table](../firmware/Motherboard_Demo/E06_07_ips200pro_table/README.md) | **IPS200PRO 表格**：演示表格控件。 | [打开工程](../firmware/Motherboard_Demo/E06_07_ips200pro_table/mdk/seekfree.uvproj) |
| [E06_08_ips200pro_meter](../firmware/Motherboard_Demo/E06_08_ips200pro_meter/README.md) | **IPS200PRO 仪表**：演示仪表控件。 | [打开工程](../firmware/Motherboard_Demo/E06_08_ips200pro_meter/mdk/seekfree.uvproj) |
| [E06_09_ips200pro_clock](../firmware/Motherboard_Demo/E06_09_ips200pro_clock/README.md) | **IPS200PRO 时钟**：演示时钟控件。 | [打开工程](../firmware/Motherboard_Demo/E06_09_ips200pro_clock/mdk/seekfree.uvproj) |
| [E06_10_ips200pro_progress_bar](../firmware/Motherboard_Demo/E06_10_ips200pro_progress_bar/README.md) | **IPS200PRO 进度条**：演示进度条控件。 | [打开工程](../firmware/Motherboard_Demo/E06_10_ips200pro_progress_bar/mdk/seekfree.uvproj) |
| [E06_11_ips200pro_calendar](../firmware/Motherboard_Demo/E06_11_ips200pro_calendar/README.md) | **IPS200PRO 日历**：演示日历控件。 | [打开工程](../firmware/Motherboard_Demo/E06_11_ips200pro_calendar/mdk/seekfree.uvproj) |
| [E06_12_ips200pro_waveform](../firmware/Motherboard_Demo/E06_12_ips200pro_waveform/README.md) | **IPS200PRO 波形**：演示波形控件。 | [打开工程](../firmware/Motherboard_Demo/E06_12_ips200pro_waveform/mdk/seekfree.uvproj) |
| [E06_13_ips200pro_container](../firmware/Motherboard_Demo/E06_13_ips200pro_container/README.md) | **IPS200PRO 容器**：演示容器控件。 | [打开工程](../firmware/Motherboard_Demo/E06_13_ips200pro_container/mdk/seekfree.uvproj) |
| [E06_14_ips200pro_mt9v03x](../firmware/Motherboard_Demo/E06_14_ips200pro_mt9v03x/README.md) | **IPS200PRO 摄像头图像**：在 IPS200PRO 上显示 MT9V03X 图像。 | [打开工程](../firmware/Motherboard_Demo/E06_14_ips200pro_mt9v03x/mdk/seekfree.uvproj) |

## E07 · 通信与视觉开发

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E07_01_wireless_uart_demo](../firmware/Motherboard_Demo/E07_01_wireless_uart_demo/README.md) | **无线串口**：验证无线串口收发。 | [打开工程](../firmware/Motherboard_Demo/E07_01_wireless_uart_demo/mdk/seekfree.uvproj) |
| [E07_07_wifi_spi_tcp_client_demo](../firmware/Motherboard_Demo/E07_07_wifi_spi_tcp_client_demo/README.md) | **Wi-Fi SPI TCP 客户端**：通过 Wi-Fi SPI 模块连接 TCP 服务端。 | [打开工程](../firmware/Motherboard_Demo/E07_07_wifi_spi_tcp_client_demo/mdk/seekfree.uvproj) |
| [E07_08_wifi_spi_oscilloscope_demo](../firmware/Motherboard_Demo/E07_08_wifi_spi_oscilloscope_demo/README.md) | **Wi-Fi SPI 虚拟示波器**：向逐飞助手发送示波器数据。 | [打开工程](../firmware/Motherboard_Demo/E07_08_wifi_spi_oscilloscope_demo/mdk/seekfree.uvproj) |
| [E07_09_wifi_spi_mt9v03x_demo](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo/README.md) | **摄像头视觉/控制开发版 fw62**：含图像处理与电机/舵机控制，和整合版是独立工程。 | [打开工程](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo/mdk/seekfree.uvproj) |
| [E07_09_wifi_spi_mt9v03x_demo_left_edge_fix](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/README.md) | **整车整合版（历史目录名 left_edge_fix）**：包含视觉处理、舵机控制与双电机驱动；固件标记 fw103-preview42，编码器速度闭环当前默认关闭。 | [打开工程](../firmware/Motherboard_Demo/E07_09_wifi_spi_mt9v03x_demo_left_edge_fix/mdk/seekfree.uvproj) |

## E08 · 距离传感器

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E08_08_dl1a_demo](../firmware/Motherboard_Demo/E08_08_dl1a_demo/README.md) | **DL1A 测距**：读取 DL1A 测距模块。 | [打开工程](../firmware/Motherboard_Demo/E08_08_dl1a_demo/mdk/seekfree.uvproj) |
| [E08_09_dl1b_demo](../firmware/Motherboard_Demo/E08_09_dl1b_demo/README.md) | **DL1B 测距**：读取 DL1B 测距模块。 | [打开工程](../firmware/Motherboard_Demo/E08_09_dl1b_demo/mdk/seekfree.uvproj) |

## E09 · 摄像头显示与上传

| 例程 | 功能 | 工程入口 |
|---|---|---|
| [E09_01_mt9v03x_usbcdc_seekfree_assistant_demo](../firmware/Motherboard_Demo/E09_01_mt9v03x_usbcdc_seekfree_assistant_demo/README.md) | **USB CDC 图像上传**：通过 USB CDC 向逐飞助手传送摄像头图像。 | [打开工程](../firmware/Motherboard_Demo/E09_01_mt9v03x_usbcdc_seekfree_assistant_demo/mdk/seekfree.uvproj) |
| [E09_06_mt9v03x_tft180_display_demo](../firmware/Motherboard_Demo/E09_06_mt9v03x_tft180_display_demo/README.md) | **摄像头 + TFT180**：在 TFT180 上显示摄像头图像。 | [打开工程](../firmware/Motherboard_Demo/E09_06_mt9v03x_tft180_display_demo/mdk/seekfree.uvproj) |
| [E09_07_mt9v03x_ips114_display_demo](../firmware/Motherboard_Demo/E09_07_mt9v03x_ips114_display_demo/README.md) | **摄像头 + IPS114**：在 IPS114 上显示摄像头图像。 | [打开工程](../firmware/Motherboard_Demo/E09_07_mt9v03x_ips114_display_demo/mdk/seekfree.uvproj) |
| [E09_08_mt9v03x_ips200_display_demo](../firmware/Motherboard_Demo/E09_08_mt9v03x_ips200_display_demo/README.md) | **摄像头 + IPS200**：在 IPS200 上显示摄像头图像。 | [打开工程](../firmware/Motherboard_Demo/E09_08_mt9v03x_ips200_display_demo/mdk/seekfree.uvproj) |

## 历史副本

[摄像头历史副本](../firmware/Motherboard_Demo/archive/E07_09_wifi_spi_mt9v03x_demo_label_snapshot/README.md) 原来嵌套在 IPS200PRO 标签例程内；现在单独归档用于比对，不作为默认开发入口。

原始列表：[例程说明.xlsx](reference/例程说明.xlsx)。
