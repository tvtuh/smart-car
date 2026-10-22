# 来源与第三方说明

## 逐飞科技 STC32G144K 开源库

导入内容来自用户本地的 STC32G144K246 100Pin 开源库主板例程及其已有开发版本。原文件头标注 Copyright (c) 2025 SEEKFREE 逐飞科技，并声明 GPL 第 3 版或其后版本；这些文件头予以保留。

- 原始说明：[upstream-README.md](docs/reference/upstream-README.md) 与 [英文 README](docs/reference/upstream-README.en.md)。
- 随原资料携带的许可文本：[firmware/Motherboard_Demo/LICENSE](firmware/Motherboard_Demo/LICENSE)。
- 共享库原始版本记录：[version.txt](firmware/Motherboard_Demo/libraries/doc/version.txt)。

原源码注释提到 `libraries/doc/GPL3_permission_statement.txt`，但本地资料中没有这个文件；本次未伪造该文件，改为补齐随父目录资料已有的完整 LICENSE 副本。

## 必要的预编译库

`zf_device_config.LIB` 与 `zf_device_mt9v03x_dma.LIB` 是原 Keil 工程直接引用的输入，已保留原字节。这里没有为它们补造源码或重新声明授权范围；具体授权以原始资料和权利方提供的信息为准。

## 整理与本地开发版本

本仓库保留本地已有的视觉/整车修改和对应作者声明。没有将这些代码宣称为团队从零编写，也没有将已有许可统一替换成其他许可。来源、调整和排除项见 [整理记录](docs/import-notes.md)。
