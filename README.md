# ESP32-C3 Linux 示例项目

基于 ESP-IDF 框架的 ESP32-C3 开发示例集合。

## 环境要求

- **芯片**: ESP32-C3
- **开发框架**: [ESP-IDF](https://github.com/espressif/esp-idf) (v5.0+)
- **操作系统**: Linux / WSL

## 快速开始

```bash
# 1. 激活 ESP-IDF 环境
. ~/esp/esp-idf/export.sh

# 2. 编译烧录
idf.py set-target esp32c3
idf.py build flash monitor
```

## 相关资源

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/index.html)
- [ESP32-C3 技术参考手册](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_cn.pdf)
