# OV2640 — ESP32-S3 DVP 摄像头 + ST7735S LCD

OV2640 摄像头（DVP 8 位输出，模块板载 12 MHz 晶振）采集 raw YUV422 160x120，转 RGB565 缩放至 128x96 后显示到 ST7735S TFT（SPI），并打印帧率。

## 硬件接线

| 外设 | 信号 | GPIO |
| --- | --- | --- |
| OV2640 DVP | D0 – D7 | 4 – 11 |
| | PCLK / VSYNC / HREF | 12 / 13 / 14 |
| | XCLK | 不接（模块自带 12 MHz 晶振，固件不驱动） |
| | SIOD / SIOC（SCCB） | 16 / 17 |
| | PWDN / RST | 18 / 21 |
| ST7735S SPI | SCLK / MOSI / CS / DC / RST | 41 / 42 / 40 / 39 / 38 |

引脚在 `main/main.c` 开头的 `PIN_CAM_*` / `PIN_LCD_*` 宏处修改。

## 构建与烧录

Git Bash / MSYS 下 idf.py 会拒绝运行，用根目录 `build.bat`（经 cmd 调 idf.py，手动设置 IDF_PATH 与工具链 PATH）：

```sh
cmd //c "E:\esp\OV2640\build.bat build"
cmd //c "E:\esp\OV2640\build.bat -p COMx flash"
```

## 目录结构

- `main/` — 采集 → YUV422 转 RGB565 + 缩放 → 队列 → LCD 刷新 + 帧率 的流水线
- `components/camera` — esp_driver_cam（esp_cam_ctlr DVP）采集封装
- `components/ov2640` — SCCB 寄存器配置（移植自 STM32-OV2640-master，寄存器表逐字节校验一致）
- `components/st7735s` — ST7735S 驱动（esp_driver_spi，SPI2 20 MHz）

## 关键实现说明

- **raw 输出模式**：STM32 参考工程是 JPEG 流水线，其 `OV2640_YUV422`/`OV2640_JPEG` 表把 DSP 的 IMAGE_MODE（0xDA）置 0x10（JPEG 码流）。raw DVP 定长帧采集必须置 0x00（见 `OV2640_YUV422_RAW` 表），并强制精确输出尺寸——JPEG 分辨率表的输出窗口单位是 4 像素，"160x120" 表实际输出 176x144。
- **无 PSRAM 的 esp_cache_msync 问题**：IDF 的 `esp_cache_msync` 只接受缓存映射区地址（flash/PSRAM），而 esp_cam_ctlr 启动 DMA 前会无条件对帧缓冲调用它——帧缓冲在内部 SRAM 时（无 PSRAM 板）报 `invalid addr or null pointer`，`esp_cam_ctlr_start` 失败。`components/camera` 用 `-Wl,--wrap=esp_cache_msync` 垫片对内部 SRAM 地址跳过同步（S3 内部 RAM 不经过缓存，CPU/DMA 天然一致，同步本是空操作）。
- **模块自带晶振**：`PIN_CAM_XCLK = GPIO_NUM_NC` 时 `external_xtal = true`，不让 GPIO15 输出时钟去打模块晶振（否则 SCCB 总线受干扰，表现为随机 NACK）。
