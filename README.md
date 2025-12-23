# 使用串口传输显示动画工程说明

## 一、工程简介

本工程基于 **STM32G431**，利用板载并口驱动的 TFT LCD，在 MCU 端通过串口接收上位机发送的图像数据，并在 LCD 上连续刷新，实现“播放动画”的效果。

主要特性：
- 使用板载并口 LCD 驱动（`lcd.c` / `lcd.h`）。
- 使用 **USART1** 以 **1 Mbps** 波特率从上位机接收图像数据。
- 采用 **双缓冲机制** (`uart_rx_buf1` / `uart_rx_buf2`) 与中断接收，避免接收与刷新冲突。
- 通过 `LCD_UartPic()` 函数将一帧数据绘制到 LCD 指定区域，实现单色动画播放。
---
## 二、硬件连接

- MCU：STM32G431RBTx。
- LCD：并口驱动，已在 `lcd.c` 中完成时序驱动和初始化。
- 串口：
  - USART1_TX：PA9
  - USART1_RX：PA10
- 上位机：通过 USB 转串口或板载虚拟串口连接到 USART1（蓝桥杯板子下载口已经接入USART1可以直接使用），对应波特率 **1000000**（1 Mbps），8N1，无校验。

---
## 三、文件结构与关键函数

### 1. 主要文件

- `Src/main.c`
  - `MX_UART1_Init()`：配置 USART1 时钟、GPIO、波特率和接收中断。
  - `uart_rx_buf1` / `uart_rx_buf2`：两个 6 KB 接收缓冲区，轮换存放一帧图像数据。
  - `buf` / `disp`：
    - `buf`：当前正在接收使用的缓冲区（1 -> buf1，2 -> buf2）。
    - `disp`：主循环显示标志（0 不显示，1 显示 buf1，2 显示 buf2）。
  - `main()`：轮询 `disp`，在 `disp` 置位时调用 `LCD_UartPic()` 显示对应缓冲区的内容。

- `Src/stm32g4xx_it.c`
  - `USART1_IRQHandler()`：
    - 按字节接收 USART1 数据，通过 RXNE 中断读取 `USART1->RDR`。
    - 将数据填入当前缓冲区 `uart_rx_ptr`，计数 `uart_rx_index` 达到一帧长度（4800 字节）后：
      - 切换缓冲区（buf1 <-> buf2）。
      - 设置 `disp` 标志，通知 `main()` 去刷新 LCD。

- `Src/lcd.c` / `Inc/lcd.h`
  - `LCD_Init()`、`LCD_Clear()`、`LCD_SetTextColor()` 等：基础 LCD 驱动接口。
  - `LCD_UartPic(uc8 *c)`：
    - 将一帧串口数据渲染成单色图像，显示在 LCD 底部指定区域。

### 2. 图像数据格式（协议说明）

`LCD_UartPic()` 的实现大致如下（简化描述）：

- 逻辑显示区域：
  - X 方向：`x = 40 .. 199`，一共 **160 列**。
  - Y 方向：内部循环 `y = 0 .. 29`，外加每字节 8 位 -> 高度为 **30 * 8 = 240 像素**。
- 索引：从 `c[1]` 开始使用数据（`index` 初值为 1）。
- 每一列使用 `30` 个字节，每个字节的 8 位依次表示 8 个像素点（单色）：
  - 某一列所需字节数：30
  - 总列数：160
  - 每帧字节总数：`160 * 30 = 4800` 字节

因此，上位机每发送一帧数据时：

- 一帧大小：**4800 字节**。
- 格式：`frame[k]` 的每一位对应一个像素：
  - 位为 0 -> 显示背景色 `BackColor`。
  - 位为 1 -> 显示前景色 `TextColor`。

---
## 四、上位机软件下载与配置
 - [上位机开源仓库下载地址](https://github.com/AnChangNice/oled_display_gui)从这里下载
### 相关配置点击 Scan Port 扫描到串口后，连接对应串口，相关参数配置如下：
![配置](/docs/img/1.png)
- 点击 `Open Sample Windows`把框放入想要显示区域,点击`Start Sending`即可开始发送
---
## 效果演示  
![演示1](/docs/img/1.gif)

![演示2](/docs/img/2.gif)



