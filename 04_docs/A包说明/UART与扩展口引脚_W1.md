# HX4S20C（EG4S20BG256）UART / 扩展口引脚 调查结论

调查日期：本次会话 ｜ 结论分三档：【已证实】/【来源冲突，需上板确认】/【未找到】

---

## (a) UART 接收引脚

### 【已证实】`uart_rx = F12`（输入，PULLUP），`uart_tx = D12`（输出）

三个互相独立的证据：

| # | 来源 | 原文 / 内容 |
|---|---|---|
| 1 | 《HX4S20 开发板手册 HDL版2410》 **p.11** §⑦USB转UART | “串口引脚分配表：**RXD → F12**，**TXD → D12**”；同节说明板载 **CH340** 芯片做 USB 转串口，接口是板左侧 **Type-C**（实物图标识 ⑦USB-UART，兼 5V 供电） |
| 2 | 板卡原理图 `3_原理图\开发板原理图\2_FPGA.pdf` | FPGA 符号上网络标号 `RXD` 与球号 `F12` 同一引脚行；`TXD` 与 `D12` 同一行（与手册一致） |
| 3 | 官方 DEMO《16_串口通信设计》pin 约束（随板资料，`2_Example\DEMO实验介绍\`） | `set_pin_assignment { uart_rx } { LOCATION = F12; IOSTANDARD = LVCMOS33; PULLTYPE = PULLUP; }` / `set_pin_assignment { uart_tx } { LOCATION = D12; ... DRIVESTRENGTH = 8; PULLTYPE = NONE; }`；clk = R7，rst_n = A2，115200 8N1 环回 |

**注意**：RX 一定要 `PULLTYPE = PULLUP`（UART 空闲为高，防止悬空误判起始位）。

**占用冲突检查**：F12 / D12
- 不在工程 `pin.adc` 的任何一行；
- 不出现在手册的任何其它引脚表（LED / 按键 / 拨码 / 数码管 / HDMI A·B / VGA / JTAG / TF 卡 / QSPI / 以太网 / 蜂鸣器）；
- **不在两个 40 芯扩展口 J1/J2 上**，是 CH340 专用引脚。
→ 结论：**可直接使用，无冲突**。

---

## (b) 扩展口引脚分配表（手册原表，逐条照录）

手册正文把两个 40 芯口叫法有点乱（正文 ③/⑩ 都写 “J2”，实物图注写 JP1/JP2）。按手册自己的表头：**J2 表 = GPIOB**，**J1 表 = GPIOA**；原理图 `4_GPIO.pdf` 与手册的**接插件针号↔网络名完全一致**（已逐行核对：J1 第 1 脚 = GPIOA_0，第 4 脚 = GPIOA_3，第 11 脚 = 5V，第 12 脚 = GND，第 29 脚 = 3.3V，第 30 脚 = GND …）。

### GPIOB 扩展口（手册 “J2 40芯扩展接口”，实物图左侧 JP2）

| 针号 | 信号 | FPGA 引脚 | 针号 | 信号 | FPGA 引脚 |
|---|---|---|---|---|---|
| 1 | GPIOB_0 | P4 | 2 | GPIOB_1 | N4 |
| 3 | GPIOB_2 | N3 | 4 | GPIOB_3 | M5 |
| 5 | GPIOB_4 | M3 | 6 | GPIOB_5 | M4 |
| 7 | GPIOB_6 | L3 | 8 | GPIOB_7 | J3 |
| 9 | GPIOB_8 | J4 | 10 | GPIOB_9 | H4 |
| 11 | **VCC5V** | — | 12 | **GND** | — |
| 13 | GPIOB_10 | N1 | 14 | GPIOB_11 | M2 |
| 15 | GPIOB_12 | M1 | 16 | GPIOB_13 | L1 |
| 17 | GPIOB_14 | H2 | 18 | GPIOB_15 | H1 |
| 19 | GPIOB_16 | G1 | 20 | GPIOB_17 | G3 |
| 21 | GPIOB_18 | L4 | 22 | GPIOB_19 | L5 |
| 23 | GPIOB_20 | K6 | 24 | GPIOB_21 | H3 |
| 25 | GPIOB_22 | H5 | 26 | GPIOB_23 | F3 |
| 27 | GPIOB_24 | E2 | 28 | GPIOB_25 | J11 |
| 29 | **3.3V** | — | 30 | **GND** | — |
| 31 | GPIOB_26 | J12 | 32 | GPIOB_27 | K11 |
| 33 | GPIOB_28 | K14 | 34 | GPIOB_29 | L12 |
| 35 | GPIOB_30 | M12 | 36 | GPIOB_31 | M11 |
| 37 | GPIOB_32 | N12 | 38 | GPIOB_33 | P14 |
| 39 | GPIOB_34 | P12 | 40 | GPIOB_35 | R12 |

### GPIOA 扩展口（手册 “J1 扩展口”，实物图右侧 J1）

| 针号 | 信号 | FPGA 引脚 | 针号 | 信号 | FPGA 引脚 |
|---|---|---|---|---|---|
| 1 | GPIOA_0 | D14 | 2 | GPIOA_1 | G11 |
| 3 | GPIOA_2 | G12 | **4** | **GPIOA_3** | **F13** ← I2S LRCK |
| 5 | GPIOA_4 | H13 | 6 | GPIOA_5 | H14 |
| 7 | GPIOA_6 | J14 | **8** | **GPIOA_7** | **J13** ← I2S BCLK |
| 9 | GPIOA_8 | K12 | **10** | **GPIOA_9** | **L14** ← I2S DOUT |
| 11 | **VCC5V** | — | 12 | **GND** | — |
| 13 | GPIOA_10 | M14 | 14 | GPIOA_11 | P11 |
| 15 | GPIOA_12 | L10 | 16 | GPIOA_13 | M10 |
| 17 | GPIOA_14 | D16 | 18 | GPIOA_15 | E15 |
| 19 | GPIOA_16 | F14 | 20 | GPIOA_17 | F15 |
| 21 | GPIOA_18 | G16 | 22 | GPIOA_19 | G14 |
| 23 | GPIOA_20 | K15 | 24 | GPIOA_21 | J16 |
| 25 | GPIOA_22 | H15 | 26 | GPIOA_23 | H16 |
| 27 | GPIOA_24 | K16 | 28 | GPIOA_25 | M16 |
| 29 | **3.3V** | — | 30 | **GND** | — |
| 31 | GPIOA_26 | M15 | 32 | GPIOA_27 | L16 |
| 33 | GPIOA_28 | N16 | 34 | GPIOA_29 | N14 |
| 35 | GPIOA_30 | P16 | 36 | GPIOA_31 | P15 |
| 37 | GPIOA_32 | R16 | 38 | GPIOA_33 | R15 |
| 39 | GPIOA_34 | T15 | 40 | GPIOA_35 | R14 |

（手册注：J1 第 1 脚在**左上**，J2 第 1 脚在**右下**，锁定引脚时按实物缺口方向对照。）

---

## (c) 关于 “J4 pin 3 / 7 / 9”

- **板上没有承载 I2S 的 J4 排针。** 原理图里 **J4 = VGA 连接器（DB15）**，在 `5_DIG_VGA.pdf`（符号同时标注 “VGA”）。因此 `pin.adc` 里 `#J4 pin 7` 这类注释是**错的/不可用的来源**。
- `F13 / J13 / L14` 按手册 J1 表 = **GPIOA_3 / GPIOA_7 / GPIOA_9**，即 **J1 扩展口第 4 / 8 / 10 脚**。注释里的 3 / 7 / 9 恰好是 GPIOA 信号的序号，而不是针号 —— 这是唯一自洽的解释。
- 【来源冲突，需上板确认】板卡原理图 `2_FPGA.pdf` 给这三只球标的网络名却是 `GPIOA7`(F13) / `GPIOA10`(J13) / `GPIOA24`(L14)，与手册 J1 表（GPIOA_3 / 7 / 9）**对不上**；J1 侧网络名是 `GPIOA_x` 系列，FPGA 侧是 `GPIOAx` 系列，两套名字之间的对应关系我在原理图上无法完全确认。
  → 若 I2S 输入是否真的落在 J1 第 4/8/10 脚对项目有影响，建议**用万用表/示波器在板上实测连通性**，不要只信手册表。
  → 这一冲突**不影响 (a) 的 UART 结论**（F12/D12 不经过任何扩展口）。

---

## 附：manual_img 16 张图的实际内容（我逐张看过）

| 文件 | 内容 | 与串口相关？ |
|---|---|---|
| img_001.jpg | KONXIN 厂标 | 否 |
| img_002.jpg | “开发板使用手册”标题 | 否 |
| img_003.jpg | **开发板实物图**，标注 ①FPGA ②4组LED ③GPIO扩展口JP2 ④HDMI A ⑤电源开关 ⑥USB-JTAG **⑦USB-UART** ⑧HDMI B ⑨VGA ⑩GPIO扩展口J1 ⑪数码管 … ⑳固化FLASH | 只证明有 USB-UART 口 |
| img_004.jpg | EG4S20NG88/BG256/CG324 资源对比表（本项目 BG256） | 否 |
| img_005.jpg | **J2 / IDC40 扩展口原理图**：GPIOB_0~35、5V、3V3、GND | 否（无球号） |
| img_006.jpg | HDMI 的 AZ1045-04F ESD 阵列 | 否 |
| img_007.jpg | 电源拓扑框图 | 否 |
| img_008.jpg | FPGA 配置/JTAG 部分（TCK/TDI/TDO/TMS） | 否 |
| **img_009.jpg** | **CH340G 原理图**（U41，12MHz 晶振，Type-C：DM/DP→UD+/UD−），FPGA 侧网络标号 `RXD`(接 CH340 TXD) 和 `TXD`(接 CH340 RXD) | **是，但图中没有 FPGA 球号** |
| img_010.jpg | HDMI_B 差分/DDC 原理图 | 否 |
| img_011.jpg | VGA 电阻网络 + VGA 连接器原理图 | 否 |
| img_012.jpg | **J1 / IDC40 扩展口原理图**：GPIOA_0~35、5V、3V3、GND | 否（无球号） |
| img_013.jpg | 数码管/双色 LED 位选驱动（PNP + RN） | 否 |
| img_014.jpg | 8 组双色 LED 原理图 | 否 |
| img_015.jpg | RTL8211E 千兆 PHY | 否 |
| img_016.jpg | W25Q16 固化 FLASH | 否 |

`schem_img\` 只有 `cfg.png`（一张空白图，2000×1500，无内容）。

---

## 主要引用文件（本地）

- `C:\Users\jwen\Desktop\HX4S20开发板手册-HDL版2410_书签整理.pdf`（另有 `HX4S20_Contest_202606\9_康芯开发板使用手册及实验平台\` 下同名副本）
- `C:\Users\jwen\Desktop\HX4S20_Contest_202606\2_Example\DEMO实验介绍\16_串口通信设计.pdf`
- `C:\Users\jwen\Desktop\HX4S20_Contest_202606\3_原理图\开发板原理图\2_FPGA.pdf`、`4_GPIO.pdf`、`5_DIG_VGA.pdf`
- 提取出的纯文本：`C:\Users\jwen\Desktop\anlu\_x\`（manual.txt / fpga.txt / gpio.txt / digvga.txt 等）
