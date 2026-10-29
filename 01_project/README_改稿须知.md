# Task C · v7f 交付包（给 A 改）

> 这一版是 C（音频方向）上板确认过的完整工程：**图片 + 音频 + 频谱 + 字幕 全部正常**。
> 包里有完整源码、码流、TD 脚本、时序报告、SD 音频镜像，A 改完可以直接复现和比对。

打包时间：2026-10-29

---

## 1. 码流（先确认能跑）

```
bit\TaskC_v7f.bit
SHA256  EA78381121BCDB397F7A314E67D911BA47860979370F1EB79603A552A95372F6
```

- **SD 卡必须已经是约定状态**（见 §7），否则只能看到图片、没有声音
- 烧完**完全断电再上电一次**（音量参数要复位）
- SW3 拨上 = 紧急，拨下 = 正常；KEY2 = 手动下一张，KEY3 = 轮播开关，KEY4 = 参数项

---

## 2. 目录结构

```
TaskC_v7f_20261029\
  bit\          TaskC_v7f.bit + .sha256
  src\          全部 55 个源文件（含 pin_audio_sd.adc / timing.sdc）
  user_source\  hdl_source\include\global_def.v（sdram.v 的 include，必须有）
  impl.tcl      完整流程：综合→opt→place→route→bitgen
  syn.tcl       只跑到综合（快速检查语法/资源）
  report\       route.qor（关键）、v7f_route.timing、v7f_phy.area
  sd_audio\     audio_sd.bin（SD 音频镜像，写入用）
  tools\        逐位模型：loud_model.py（音量链）、viz_probe.py（频谱前端）
  SRC_MANIFEST.sha256   每个源文件的哈希
  verify_package.ps1    自查包有没有被改动
```

---

## 3. 怎么自己编译

**先把整个文件夹解压到纯英文路径**（例如 `D:\work\TaskC_v7f`），
TD 对中文路径不稳，报错还很隐晦。

**只能用 `td_commands_prompt.exe`，不能用 `td.exe -batch`（后者只开 GUI 不执行）：**

```powershell
cd D:\work\TaskC_v7f
D:\TD_6.2.1\bin\td_commands_prompt.exe impl.tcl
```

注意事项：

1. **`impl.tcl` 必须是 ASCII / 无 BOM**。带 BOM 会被当 GBK，报
   `invalid command name "閿樼袱mport_device"`。
2. 命令行传脚本路径**用正斜杠**：`D:/work/TaskC_v7f/impl.tcl`。用反斜杠时 `\` 会被吃掉
   （报 `couldn't read file "D:TaskC4impl.tcl"`）。
3. 重跑前先删掉上次的 `v7_impl.al` 和 `v7_impl\` 目录，否则报
   `PRJ-8646 ERROR: Project … already exists`。
4. `bitgen` 出来的 `.bit` **SHA 每次都会不同，这是正常的**（TD 的 bitgen 不是逐字节可复现）。
   **判断"是不是同一个设计"要看 `route.qor` 的 6 个数字，不是 SHA：**

```
Setup WNS  80        Setup TNS  0        violated S-EP  0
Hold  WNS   3        Hold  TNS  0        violated H-EP  0
#slices 7584   #RAMs 40   #DSPs 18   （LUT 约 13.3k / 19600）
```

---

## 4. 谁的地盘：改之前先看这一节

**A 的地盘（随便改）**

```
bmp_read.v  scaler_bilinear.v  frame_read_write.v  frame_fifo_read.v
frame_fifo_write.v  video_layers_top.v  mode_fsm.v  layer_mixer.v
osd_bar.v  text_layer.v  font_rom.v  color_bar.v  scroll_timer.v
latency_meter.v  video_timing_data.v  video_rgb_to_axis_640x480.v
video_delay.v  video_define.v  sdram.v  sdr_*.enc.v
```

**B 的地盘**

```
param_regs.v（含 C 为了音量新增的 output volume 端口，别删这个端口）
```

**C 的地盘（改了要联动，尽量别动）**

```
top_audio_sd.v      顶层接线 + vol_scale（音量）+ 音频 CDC
audio_viz.v         频谱（滤波器组 + 对数电平 + 柱子绘制）
audio_sd_stream.v   SD→FIFO→I2S 播放，音源切换
audio_hdmi_i2s.v    I2S/HDMI 音频打包 + boost_sat 增益曲线
I2S_receiver.v      （已从数据通路旁路，不再使用）
audio_arc_calculate.v  ACR 参数
sd_card_bmp.v       ★ 图片通路 + 【SD 总线音频/图片分时复用】← 半 A 半 C
```

**契约文件（不要动）**

```
pin_audio_sd.adc    引脚分配（《接口契约与分工表》/《板级接口与显示资源》）
timing.sdc          时序约束
seg_scan.v / seg_decoder.v  数码管分配（第 1/4/5/6 位）
```

---

## 5. 八条"一动就出事"的坑（都踩过）

1. **`sd_card_bmp.v` 的 `sector_lut` 必须保持 `function [31:0]`。**
   曾经收窄到 17 位省逻辑 → 直接 `PHY-8023` 布线失败 + 板上按 KEY3 崩溃复位。

2. **音频借 SD 总线的三个条件一个都不能少**（`sd_card_bmp.v`）：
   `aud_req && scan_done && !bmp_sd_sec_read && sd_fsm_idle && sd_init_done`。
   少 `sd_fsm_idle` 或 `!bmp_sd_sec_read` 就会在事务中途把总线抢走 →
   `bmp_read` 丢一个扇区 → 字节流错位 512 → 装载永远完不成 → 画面卡死。

3. **总线只能在事务边界归还**：`if (sd_sec_read_end || (aud_owner_timeout && sd_fsm_idle))`。
   去掉 `&& sd_fsm_idle` 就会重演"切 SW3 偶发卡死"。

4. **`band_h` 那条链**（`audio_viz.v`）：
   `band_h = lvl>>3 + lvl>>4`（= lvl*48/256，最高 47 px）。
   窗口是 `[336, 408)`，柱底固定 **y = 408**，字幕在 **y = 424 / 448**。

5. **`audio_viz.v` 的 `SPEC_X0/SPEC_X1/SPEC_Y_BASE/SPEC_Y_TOP`
   必须和 `top_audio_sd.v` 传给 `video_layers_top` 的 `SPEC_X/Y/W/H` 配套**
   （`layer_mixer` 是按区域**硬裁**的，两边不一致柱子会被切掉一半）。
   现在：`SPEC_Y=336, SPEC_H=72`，柱顶 `SPEC_Y_TOP=336`，柱底 `SPEC_Y_BASE=408`。

6. **`audio_hdmi_i2s.v` 的 `next24` 是符号扩展**（16 位样点放进 24 位端口**不左移**），
   音量全靠 `top_audio_sd.v` 里 `vol_scale` 的 `q = p;`（**不要加 `>>> 7`**）。
   加回右移 7 位 → 整体音量掉 **42 dB**（板上就是"声音小到几乎听不见"）。

7. **`sd_card_bmp.v` 的装载看门狗（2 s 无进展）自愈路径本身有缺陷**：
   超时后 `scan_kicked` 会在 `sd_init_done` 还没回来时被置 1，新扫描请求被 `bmp_read`
   丢掉 → `scan_done` 回不来 → 只能断电。**不要指望它兜底**，别让它有机会触发。

8. **`sd_card_bmp.v` 的 `SCAN_TARGET_COUNT`（现在 `4'd12`）必须等于卡上 BMP 张数**，
   而且**第 12 张（最后一张）是应急疏散图**，轮播折返点是 `count-2`，不会转到它。

---

## 6. 改完怎么自查

### 编译后先看 `route.qor`

| 项 | 期望 |
|---|---|
| `PHY-8023` / `RUN-8102` | **0 / 0**（出现就说明资源挤爆或布线冲突） |
| `#slices` | ≤ 7700（现在 7584 / 78% 左右，再涨很容易出 PHY-8023） |
| `#DSPs` | ≤ 29（现在 18） |
| `ext_mem_clk` SWNS | 现在 **−0.130 ns / 2 端点**，**这是 A 的 `frame_read_write` 老路径，原始基线就是 −0.132 / 1 端点**，不是新问题；只要别更差就行 |
| `sd_card_clk` 100 MHz | 现在 **+0.104 ns / 0 端点** |
| `video_clk` 25 MHz | 现在 **+0.349 ns / 0 端点** |

### 上板清单

| # | 操作 | 期望 |
|---|---|---|
| 1 | 上电 | 约 2.5 s 出第一张图，背景音乐（普通音源）响 |
| 2 | KEY2 手动切图 20 次 | 每次干净换一张，不回第一张、数码管不乱 |
| 3 | KEY3 开轮播 | 每 1 s 一张，不转到应急图 |
| 4 | SW3 拨上 | 字幕变红 + 约 0.5 s 出应急图 + 音频切应急 |
| 5 | **SW3 拨回** | **图片瞬间回到原来那张（不黑屏、不读卡）**，音频同步切回 |
| 6 | 来回拨 SW3 30 次 | 不卡死，不放出旧录音 |
| 7 | 打开 SW4 | 8 根柱子随音乐起伏，最高不碰到字幕 |
| 8 | KEY4 + C7/C8 | 参数项 / 数值能调，OSD 面板在左上角 |

---

## 7. SD 卡约定（改代码不用动卡）

| 项 | 值 |
|---|---|
| 音频区起始 | **绝对 LBA 131072**，共 **720** 扇区 |
| alarm | 471 扇区 @ 131072（16 kHz / 单声道 / 16 bit PCM 小端） |
| ring | 249 扇区 @ 131543 |
| 音频镜像 | `sd_audio\audio_sd.bin`，SHA256 `05A5995EC59CE13ED049E32B1233B7C50D23912E9814FB011326C6EA113C6BE4` |
| BMP | 12 张，**第 12 张 = 应急疏散图**（`SCAN_TARGET_COUNT = 12`） |
| 音频扇区参数 | `audio_sd_stream.v` 的 `ALARM_BASE_SECTOR=131072 / ALARM_SECTORS=471 / RING_BASE_SECTOR=131543 / RING_SECTORS=249` |

> 改音频文件就必须同步改这四个参数和写入脚本，**BMP 在卡上的位置不能动**（音频区在后面，不重叠）。

---

## 8. C 这边的功能点（A 改东西时别弄坏）

| 功能 | 在哪里 | 关键实现 |
|---|---|---|
| 真实录音播放（SD→FIFO→I2S/HDMI） | `audio_sd_stream.v` + `audio_hdmi_i2s.v` | 2 级同步 + 20 ms 去抖 + `cur_sec_safe` 地址护栏 + FIFO 复位 5.12 µs |
| 音源切换（与图片同步） | `audio_sd_stream.v` | 图片看 `sw_emergency`，音频看 `~sw_emergency`（**板上实测两者极性相反**，别"统一"） |
| 音量 | `top_audio_sd.v` `vol_scale` | `q = p`（volume 255 = 满量程），`boost_sat` 提供 4x/2x/1.5x |
| 频谱 | `audio_viz.v` | 9 级一阶 IIR 滤波器组 → `4*log2` 量化 → 攻击瞬时 / 释放 300 ms 线性 |
| 数码管第 4/5 位 | `top_audio_sd.v` → `seg_data_3/4` | 音频电平 0~99（对数刻度） |
| LED3 | `led_audio_act` | 有音频样点就亮 |

---

## 9. 自查包完整性

```powershell
powershell -ExecutionPolicy Bypass -File .\verify_package.ps1
```

应输出 `BIT OK` / `SRC OK 55 files` / `SD AUDIO OK` / `PACKAGE INTACT.`

---

## 10. 改坏了怎么回去

照 §1 的 `bit\TaskC_v7f.bit` 重新烧就行；
源码回滚用 `SRC_MANIFEST.sha256` 一个文件一个文件比对，或者直接跟 C 要这一包的 `src\`。
