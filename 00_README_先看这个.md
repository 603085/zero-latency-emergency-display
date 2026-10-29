# Task A + B + C 整合工程 · v7f（2026-10-29）

> **这是一个工程，不是三个。** A / B / C 三部分的代码全部在同一个 `top` 里，
> 一起综合、一起布线、一起出一个码流。所以源码不能拆开单独烧。
>
> 这一版是 C（音频方向）在板子上确认过的完整状态：
> **图片轮播 + 手动切图 + 应急模式 + 真实录音播放 + 8 段频谱 + 参数 OSD + 字幕** 全部正常。

---

## 1. 东西在哪

> **所有目录名和脚本名都用英文**（TD / PowerShell 对中文路径不稳），文档内容还是中文。

| 目录 | 内容 | 谁看 |
|---|---|---|
| `01_project\` | **全部源码（55 个 .v）+ 码流 + TD 脚本 + 时序报告** | 所有人，尤其 A |
| `02_sd_images\` | 12 张 BMP（第 12 张 = 应急疏散图）+ 用法说明 | 要重做 SD 卡的人 |
| `03_sd_audio\` | 音频区镜像 `audio_sd.bin` + 写卡工具 + 扇区约定 | 要重做 SD 卡的人 |
| `04_docs\` | 接口契约与分工表、板级资源、A 包说明、功能框图 | 所有人 |
| `05_verify\` | `verify_package.ps1`，检查这个包有没有被改动 | 任何人 |

**`01_project\README_改稿须知.md` 是给要改代码的人看的，先读它。**
（谁的地盘、八条"一动就出事"的坑、改完怎么自查。）

`01_project\` 里的结构：

```
src\                 55 个源文件（含 pin_audio_sd.adc / timing.sdc）
user_source\         hdl_source\include\global_def.v（sdram.v 的 include，不能删）
impl.tcl / syn.tcl   完整流程 / 只综合
bit\                 TaskC_v7f.bit + .sha256
report\              route.qor（关键）、完整时序报告、资源报告
SRC_MANIFEST.sha256  每个源文件的哈希
README_改稿须知.md
```

---

## 2. 直接烧（不改代码）

```
01_project\bit\TaskC_v7f.bit
SHA256  EA78381121BCDB397F7A314E67D911BA47860979370F1EB79603A552A95372F6
```

- SD 卡必须是约定状态（见 §4），否则只有画面、没有声音
- 烧完**完全断电再上电一次**（音量参数要复位）
- 按键：KEY1 复位 / KEY2 手动下一张 / KEY3 轮播开关 / KEY4 参数项
- 开关：SW3 拨上 = 紧急，拨下 = 正常；SW4 = 频谱开关；C7 = 参数方向；C8 = 参数加减

---

## 3. 自己编译

**先把整个文件夹解压到纯英文路径**（例如 `D:\work\TaskABC_v7f`）——TD 对中文路径不稳。

```powershell
cd D:\work\TaskABC_v7f\01_project
D:\TD_6.2.1\bin\td_commands_prompt.exe impl.tcl
```

要点：

- **只能用 `td_commands_prompt.exe`**；`td.exe -batch` 只开 GUI 不执行
- tcl 必须 ASCII / 无 BOM；命令行里脚本路径**用正斜杠**
- 重跑前删掉 `v7_impl.al` 和 `v7_impl\`，否则 `PRJ-8646`
- `.bit` 的 SHA 每次都会变（bitgen 不是逐字节可复现）；
  **判断是不是同一个设计，看 `report\route.qor` 的 6 个数字**
- `user_source\hdl_source\include\global_def.v` 是 `sdram.v` 的 include，不能删

本版基线（`report\route.qor`）：

```
Setup WNS  80      Setup TNS  0     violated S-EP  0
Hold  WNS   3      Hold  TNS  0     violated H-EP  0
#slices 7584   #RAMs 40   #DSPs 18   LUT 约 13.3k / 19600
sd_card_clk +0.104ns  video_clk +0.349ns  ext_mem_clk -0.130ns(2 端点，A 的老路径，基线就是 -0.132)
```

---

## 4. SD 卡约定（改代码不用动卡）

**卡上东西必须成套，不要单独删/单独加**，原因见 `02_sd_images\_怎么用.txt`
（`bmp_read` 读的是**原始扇区**，不认文件系统）。

| 项 | 值 |
|---|---|
| BMP | 12 张，按文件名 01→12 整套拷到卡根目录；**第 12 张必须是应急图** |
| 音频区 | 起始**绝对 LBA 131072**，共 720 扇区 |
| alarm | 471 扇区 @ 131072（16 kHz / 单声道 / 16 bit PCM 小端） |
| ring | 249 扇区 @ 131543 |
| 音频镜像 | `03_sd_audio\audio_sd.bin`，SHA256 `05A5995E…` |

> 加减图片后**必须同步改** `src\top_audio_sd.v` 里的
> `.SCAN_TARGET_COUNT (4'd12)` = 卡上图片张数。
> 写音频区用 `03_sd_audio\write_audio.ps1`（**要管理员权限**，写的是卷句柄，偏移从分区起点算）。

---

## 5. 谁改什么（摘要，详细见 `01_project\README_改稿须知.md`）

| 人 | 文件 |
|---|---|
| **A** | `bmp_read / scaler_bilinear / frame_read_write / frame_fifo_* / video_layers_top / mode_fsm / layer_mixer / osd_bar / text_layer / font_rom / color_bar / scroll_timer / latency_meter / video_* / sdram / sdr_*` |
| **B** | `param_regs.v`（含 C 为音量新增的 `output volume` 端口，别删） |
| **C** | `top_audio_sd / audio_viz / audio_sd_stream / audio_hdmi_i2s / audio_arc_calculate / I2S_receiver / sd_card_bmp`（后者是图片通路 + SD 总线音图分时复用，半 A 半 C） |
| **别动** | `pin_audio_sd.adc`（引脚）、`timing.sdc`（时序）、`seg_scan / seg_decoder`（数码管分配） |

---

## 6. 自查

```powershell
powershell -ExecutionPolicy Bypass -File .\05_verify\verify_package.ps1
```

应输出 `BIT OK` / `SD AUDIO OK` / `SRC OK 55 files` / `BMP OK 12` / `PACKAGE INTACT.`

---

## 7. 这一版的来龙去脉（怕出问题翻这个）

C 这边从"能响"到"能交"一共改了 6 轮，每轮的根因和修法都在
`01_project\README_改稿须知.md` 的"八条坑"里。关键三条：

1. **音画极性是相反的**：图片看 `sw_emergency`，音频看 `~sw_emergency`。
   板上实测，别"统一"成一个信号，否则总有一边是反的。
2. **音量掉 42 dB 的根因**：16 位样点放进 24 位端口时用的是符号扩展（值不变），
   再叠一个 `>>7`。修法是 `vol_scale` 里 `q = p`（不右移）。
3. **切 SW3 偶发卡死**：音频借 SD 总线时，2.6 ms 超时会在**事务中途**把 owner 撤掉，
   `bmp_read` 白吃一个扇区 → 字节流错位 512 → 装载卡死。
   修法是"只能在 `sd_sec_read_end` 归还，且超时放宽到 671 ms"。
