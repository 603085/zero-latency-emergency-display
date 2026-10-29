# Task B 骨架说明与开发指南

**给**：B（画质与参数方向）
**前提**：A 已完成，接口已冻结。B 的骨架件已给，**仿真 47 项全过**。
**配套**：`板级接口与显示资源.md`（引脚和参数交互设计）、`接口清单_TaskA.md`（A 的接口）

---

# 第〇部分：这份文档怎么用

按顺序读：

| 章节 | 内容 |
|---|---|
| 第一部分 | 你的任务清单（4 个交付物，按优先级） |
| 第二部分 | 骨架怎么跑（**先跑通再改**） |
| 第三部分 | `param_regs.v` 已实现什么、接口是什么 |
| 第四部分 | 第 2 个任务：写 `osd_bar.v` |
| 第五部分 | 第 3 个任务：写 `scaler.v` |
| 第六部分 | 第 4 个任务：改 `bmp_read.v`（**最后做，风险最高**） |
| 第七部分 | ⚠️ 踩过的 6 个坑（**必读，能省你一天**） |

---

# 第一部分：你的任务清单

## 交付物（对照 `接口契约与分工表.md`）

| # | 交付物 | 对应 | 状态 |
|---|---|---|---|
| 1 | `param_regs.v` 参数寄存器组 | 骨架件 | ✅ **骨架已给，47 项仿真全过** |
| 2 | `osd_bar.v` 参数 OSD 条 | 扩展项④ | ⬜ 你来写 |
| 3 | `scaler.v` 缩放引擎 | 扩展项③ | ⬜ 你来写 |
| 4 | `SD/bmp_read.v` 改造 | 扩展项③ | ⬜ 最后做 |

## 验收标准（赛题要求）

| 扩展项 | 验收标准 |
|---|---|
| **④ 实时调节与 OSD 信息叠加** | 按 `key3` 进入参数调节，画面**弹出 OSD 条**显示当前参数值，**2 秒后淡出**；拨码调节亮度可见变化 |
| **③ 图片缩放适配** | 放入非 640×480 的合法 BMP 也能正确缩放显示，**无错行、无撕裂、无变形** |

## 建议节奏

```
第 1 周：跑通 param_regs 仿真 → 写 osd_bar.v + testbench
第 2 周：把 param_regs + osd_bar 接进 top 联调 → 扩展项④ 可演示
第 3 周：scaler.v（扩展项③）
第 4 周：bmp_read.v 改造（时间够再做）
```

**⚠️ 如果时间紧，优先级是：④ > ③**。因为 ④ 是赛题明确列出的扩展项，而 ③ 可以降级
（见第五部分末尾的"好消息"）。

---

# 第二部分：骨架怎么跑

## 一键脚本

**双击**：
```
C:\Users\jwen\Desktop\anlu\rtl\build_task_b_modelsim.bat
```

**期望输出**：
```
  tb_param_regs RESULT
    checks passed : 47
    checks failed : 0
    RESULT: ALL CHECKS PASSED
    ...
```

我实测过，就是这个结果。

## 手动跑（想改参数时用）

```cmd
cd /d C:\Users\jwen\Desktop\anlu\rtl
vlib work_b
vlog -work work_b -timescale 1ns/1ps param_regs.v
vlog -work work_b -timescale 1ns/1ps tb_param_regs.v
vsim -c -do "run -all; quit -f" work_b.tb_param_regs
```

## 47 项检查覆盖了什么

| 部分 | 检查数 | 内容 |
|---|---|---|
| PART 1 | 8 | 复位默认值（亮度128/对比度128/淡入淡出/3000ms/预设0） |
| PART 2 | 5 | `key3` 循环切换 5 个参数项 → **A 已把项数扩到 6（含音量），这条断言需要按 6 项更新后重跑** |
| PART 3 | 12 | 各参数的增减步进、互不干扰、双向钳位 |
| PART 4 | 5 | 饱和不回绕（亮度 0/255、dwell 200/10000） |
| PART 5 | 6 | OSD 弹出/保持/淡出/重新弹出 |
| PART 6 | 1 | activity 脉冲 |
| 其他 | 10 | 边界与交叉验证 |

---

# 第三部分：`param_regs.v` 已实现什么

## 接口

```verilog
module param_regs #(
    parameter CLK_HZ        = 25_000_000,
    parameter SYNC_STAGES   = 2,
    parameter KEY_DEBOUNCE  = 18,            // 2^18/25MHz = 10.5 ms
    parameter RATE_FAST     = 23'd4_000_000, // 160 ms 首次步进
    parameter RATE_SLOW     = 23'd1_000_000, //  40 ms 重复步进
    parameter OSD_HOLD      = 23'd50_000_000,// 2 s OSD 保持
    parameter OSD_FADE      = 23'd6_250_000, // 250 ms 淡出
    parameter DWELL_MIN     = 16'd200,
    parameter DWELL_MAX     = 16'd10000,
    parameter DWELL_STEP    = 16'd250,
    parameter DWELL_DEFAULT = 16'd3000
)(
    input  wire        clk,              // video_clk 25MHz
    input  wire        rst,              // 高有效

    // 物理输入
    // ★ 2026-10-14 本工程（A 侧）改了交互，签名随之变动：
    //     key_next_param：短按 = 按方向改数值；长按 0.6 s = 换参数项
    //     sw_up         ：按住 = 按【当前方向】连续步进（原"参数 +"）
    //     dir_up        ：方向选择，1 = +，0 = −（原端口名 sw_dn / "参数 -"）
    input  wire        key_next_param,   // C1 KEY4（低有效）
    input  wire        sw_up,            // C8：按住连续步进
    input  wire        dir_up,           // C7：方向选择

    // 参数输出 → 给 A 的图层和 C 的音频
    output reg  [7:0]  brightness,       // 128 = 中性
    output reg  [7:0]  contrast,         // 128 = 中性
    output reg  [1:0]  transition,       // 0 = 硬切, 1 = 淡入淡出
    output reg  [15:0] dwell_ms,         // 200~10000
    output reg  [1:0]  preset,           // 0~3
    output reg         preset_en,        // 用户是否选过预设

    // ★ 本工程新增的第 6 项参数（音量）：不在最初冻结的签名里。
    //   位宽/默认值见下表；B 的原件如果不带这一项，顶层例化会报缺端口。
    output reg  [7:0]  volume,           // 128 = 原音量, 0 = 静音

    // OSD 控制 → 给你的 osd_bar
    output reg  [2:0]  cur_param,
    output reg         osd_visible,
    output reg  [7:0]  osd_alpha,
    output reg  [7:0]  osd_value,

    // 状态
    output reg         activity,         // 单周期脉冲
    output reg  [15:0] step_count        // 累计步进次数（调试用）
);
```

## 交互模型（★ 2026-10-14 最终版；历次改版见下方注释）

```
短按 key4 (C1)     → 【切换】cur_param 前进一项（0→1→…→5→0），并弹出 OSD
长按 key4 (0.6 s)  → 【只弹出 OSD】显示当前项；不切换、不改数值
按住 C8 (sw_up)    → 【只有它才改数值】按当前方向步进
                     （160 ms 第一步、之后每 40 ms 一步）
拨 C7 (dir_up)     → 只选方向：+ / −
无操作 2 s         → OSD 自己消失（OSD_HOLD = 2 s，然后 250 ms 淡出）
```

> ⚠️ 两条容易搞错的：
> 1. **KEY4 不改数值**（2026-10-14 第一版是"短按改值/长按换项"，用户要求改回来）；
> 2. **长按只负责把 OSD 显示出来**（"我之前在调哪一项"一眼可见），不换项、不改值；
>    长按后松手也不会再切一项。
>
> 历次模型：① 2026-10-12 前 `按 key3 切项 + 拨 sw_up/sw_dn 各自加减`；
> ② 2026-10-14 早 `短按改值 / 长按换项 / C7 方向 / C8 连续`；
> ③ **最终版 = 上面这张表**。端口名 `sw_dn` → `dir_up` 自 ② 起生效。
> **任何一次改版都会让仿真基线失效，重跑前必须同步。**

> ⚠️ **项数变更（A 加的，2026-10-12）**：原来 5 项（0~4），现在 **6 项（0~5）**，
> 第 5 项是**音量**。`PARAM_LAST` 从 `3'd4` 改成 `3'd5`。
> **这会让原来那套 47 项仿真里的 PART 2/PART 3 断言失效**（按 5 次不回绕了），
> 重跑 `tb_param_regs` 前要把循环期望改成 6 项，并补音量项的默认值/步进/饱和检查。
> 变更前请勿再对外声明"47 项全过"。

| `cur_param` | 参数 | 量程 | 步进 |
|---|---|---|---|
| 0 | 亮度 | 0~255 | 8 |
| 1 | 对比度 | 0~255 | 8 |
| 2 | 转场类型 | 0/1 | 双向都切换 |
| 3 | 停留时间 | 200~10000 | 250 |
| 4 | 配色预设 | 0~3 | 循环 |
| **5** | **音量**（A 新增） | **0~255** | **8**（上电默认 **0 = 静音**；128 = 原音量，255 ≈ ×2） |

**所有参数都是饱和钳位，不回绕。**

## ★ 2026-10-14 A 侧交互改版（本工程已改，B 的原件需同步）

原来的模型是："KEY4(C1) 切参数项 + C8/C7 两个拨码各自加减"。
本工程按用户要求改成了 **"一个键 + 一个方向拨码"**：

| 操作 | 行为 |
|---|---|
| **KEY4(C1) 短按**（< 0.6 s 松手） | **切换**：`cur_param` 前进一项（0→1→…→5→0） |
| **KEY4(C1) 长按**（≥ 0.6 s） | **只弹出 OSD**（显示当前项）；不换项、不改数值 |
| 长按后松手 | **不再**执行短按动作（不会"弹完 OSD 又切走一项"） |
| **C7(SW2)** `dir_up` | **方向选择**：拨上 = +，拨下 = −（原为"参数 −"） |
| **C8(SW1)** `sw_up` | **只有它才改数值**：按住 = 按当前方向连续步进（原为"参数 +"） |

* 端口名：`sw_dn` → **`dir_up`**（顶层端口 `sw_param_dn` → `sw_dir`），引脚仍是 C7；
* 时间：消抖 10.5 ms、长按阈值 0.6 s；C8 连续步进仍是 160 ms/40 ms（未改）；
* **KEY4 不改数值**（第二次改版的核心变化）；
* "武装"门只剩 C8（复位后 ≈84 ms 且 C8 在 0 位才允许连续步进）——
  C7 是**方向电平**，停在 1 位只表示方向为 +，不会自己扫参数，所以不参与武装判断。

## 长按重复的精确行为（C8，未改）

拨码是**开关**不是按键，所以做了长按重复。行为已用脚本建模验证：

```
第一次步进：按住 RATE_FAST + 1 个时钟后  （160 ms）
后续步进  ：每 RATE_SLOW + 1 个时钟一次    （40 ms）
松开后重新计时
```

**`+1` 是装载递减计数器的固有延迟，不是 bug。** 我为此专门写了
`sim_param_timing.py` 建模，避免你重踩。

---

# 第四部分：第 2 个任务 —— 写 `osd_bar.v`

## 接口（已冻结）

```verilog
module osd_bar (
    input  wire        clk,          // video_clk 25MHz
    input  wire        rst,
    input  wire [9:0]  x, y,         // 像素坐标（和 de_r 同步）
    input  wire        de,           // 数据有效

    // 来自 param_regs
    input  wire        osd_visible,
    input  wire [7:0]  osd_alpha,
    input  wire [2:0]  cur_param,
    input  wire [7:0]  osd_value,
    input  wire [7:0]  brightness, contrast,
    input  wire [1:0]  transition,
    input  wire [15:0] dwell_ms,
    input  wire [1:0]  preset,

    // 输出给 layer_mixer
    output reg  [23:0] osd_rgb,
    output reg  [7:0]  osd_a
);
```

## 要显示什么

```
┌──────────────────────┐
│ BRIGHT      136      │   ← 参数名 + 当前值
│ ████████░░░░░░░░     │   ← 进度条
└──────────────────────┘
```

| `cur_param` | 参数名 | 值显示 |
|---|---|---|
| 0 | `BRIGHT` | 十进制 |
| 1 | `CONTRA` | 十进制 |
| 2 | `TRANS` | `FADE` / `CUT` |
| 3 | `DWELL` | 十进制 |
| 4 | `PRESET` | `STD`/`EMG`/`NIGHT`/`HI` |
| **5** | **`VOLUME`**（A 新增） | **十进制**（128 = 原音量，0 = 静音） |

## 屏幕位置（已约定，别和 A、C 打架）

```
x:  20 ~ 276     (宽 256)
y:  16 ~  80     (高 64)
```

| 谁 | 区域 |
|---|---|
| **B 的 OSD** | **y = 16 ~ 80** |
| C 的频谱 | y = 360 ~ 408 |
| A 的字幕 | y = 424 / 448 |

## ⭐ 直接复用 A 的 `font_rom.v`

**不要自己写字模。** 例化 A 的模块：

```verilog
font_rom u_font (
    .ch       (char_code[6:0]),   // ASCII
    .row      (row_in_glyph),     // 0~15
    .row_bits (pixel_bits)        // bit[7] 是最左像素
);
```

**支持字符**：`0-9` `A-Z` `空格` `:` `-` `.` `/` `>`（右箭头）

A 已经用 624 个测试点验证过它了（39 字符 × 16 行全部比对），**可以放心用**。

## ⚠️ 一个关键设计点：为什么要单独一层

A 已经把 `layer_mixer` 留好了**第 4 个图层**给你：

```
图片 → 转场 → 字幕(A) → 频谱(C) → 亮度对比度增强 → OSD(B) → 输出
                                                      ↑
                                            注意：在增强之后！
```

**原因**：你的 OSD 显示的就是亮度/对比度参数。如果放在增强之前，
用户把亮度调到 20 时，**正好在最需要看清的时候 OSD 变暗了**。

A 已经用仿真验证了这一点：
```
[PASS] mixer: OSD stays full colour at brightness=0 contrast=0
```

**所以你的 `osd_rgb` 输出的是"最终颜色"，不需要考虑亮度。**

---

# 第五部分：第 3 个任务 —— 写 `scaler.v`

## 要解决什么

例程只接受 **640×480**。`SD/bmp_read.v` 里有写死的校验：

```verilog
assign header_match = (header_0 == "B") && (header_1 == "M") &&
                      (width[15:0]  == bmp_width) &&    // 写死 640
                      (height[15:0] == bmp_height) &&   // 写死 480
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);
```

## 两处要改

| 改哪里 | 怎么改 |
|---|---|
| `SD/bmp_read.v` | 从文件头读出**实际宽高**，不再和 640×480 比对 |
| 新建 `scaler.v` | 把任意尺寸缩放到 640×480 |

## 缩放算法：用双线性，不要最近邻

**原因**：疏散图上有**细箭头和小字**，最近邻缩放会让它们糊掉或断裂。
**在应急场景里，糊掉的小字等于没有。**

实现建议：
- **水平方向**：行缓冲（line buffer）里做 4 抽头插值
- **垂直方向**：需要**两行缓冲**，对相邻两行插值

## ⭐ 好消息：这个任务可以降级

我查过 `doc/convert/convert_images_to_bmp.py`，**它已经在 PC 端用 LANCZOS 把图缩放到 640×480 了**。

**所以主链路根本不需要运行时缩放。**

`scaler.v` 的定位是**加分项**："让系统接受任意尺寸素材，不用 PC 端预处理"。

**如果第 3 周来不及**，直接跳过，先保住扩展项 ①②④⑤ 四项。答辩时可以说：
> "我们支持任意尺寸素材，但推荐离线预处理以获得最佳质量。"

---

# 第六部分：第 4 个任务 —— 改 `bmp_read.v`（**最后做**）

改了 `header_match` 之后，**下游全部受影响**：

| 受影响的地方 | 原来 | 要改成 |
|---|---|---|
| `frame_read_write` 的 `FRAME_WIDTH/HEIGHT` | 640/480 | 动态 |
| `read_len` / `write_len` | 307200（写死） | 按实际尺寸算 |
| SDRAM 缓冲地址 `BUF0_ADDR`/`BUF1_ADDR` | 写死 | 按实际尺寸算 |

**⚠️ 风险最高的一步。**

**做法**：
1. **先备份能跑的版本**（整个工程目录拷一份）
2. 小步改，每改一处跑一次仿真
3. 改完上板验证"640×480 的图还能正常显示"（**回归测试**）

**如果时间不够，这一项可以不做** —— 例程原本的 640×480 也满足赛题基础要求。

---

# 第七部分：⚠️ 踩过的 6 个坑（必读）

这些是我在写 `param_regs.v` 时**实际踩到的**，你大概率也会遇到。

## 坑 1：Verilog 的 `~` 是位取反，不是逻辑取反

```verilog
// 错：transition 是 2 位，~2'd1 = 2'b10 = 2，是无效值！
transition <= ~transition;

// 对
transition <= (transition == 2'd0) ? 2'd1 : 2'd0;
```

**症状**：参数变成非法值，后续判断全乱。**这个 bug 是 testbench 抓出来的。**

## 坑 2：同一个 always 块里"先清零后置位"会失效

```verilog
// 错：up_step <= 1'b0 在块首，后面的 <= 1'b1 要看执行路径，容易失效
always @(posedge clk) begin
    up_step <= 1'b0;
    ...
    up_step <= 1'b1;
end
```

**修法**：把脉冲产生和计时拆成不同块，或者用"装载递减计数器"模式（我最终用的）。

## 坑 3：用等值比较做重复触发，命中一次就再也不命中

```verilog
// 错：计数器跳到 RATE_FAST 后，再也回不到 RATE_FAST-1
else if (up_timer == RATE_FAST - 1) begin
    up_timer <= RATE_FAST;
    up_step  <= 1'b1;
end
```

**修法**：用递减到 0 再重装载。

## 坑 4：testbench 的按键时间只比消抖窗长一点点

**症状**：**只有第一次按键生效，之后全失效。**

**原因**：消抖器需要在**两个状态**都有完整的安静窗口。按下的时间刚过窗口，
电平检测器还在计数中，下一次按键就被吞掉了。

**规则**：**按住时间 ≥ 4 倍消抖窗，两次按键间隔 ≥ 2 倍消抖窗。**

## 坑 5：层次引用会被优化掉

```verilog
// 可能读到 x，因为信号被综合/优化掉了
$display("%0d", u_dut.up_step);

// 可靠做法：在 DUT 里加正式输出端口
output reg [15:0] step_count;
```

**我在 `param_regs.v` 里加了 `step_count` 输出端口**，就是为了这个。

## 坑 6：PowerShell 改文本文件会加 BOM

```powershell
# 危险：会写 UTF-8 BOM，ModelSim 报 non-printable character 0xef
Set-Content file.v -Encoding UTF8
```

**症状**：ModelSim 报 `unexpected non-printable character 0xef`。

**规则**：**用编辑器的"无 BOM UTF-8"保存，不要用 PowerShell 改 Verilog 文件。**

---

# 第八部分：接进 `top` 的接线

## 需要加到 `top_tf_hdmi_audio.v` 的信号

```verilog
// ===================== Task B : 参数寄存器 =====================
wire [7:0]  param_brightness, param_contrast;
wire [1:0]  param_transition, param_preset;
wire        param_preset_en;
wire [15:0] param_dwell_ms;
wire [7:0]  param_volume;            // ★ A 新增：音量，送到音频缩放
wire [2:0]  b_cur_param;
wire        osd_visible;
wire [7:0]  osd_alpha, osd_value;
wire        param_activity;
wire [15:0] step_count;

param_regs u_param_regs (
    .clk            (video_clk),
    .rst            (rst_all),
    .key_next_param (key4),          // ★ 注意：是 key4(C1)，不是 key3(B1 轮播)
    .sw_up          (sw_param_up),   // C8：按住 = 按当前方向连续步进
    .dir_up         (sw_dir),        // C7：方向选择（★ 2026-10-14 由 sw_dn 改名）
    .brightness     (param_brightness),
    .contrast       (param_contrast),
    .transition     (param_transition),
    .dwell_ms       (param_dwell_ms),
    .preset         (param_preset),
    .preset_en      (param_preset_en),
    .volume         (param_volume),  // ★ A 新增
    .cur_param      (b_cur_param),
    .osd_visible    (osd_visible),
    .osd_alpha      (osd_alpha),
    .osd_value      (osd_value),
    .activity       (param_activity),
    .step_count     (step_count)
);
```

> 上面这段是**本工程实际接法**（[top_tf_hdmi_audio.v](src/user_source/hdl_source/top_tf_hdmi_audio.v)）。
> 与最初文档的两处差别：`key_next_param` 接 **key4(C1)**（本工程 `key3` 是 B1 轮播键），
> 以及多一个 `.volume`。

## 接到 A 的包装模块

`video_layers_top` 已经有对应的输入端口了：

```verilog
u_video_layers (
    ...
    .param_brightness (param_brightness),   // ← 接这里
    .param_contrast   (param_contrast),
    .param_transition (param_transition),
    .param_preset     (param_preset),
    .param_preset_en  (param_preset_en),
    .osd_rgb          (osd_rgb),            // ← 你的 osd_bar 输出
    .osd_alpha        (osd_a),
    ...
);
```

## 引脚约束（追加到 `pin.adc`）

```tcl
# 参数调节拨码（B）—— ★ 2026-10-14 改版：C7 从"参数 -"变成"方向选择"
set_pin_assignment { sw_param_up } { LOCATION = C8; IOSTANDARD = LVCMOS33; PULLTYPE = PULLUP; }
set_pin_assignment { sw_dir }      { LOCATION = C7; IOSTANDARD = LVCMOS33; PULLTYPE = PULLUP; }
# 参数激活指示 LED（B）
set_pin_assignment { led_param_act } { LOCATION = A3; IOSTANDARD = LVCMOS33; DRIVESTRENGTH = 8; }
# key3 的约束在 A 那边已经加了（C1）
```

---

# 第九部分：要遵守的规则（来自接口契约）

| 规则 | 说明 |
|---|---|
| **`param_regs.v` 是全系统唯一的参数源** | 别人**只读**，不得在别处再声明一份 |
| **`top_tf_hdmi_audio.v` 由 A 统一改** | 你要接线，告诉 A 或提交补丁给他 |
| **不要动 A 的模块** | `layer_mixer.v` / `text_layer.v` / `font_rom.v` 只读 |
| **扩展项 ③ 可以降级** | 时间不够就跳过，先保 ④ |
| **上板前先跑仿真** | 47 项全过再上板 |

---

# 第十部分：一句话总结

> **`param_regs.v` 骨架已经能跑，47 项仿真全过。**
> **你的三个新任务**：写 `osd_bar.v`（扩展项④）→ 写 `scaler.v`（扩展项③）→ 改 `bmp_read.v`。
> **先双击 `build_task_b_modelsim.bat` 确认骨架正常，再动代码。**
> **第七部分的 6 个坑必读** —— 那是我实际踩过的，能省你一天。
