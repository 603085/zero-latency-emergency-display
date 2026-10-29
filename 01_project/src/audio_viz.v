//=============================================================================
//  audio_viz.v  ——  C（音频方向）骨架件：音频可视化 + 电平指示
//=============================================================================
//  项目 : 基于安路 EG4S20 的零延迟应急信息发布与导视系统
//  赛题 : 2026 全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道
//         安路赛道 · 选题一
//  板卡 : HX4S20C (EG4S20BG256)     工具链 : 安路 TD 6.2.1
//  归属 : C 独占（接口契约与分工表 · 第六节 文件归属矩阵）
//
//-----------------------------------------------------------------------------
//  1. 它做什么
//-----------------------------------------------------------------------------
//  * 8 段频谱  : 从 I2S 收到的 PCM 里实时提取 8 个倍频程能量，
//                以"频谱柱图层"输出给 layer_mixer（spec_rgb / spec_alpha）。
//  * 音频电平  : 0~99 峰值电平，供数码管低 4 位显示（level_disp / level_peak）。
//  * 音频指示  : led_audio_act（板上 LED3 = C10），亮度随电平变化。
//
//  全硬件、无处理器、无 FFT —— 与创新点 1"按键→像素的单帧确定性延迟"同构：
//  每个像素时钟都在推进，延迟由时序决定，不随时机波动。
//
//-----------------------------------------------------------------------------
//  2. 频谱算法：逐级差分滤波器组（重点，答辩要讲）
//-----------------------------------------------------------------------------
//        x --> [p0] --> [p1] --> [p2] --> ... --> [p8]
//               |         |         |              |
//              b0=p0-p1  b1=p1-p2  b2=p2-p3  ...  b7=p7-p8
//
//  * 9 个一阶 IIR 低通串联，相邻两级相减 = 一个带通
//  * 每个极点 = 一次乘加 → 8 段频谱只要 9 个乘法器
//  * 极点截止频率按倍频程排列 120Hz·2^k (k=0..8)，每段正好一个八度
//
//  为什么不用 FFT：FFT 要块处理 + 蝶形运算 + 大量存储，LUT/DSP 占用高一个
//  数量级，而且输出是"一批一批"的；本方案逐样本连续输出，与像素流同构。
//
//-----------------------------------------------------------------------------
//  3. 定点格式（与 verification/audio_viz_model.py 逐位一致，模型已 PASS）
//-----------------------------------------------------------------------------
//      acc  <= acc + ((($signed(x) >>> LPF_IN) <<< 8) - acc) * A_k >>> 8   // 极点
//      p    <= acc >>> 8                                                   // 用新 acc
//      bar_k = min(|p_k - p_{k+1}| >>> SCALE_SHIFT[k], 255)
//
//  必须注意的两个坑（开发中都踩过）：
//   * A_k 必须四舍五入取整，不能截断。截断会让 a 系统性偏小，
//     极点带宽整体变窄，频响曲线整个塌掉。
//   * p 要用"更新后的 acc"计算（与参考模型一致），否则频响会偏移。
//
//-----------------------------------------------------------------------------
//  4. 端口
//-----------------------------------------------------------------------------
//  契约（接口契约与分工表 4.3）给的三个输入 audio_valid / audio_left_data /
//  audio_right_data 原样接收，不碰 I2S_receiver.v 与 audio_arc_calculate.v。
//
//  比契约多出的 (de, x, y)：频谱是"画面上的图层"，必须知道当前像素坐标才能
//  决定这个像素是不是柱子。x/y 由 A 的 layer_mixer 内部生成，接过来即可。
//
//  电平输出的两种格式（板级接口与显示资源(1) 2.4.4 节，方案 1+3）：
//   * level_peak [7:0]  0~99 十进制，给 A 用 seg_decoder 转段码
//   * level_disp [15:0] [15:8]=十位段码 [7:0]=个位段码，已译码
//     → A 在 top 里直接 .seg_data_3(level_disp[15:8]), .seg_data_4(level_disp[7:0])
//
//-----------------------------------------------------------------------------
//  5. 位宽分析（为什么 LPF_IN=4、ACC_SHIFT=8）
//-----------------------------------------------------------------------------
//  音频样本有符号 24bit，|x| <= 2^23
//    x'      = x >>> LPF_IN(=4)        → |x'| <= 2^19
//    acc 稳态 = x' << 8                → |acc| <= 2^27   （32bit 有符号安全）
//    中间乘积 = 28bit × 8bit           → 36bit，Verilog 自动扩宽，无溢出
//  若把 LPF_IN 改小，|acc| 会进入 2^28~2^31，有溢出风险 —— 改了要重算并重跑模型。
//
//  预期资源（TD 6.2.1 综合实测见 verification/td_synth_report.txt）：
//    9 个 28×8 乘法器 → EG4S20 有 29 个 DSP，占 31%
//
//-----------------------------------------------------------------------------
//  6. 验证状态
//-----------------------------------------------------------------------------
//   * 定点参考模型 verification/audio_viz_model.py ：总判定 PASS
//       - 单音扫描 18 点，频谱重心单调右移 1.02 → 5.72 段
//       - 8 音阶方波重心单调递增，无削顶，亮度 153~165
//       - 静音归零 / 满量程直流抑制 / 满量程不溢出 / 停止后释放归零  全 PASS
//   * TD 6.2.1 综合：voir verification/ 下综合报告
//=============================================================================

`timescale 1ns / 1ps

module audio_viz #(
    // ---------------- 音频与滤波器组 ----------------
    //  ★ SAMPLE_RATE 只决定**极点系数 A_k**（滤波器组各段的中心频率），
    //    它必须保持 48000 —— 见下面 A_k 的推导，也见 TICK_RATE 的注释。
    parameter integer SAMPLE_RATE  = 48000,
    //  ★ TICK_RATE 决定**电平/柱高回落的时间常数**，应当填**真实**的
    //    audio_valid 脉冲率。
    //
    //    为什么要和 SAMPLE_RATE 分开（这是一个真实踩到的坑）：
    //      基线工程里的 I2S 模块是"每次 I_mclk 翻转 BCLK、64 个 BCLK 一帧"，
    //      而 I_mclk = 12.288 MHz，所以真实的帧率是
    //          12.288 MHz / 128 = 96 kHz
    //      不是 48 kHz。而 audio_valid 就是在帧率上pulse的。
    //
    //      如果两个率都取 48 kHz，电平表的保持时间和柱子回落速度
    //      都会慢一倍（看起来"迟钝"）。分开之后：
    //        * 系数仍按 48 kHz 算 → 各频段仍在人耳/语音关心范围（120Hz~7.7kHz）
    //        * 时间常数按 96 kHz 算 → 电平表反应速度正确
    parameter integer TICK_RATE    = 48000,
    parameter integer LPF_IN       = 4,       // 样本进滤波器组前右移（防 Q8 溢出）
    parameter integer ACC_SHIFT    = 8,       // Q8 累加器 → 极点

    // ---------------- 频谱柱显示区域 ----------------
    //  ★ 默认值和 A 的 layer_mixer 完全对齐（这是必须的，不是随意选的）：
    //    A 的 layer_mixer 用"区域遮罩"裁剪频谱层：
    //        spec_in_area = (x >= SPEC_X) && (x < SPEC_X+SPEC_W)
    //                    && (y >= SPEC_Y) && (y < SPEC_Y+SPEC_H)
    //    而 video_layers_top 传下去的是 SPEC_X=192 SPEC_Y=360 SPEC_W=256 SPEC_H=48
    //    所以只有落在 x∈[192,448)、y∈[360,408) 里的像素才会显示出来。
    //
    //    如果这里的窗口和 A 的不一致会怎样：
    //      * 窗口偏左（比如默认 0）→ 8 根柱子全在 x<192，被遮罩切掉，屏幕上一片空
    //      * 窗口偏右/偏大      → 右边的柱子被切掉一半
    //    换句话说：**改这里就必须同步改 A 的 SPEC_*，或者反之**。
    parameter integer SPEC_X0      = 192,     // 区域左边界（= A 的 SPEC_X）
    parameter integer SPEC_X1      = 447,     // 区域右边界（= SPEC_X + SPEC_W - 1）
    parameter integer SPEC_Y_BASE  = 408,     // 柱底 y（= SPEC_Y + SPEC_H）
    parameter integer SPEC_Y_TOP   = 360,     // 最高柱顶 y（= SPEC_Y）
    // 256 px / 8 根 = 每根占 32px。取 柱宽 28 + 间隔 4，正好铺满 256。
    parameter integer BAR_W        = 28,      // 每根柱宽度（像素）
    parameter integer BAR_GAP      = 4,       // 柱间空隙（像素）

    // ---------------- 电平保持与释放 ----------------
    parameter integer RELEASE_MS   = 300,     // 频谱柱释放时间（毫秒）
    parameter integer LEVEL_MS     = 500,     // 数码管电平刷新周期（毫秒）

    // ---------------- 颜色（24bit RGB） ----------------
    parameter [23:0] COLOR_LOW     = 24'h00FF40,   // 柱底：绿
    parameter [23:0] COLOR_MID     = 24'hFFFF00,   // 中段：黄
    parameter [23:0] COLOR_HIGH    = 24'hFF3000,   // 柱顶：红
    parameter [23:0] COLOR_BG      = 24'h001020    // 柱基底线
)(
    // ---------------- 时钟与复位 ----------------
    input  wire        clk,              // video_clk = 25MHz（与像素流同域）
    input  wire        rst,              // 高有效复位 = rst_all

    // ---------------- 音频数据源（来自 I2S_receiver，只读） ----------------
    input  wire        audio_valid,      // 一个 48kHz 采样有效
    input  wire [23:0] audio_left_data,  // 有符号 24bit
    input  wire [23:0] audio_right_data, // 有符号 24bit（保留接口，本版不用）

    // ---------------- 开关 / 参数 ----------------
    input  wire        sw_audio_en,      // C5  SW4：1=显示频谱，0=关闭
    input  wire        self_test,        // 1=整屏自测（自带扫频激励，独占用屏）

    // ---------------- 像素坐标（由 A 的 layer_mixer 生成后引回） ----------------
    input  wire        de,               // 像素有效
    input  wire [9:0]  x,                // 0~639
    input  wire [9:0]  y,                // 0~479

    // ---------------- 输出给 layer_mixer 的图层（契约 4.1） ----------------
    output reg  [23:0] spec_rgb,
    output reg  [7:0]  spec_alpha,       // 0=透明，255=不透明（二值）

    // ---------------- 输出给板上指示 ----------------
    output wire [7:0]  level_peak,       // 0~99 音频峰值电平（十进制）
    output wire [15:0] level_disp,       // [15:8]=十位段码 [7:0]=个位段码
    output reg         led_audio_act     // C10 LED3：音频有效指示
);

    //=========================================================================
    //  常量与函数
    //=========================================================================
    localparam integer NP = 9;            // 极点数 = 段数 + 1

    // 极点系数 A_k = round(256 * (1 - exp(-2*pi*fc/fs)))，fc = 120Hz * 2^k
    // *** 必须四舍五入取整，不要截断 ***
    function [7:0] pole_coef;
        input integer k;
        begin
            case (k)
                0:       pole_coef = 8'd4;
                1:       pole_coef = 8'd8;
                2:       pole_coef = 8'd16;
                3:       pole_coef = 8'd30;
                4:       pole_coef = 8'd57;
                5:       pole_coef = 8'd101;
                6:       pole_coef = 8'd162;
                7:       pole_coef = 8'd222;
                default: pole_coef = 8'd251;    // k = 8
            endcase
        end
    endfunction

    // 9 个固定系数乘法改成移位加法，避免 TD 把 9 个乘法全部映射到 DSP。
    // 系数值必须和 pole_coef 完全一致：
    //   4, 8, 16, 30, 57, 101, 162, 222, 251
    function signed [39:0] pole_mul;
        input signed [31:0] d;
        input integer k;
        reg signed [39:0] q;
        begin
            case (k)
                0: q = d <<< 2;
                1: q = d <<< 3;
                2: q = d <<< 4;
                3: q = (d <<< 5) - (d <<< 1);                  // 32 - 2 = 30
                4: q = (d <<< 6) - (d <<< 3) + d;              // 64 - 8 + 1 = 57
                5: q = (d <<< 7) - (d <<< 5) + (d <<< 2) + d; // 128 - 32 + 4 + 1 = 101
                6: q = (d <<< 7) + (d <<< 5) + (d <<< 1);      // 128 + 32 + 2 = 162
                7: q = (d <<< 8) - (d <<< 5) - (d <<< 1);      // 256 - 32 - 2 = 222
                default: q = (d <<< 8) - (d <<< 2) - d;        // 256 - 4 - 1 = 251
            endcase
            pole_mul = q;
        end
    endfunction

    // 每段显示定标右移
    //   [C 2026-10-23] 原来 k0..k6 全用 8、k7 用 6。用真实 PCM 逐位复刻
    //   滤波器（tools/viz_model.py）后测得每段 |pole_k - pole_k+1| 的
    //   p90 从低到高是 3.6e4、4.0e4、3.1e4、2.3e4、1.6e4、1.2e4、6.8e3、2.3e3，
    //   也就是高频段本来就只有低频段的 1/15。统一用 8 的结果：
    //       k0..k2 柱高 ~120-160（看得见）
    //       k3..k5 柱高 ~45-90  （偏矮）
    //       k6,k7  柱高 ~8-35   （几乎贴底，板子上就是"频谱不动"）
    //   改成按段递减的定标，让 8 根柱子的 p90 都落在 100~220。
    function [4:0] scale_shift;
        input integer k;
        begin
            case (k)
                0,1,2:   scale_shift = 5'd7;   // display gain +6 dB
                3,4:     scale_shift = 5'd6;   // display gain +6 dB
                5,6:     scale_shift = 5'd5;   // display gain +6 dB
                default: scale_shift = 5'd3;   // k=7，display gain +6 dB
            endcase
        end
    endfunction

    // 柱子水平边界（组合常量，用比较法判定落在第几根柱 → 不用除法器）
    function [9:0] bar_lo;
        input integer k;
        begin bar_lo = SPEC_X0 + k * (BAR_W + BAR_GAP); end
    endfunction

    function [9:0] bar_hi;
        input integer k;
        begin bar_hi = SPEC_X0 + k * (BAR_W + BAR_GAP) + BAR_W - 1; end
    endfunction

    localparam integer DECAY_TICK = (TICK_RATE * RELEASE_MS) / (1000 * 256);
    localparam integer LEVEL_TICK = (TICK_RATE * LEVEL_MS) / 1000;
    localparam integer BAR_FULL_H = SPEC_Y_BASE - SPEC_Y_TOP;

    // 颜色分档阈值（按电平 0~255）
    localparam [7:0] COL_T1 = 8'd96;
    localparam [7:0] COL_T2 = 8'd192;

    // 自测扫频：每 2^13 个像素时钟前进一步（≈3kHz 步进率，听感上是连续扫频）
    localparam integer ST_STEP_DIV = 8192;

    //=========================================================================
    //  1) 音频源：真实 I2S 数据 或 内部自测扫频
    //     真实的 audio_valid 脉冲率是 48 kHz；video_clk 25MHz 时
    //     每个样本约 520 个时钟。TICK_RATE 必须与之匹配。
    //=========================================================================
    wire signed [23:0] sample_now = $signed(audio_left_data);

    reg  [15:0] st_phase;       // 相位累加器（Q16，只取高 8 位做波形）
    reg  [15:0] st_inc;         // 相位增量 = 频率
    reg  [12:0] st_div;
    reg         st_dir;         // 0=升频 1=降频
    reg signed [23:0] st_sample;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st_phase  <= 16'd0;
            st_inc    <= 16'd800;
            st_div    <= 13'd0;
            st_dir    <= 1'b0;
            st_sample <= 24'sd0;
        end else if (self_test) begin
            // 扫频：st_inc 从 800(≈0.6kHz) 到 20000(≈14.6kHz) 来回
            if (st_div >= ST_STEP_DIV[12:0]) begin
                st_div <= 13'd0;
                if (!st_dir) begin
                    if (st_inc >= 16'd20000) begin
                        st_dir <= 1'b1;
                        st_inc <= st_inc - 16'd200;
                    end else
                        st_inc <= st_inc + 16'd200;
                end else begin
                    if (st_inc <= 16'd1000) begin
                        st_dir <= 1'b0;
                        st_inc <= st_inc + 16'd200;
                    end else
                        st_inc <= st_inc - 16'd200;
                end
            end else
                st_div <= st_div + 13'd1;

            st_phase  <= st_phase + st_inc;
            // 正弦近似：用相位高 8 位做地址的三角波，幅度 1/2 满量程
            // （自测只验证通路，不要求低失真）
            st_sample <= st_phase[15] ? (24'sd0 - {st_phase[14:8], 17'd0})
                                      : {st_phase[14:8], 17'd0};
        end
    end

    // 真实音频：audio_valid 每 48kHz 一拍；自测模式每 64 个时钟自己产生一次"有效"
    reg [5:0] st_valid_div;
    always @(posedge clk or posedge rst) begin
        if (rst) st_valid_div <= 6'd0;
        else if (self_test) st_valid_div <= st_valid_div + 6'd1;
    end
    wire st_valid = self_test && (st_valid_div == 6'd0);

    wire               samp_en   = self_test ? st_valid : audio_valid;
    wire signed [23:0] samp_data = self_test ? st_sample : sample_now;

    //=========================================================================
    //  2) 滤波器组：9 个一阶 IIR 低通串联
    //=========================================================================
    reg signed [31:0] acc [0:NP-1];       // Q8 累加器
    reg signed [23:0] pole[0:NP-1];       // 极点输出 = acc >>> 8

    wire signed [23:0] x_in = samp_data >>> LPF_IN;

    // 组合算出每个极点的新 acc（用旧 acc），再一次性打拍
    reg signed [31:0] acc_nxt [0:NP-1];
    integer i;
    always @(*) begin
        for (i = 0; i < NP; i = i + 1)
            acc_nxt[i] = acc[i] + (pole_mul($signed((x_in <<< 8) - acc[i]), i) >>> 8);
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < NP; i = i + 1) begin
                acc [i] <= 32'sd256;      // 小正偏置，避免上电瞬间差分恒为 0
                pole[i] <= 24'sd1;
            end
        end else if (samp_en) begin
            for (i = 0; i < NP; i = i + 1) begin
                acc [i] <= acc_nxt[i];
                pole[i] <= acc_nxt[i] >>> ACC_SHIFT;   // 用新 acc，与参考模型一致
            end
        end
    end

    //=========================================================================
    //  3) 段能量 + 攻击/释放包络
    //=========================================================================
    reg [7:0] band_lvl [0:7];
    reg [7:0] decay_cnt;

    // ======================================================================
    //  [FIX v7c] ?? -> ??,?????(dB)??,?? Music-Spectrum ??
    // ----------------------------------------------------------------------
    //  ??????????(??? PCM ??????):
    //    1) (pole_k - pole_k+1) >>> scale_shift ???? 8bit reg = ??,
    //       ??? |pole_k - pole_k+1| ?? 0~650(? 10bit)?>>7 ????? 0,
    //       ????????????????????????
    //    2) ?????????? x49/256(= 48 px),96 px ?????????
    //  ?? p50/p90(?? audio_sd.bin):
    //      ??? log2(mag) ? p50 ? 3.4 oct?p90 ? 6.6 oct???? 8.5 oct
    //  ??????????????:q4 = 4*log2(mag),?
    //      level = clamp(q4*8 - OFFSET[k], 0, 255)
    //  ?? p50 -> ~80/255?p90 -> ~185/255??? -> ?? 255,?????????
    //  OFFSET ?????????????:??????????? 6 ????,
    //  ????,???????????,??? 8 ?????
    // ======================================================================
    // 4*log2(mag) ?????:?????????,?????? 2 ?
    function [6:0] log2_q4;
        input [24:0] m;
        integer b;
        reg [4:0] msb;
        reg [1:0] fr;
        begin
            msb = 5'd0;
            for (b = 24; b >= 0; b = b - 1)
                if (m[b]) msb = b[4:0];
            if (msb >= 5'd2)      fr = {m[msb-1], m[msb-2]};
            else if (msb == 5'd1) fr = {m[0], 1'b0};
            else                  fr = 2'b00;
            log2_q4 = {msb[4:0], fr};
        end
    endfunction

    // level = clamp(q4*8 - OFFSET[k], 0, 255);OFFSET ????????
    function [8:0] level_of;
        input [6:0] q4;
        input integer k;
        reg [10:0] off;
        reg signed [11:0] v;
        begin
            case (k)
                0:       off = 11'd32;
                1:       off = 11'd38;
                2:       off = 11'd44;
                3:       off = 11'd48;
                4:       off = 11'd48;
                5:       off = 11'd48;
                6:       off = 11'd42;
                default: off = 11'd32;
            endcase
            v = $signed({1'b0, q4, 3'b000}) - $signed({1'b0, off});
            if (v < 12'sd0)        level_of = 9'd0;
            else if (v > 12'sd255) level_of = 9'd255;
            else                   level_of = v[8:0];
        end
    endfunction

    // 25bit ?????,?? pole_k - pole_k+1 ??? 24bit ???
    wire signed [24:0] pole_e [0:NP-1];
    genvar gi;
    generate
        for (gi = 0; gi < NP; gi = gi + 1) begin : g_pole_ext
            assign pole_e[gi] = {{1{pole[gi][23]}}, pole[gi]};
        end
    endgenerate

    reg [24:0] band_mag [0:7];
    reg [7:0]  band_tgt [0:7];
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            band_mag[i] = (pole_e[i] >= pole_e[i+1])
                        ? (pole_e[i]   - pole_e[i+1])
                        : (pole_e[i+1] - pole_e[i]  );
            band_tgt[i] = level_of(log2_q4(band_mag[i]), i);
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            decay_cnt <= 8'd0;
            for (i = 0; i < 8; i = i + 1)
                band_lvl[i] <= 8'd0;
        end else if (samp_en) begin
            if (decay_cnt >= DECAY_TICK[7:0])
                decay_cnt <= 8'd0;
            else
                decay_cnt <= decay_cnt + 8'd1;

            for (i = 0; i < 8; i = i + 1) begin
                if (band_tgt[i] >= band_lvl[i])
                    band_lvl[i] <= band_tgt[i];                    // 攻击：立即跟上
                else if (decay_cnt >= DECAY_TICK[7:0])
                    band_lvl[i] <= (band_lvl[i] == 8'd0) ? 8'd0
                                                         : band_lvl[i] - 8'd1;
            end
        end
    end

    //=========================================================================
    //  4) 峰值电平 → 数码管 + LED
    //=========================================================================
    reg [7:0] peak_max;
    always @(*) begin
        peak_max = band_lvl[0];
        for (i = 1; i < 8; i = i + 1)
            if (band_lvl[i] > peak_max)
                peak_max = band_lvl[i];
    end

    // 0~255 → 0~99
    // 注意位宽：peak_max*100 只有 15bit，直接 >>8 会触发 TD 的
    // "right shift count >= width of value" 警告，所以先零扩展到 24bit 再移
    wire [23:0] peak_x100  = {16'd0, peak_max} * 24'd100;
    wire [6:0]  peak_100   = peak_x100[15:8];

    // 保持 LEVEL_MS，并且"只上不下"（慢释放），数码管数字才看得清
    reg [6:0]  level_hold;
    reg [15:0] level_cnt;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            level_hold <= 7'd0;
            level_cnt  <= 16'd0;
        end else begin
            if (level_cnt >= LEVEL_TICK[15:0]) begin
                level_cnt  <= 16'd0;
                level_hold <= peak_100;
            end else begin
                level_cnt  <= level_cnt + 16'd1;
                if (peak_100 > level_hold)
                    level_hold <= peak_100;
            end
        end
    end

    // 十进制 → 共阳数码管段码 {DP,g,f,e,d,c,b,a}，低电平点亮
    function [7:0] seg7;
        input [3:0] d;
        begin
            case (d)
                4'd0:    seg7 = 8'b1_100_0000;
                4'd1:    seg7 = 8'b1_111_1001;
                4'd2:    seg7 = 8'b1_010_0100;
                4'd3:    seg7 = 8'b1_011_0000;
                4'd4:    seg7 = 8'b1_001_1001;
                4'd5:    seg7 = 8'b1_001_0010;
                4'd6:    seg7 = 8'b1_000_0010;
                4'd7:    seg7 = 8'b1_111_1000;
                4'd8:    seg7 = 8'b1_000_0000;
                default: seg7 = 8'b1_001_0000;   // 9
            endcase
        end
    endfunction

    // 十位 / 个位：用"减 10"法，不用除法器
    reg [3:0] d_tens, d_ones;
    always @(*) begin
        if      (level_hold >= 7'd90) begin d_tens = 4'd9; d_ones = level_hold - 7'd90; end
        else if (level_hold >= 7'd80) begin d_tens = 4'd8; d_ones = level_hold - 7'd80; end
        else if (level_hold >= 7'd70) begin d_tens = 4'd7; d_ones = level_hold - 7'd70; end
        else if (level_hold >= 7'd60) begin d_tens = 4'd6; d_ones = level_hold - 7'd60; end
        else if (level_hold >= 7'd50) begin d_tens = 4'd5; d_ones = level_hold - 7'd50; end
        else if (level_hold >= 7'd40) begin d_tens = 4'd4; d_ones = level_hold - 7'd40; end
        else if (level_hold >= 7'd30) begin d_tens = 4'd3; d_ones = level_hold - 7'd30; end
        else if (level_hold >= 7'd20) begin d_tens = 4'd2; d_ones = level_hold - 7'd20; end
        else if (level_hold >= 7'd10) begin d_tens = 4'd1; d_ones = level_hold - 7'd10; end
        else                          begin d_tens = 4'd0; d_ones = level_hold;        end
    end

    assign level_peak = {1'b0, level_hold};
    assign level_disp = {seg7(d_tens), seg7(d_ones)};

    //=========================================================================
    //  5) 频谱柱 → 像素图层
    //=========================================================================
    //  柱 k 的水平范围 [bar_lo(k), bar_hi(k)]
    //  柱 k 的高度     h = (band_lvl[k] * BAR_FULL_H) >> 8   (BAR_FULL_H=48 → 24bit)
    //  点亮条件        y ∈ (SPEC_Y_BASE - h, SPEC_Y_BASE]
    //  另加 2 像素高的基底线，让"静音时也有一排刻度"
    //=========================================================================
    reg          cur_bar_vld;
    reg [3:0]    cur_bar_id;
    reg          hit;
    reg [23:0]   col;

    always @(*) begin
        cur_bar_id  = 4'd0;
        cur_bar_vld = 1'b0;
        for (i = 0; i < 8; i = i + 1) begin
            if (!cur_bar_vld && x >= bar_lo(i) && x <= bar_hi(i)) begin
                cur_bar_id  = i[3:0];
                cur_bar_vld = 1'b1;
            end
        end
    end

    // 24bit 高度，避免 8bit 溢出（48 * 255 需要 16bit 中间量）
    wire [7:0]  cur_lvl   = band_lvl[cur_bar_id];
    // BAR_FULL_H is 48 = 32 + 16; use shifts so this does not consume a DSP.
    // [FIX v7c] ????? lvl*49>>8,??? 48 px(??? 96 px,?????)?
    // ?? lvl*3>>3 = lvl*96/256,???? BAR_FULL_H,?????????
    wire [9:0]  band_h     = {3'd0,cur_lvl[7:3]} + {5'd0,cur_lvl[7:4]};   // lvl*(1/8+1/16) = lvl*48/256 -> max 47 px
    wire [9:0]  bar_top_y = SPEC_Y_BASE[9:0] - band_h[9:0];

    // 柱身命中：只有一个 always 块驱动 hit/col，避免多驱动
    always @(*) begin
        hit = 1'b0;
        col = COLOR_LOW;
        if (de && cur_bar_vld && x >= SPEC_X0[9:0] && x <= SPEC_X1[9:0]) begin
            if (cur_lvl != 8'd0 &&
                y <= SPEC_Y_BASE[9:0] && y >= bar_top_y) begin
                hit = 1'b1;
                if      (cur_lvl < COL_T1) col = COLOR_LOW;
                else if (cur_lvl < COL_T2) col = COLOR_MID;
                else                       col = COLOR_HIGH;
            end
        end
    end

    // 柱基底线（y = SPEC_Y_BASE+1），静音时也有参考刻度
    wire baseline_hit = de && cur_bar_vld &&
                        (y == (SPEC_Y_BASE[9:0] - 10'd1));

    //=========================================================================
    //  6) 自测模式：整屏彩条（验证 spec_rgb/spec_alpha 通路，不依赖音频）
    //     接口契约第七节要求"每个模块都要有一条不依赖别人的验证通道"
    //=========================================================================
    reg        st_hit;
    reg [23:0] st_col;
    always @(*) begin
        st_hit = 1'b0;
        st_col = 24'h0;
        // 8 条竖彩条，颜色由对应段的电平决定 → 直接看出滤波器组在动
        if (de && y >= 10'd60 && y <= 10'd400) begin
            for (i = 0; i < 8; i = i + 1) begin
                if (x >= (i * 80) && x <= (i * 80 + 79)) begin
                    st_hit = 1'b1;
                    st_col = {band_lvl[i][7:5], 5'b0,
                              band_lvl[i][7:4], 4'b0,
                              band_lvl[i][7:6], 6'b0};
                end
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            spec_rgb   <= 24'h0;
            spec_alpha <= 8'd0;
        end else if (self_test) begin
            spec_rgb   <= st_hit ? st_col : 24'h0;
            spec_alpha <= st_hit ? 8'd255 : 8'd0;
        end else if (!sw_audio_en) begin
            spec_rgb   <= 24'h0;
            spec_alpha <= 8'd0;
        end else begin
            spec_rgb   <= hit           ? col
                        : baseline_hit  ? COLOR_BG
                                        : 24'h0;
            spec_alpha <= (hit || baseline_hit) ? 8'd255 : 8'd0;
        end
    end

    //=========================================================================
    //  7) LED3 音频有效指示（C10）
    //     亮度随电平变化：既证明"有音频流"，又证明"电平在动"
    //=========================================================================
    reg [7:0] pwm_cnt;
    always @(posedge clk or posedge rst) begin
        if (rst) pwm_cnt <= 8'd0;
        else     pwm_cnt <= pwm_cnt + 8'd1;
    end

    always @(posedge clk or posedge rst) begin
        if (rst)
            led_audio_act <= 1'b0;
        else if (self_test)
            led_audio_act <= 1'b1;
        else if (!audio_valid)
            led_audio_act <= 1'b0;                          // 无音频流：灭
        else
            // 有音频流：最低 1/4 亮度，电平越高越亮
            led_audio_act <= (pwm_cnt < (8'd64 + (peak_max >> 1)));
    end

endmodule
