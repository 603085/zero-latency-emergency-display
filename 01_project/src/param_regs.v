// ============================================================================
//  param_regs.v -- 公共参数接口模块 / 参数字典  (Task B, skeleton item)
//
//  接口说明（重要）：本文件 = 《TaskB开发指南》§3 的冻结签名（17 个端口：
//  5 个输入 + 12 个输出）**加上本工程新增的第 13 个输出 `volume`**（音量），
//  现在共 18 个端口。除 volume 之外，其余端口名 / 方向 / 位宽 / 参数默认值
//  与冻结签名逐字一致，B 的原件可以按"补一个 volume"的方式替换回来。
//
//  ⚠️ 这是"按冻结接口重写"的一份，不是 B 那份跑了 47 项仿真的原件 ——
//     行为按文档描述实现（见下），但**没有在本工程里跑过仿真**。
//     B 的原件到位后请用它替换本文件并重跑 tb_param_regs。
//
//  契约地位（《接口契约与分工表》§4.2 + §6）：
//      param_regs.v 是【全系统唯一的参数源】。别人只读，不得在别处再声明
//      一份参数寄存器 —— 否则会出现两个"真值源"，联调必翻车。
//      本文件落地后，top 里那段临时的手写亮度加减必须删掉（已经删了）。
//
//  它管什么
//  --------
//      按 C1 循环选择参数项，按 C8 / C7 加减当前项，并把结果发布给：
//        A 的图层（亮度 / 对比度 / 转场类型 / 配色预设）
//        B 自己的 OSD 条（osd_visible / osd_alpha / cur_param / osd_value）
//        轮播计时（dwell_ms，目前 A 侧还没有对应端口，见文件尾注释）
//
//      参数表（《TaskB开发指南》第三节，已冻结）：
//        cur_param 0 亮度     0..255   步进 8
//        cur_param 1 对比度   0..255   步进 8
//        cur_param 2 转场类型 0/1     双向切换（0=硬切 1=淡入淡出）
//        cur_param 3 停留时间 200..10000 步进 250
//        cur_param 4 配色预设 0..3    循环
//      所有参数【饱和钳位，不回绕】。
//
//  ⚠️ 本工程在冻结的 5 项之外【新增了第 6 项】：
//        cur_param 5 音量     0..255   步进 8   （128 = 原音量，0 = 静音；上电默认 0 = 静音）
//     这是对《接口契约与分工表》§4.2 参数字典和《TaskB开发指南》§3 接口的
//     有意扩充 —— 新增了输出端口 `volume`。按契约"新参数向 B 提、由 B 加入"
//     的规矩，这一项要同步给 B 和他的原件，否则他的文件替换本文件时
//     top 会因为缺少 volume 端口而编译失败。参数字典那一行也建议补上。
//
//  ============================================================================
//  交互模型（★ 2026-10-14 第三次修订，最终版）
//
//      KEY4 (C1)  短按              → **切换**到下一个参数项（并弹出 OSD）
//      KEY4 (C1)  长按（≥ 0.6 s）   → **只弹出 OSD**（显示当前项），
//                                    不换项、不改数值
//      C8  (SW1)  上拨/按住         → **只有它才改数值**，按【C7 的方向】走
//                                    （160 ms 第一步、之后每 40 ms 一步；原手感）
//      C7  (SW2)  方向选择          → 拨上 = +，拨下 = −
//      无操作 2 s                   → OSD 自己消失（OSD_HOLD = 2 s，见下）
//
//      ★ 两个要点（都是用户明确要求的）：
//        · **KEY4 不改数值** —— 改值权限只给 C8；
//        · **长按只负责把 OSD 显示出来**（"我之前在调哪一项"一眼可见），
//          不切换、不改值；长按后松手也不会再切一项（key_long_fired 挡住）。
//
//  时间参数
//  ------------------------------------------------------------------
//      按键消抖 ：KEY_DEBOUNCE = 2^18 / 25 MHz ≈ 10.5 ms
//      长按阈值 ：KEY_LONG_TICKS = CLK_HZ*6/10 = 0.6 s
//      C8 连续  ：按住 160 ms 走第一步，之后每 40 ms 一步（原手感，未改）
//      OSD 保持 ：OSD_HOLD = 50_000_000 / 25 MHz = 2 s，然后 250 ms 淡出
//
//      ⚠️ 两侧都有"上电先确认空闲"的门：
//         · C8：复位后等 SETTLE_TICKS(≈83 ms) 且确认 C8 在 0 位，才允许连续步进
//           （否则"复位时 C8 停在 1 位"会被当成一直按着，约 0.7 s 扫到端点）；
//         · C1：复位后必须先见到按键【松开】一次（key_idle_seen），才认按键事件
//           （否则"上电时 C1 被按住/卡低"会一直换项）。
//      ⚠️ C7 是【方向电平】，不参与任何"武装"判断：它停在 1 位只表示方向为 +。
//
//  OSD 时序（TaskB开发指南 第二部分 PART 5）
//  ----------------------------------------
//      任何一次操作（短按换项 / 长按弹 OSD / C8 改值）→ 弹出：
//          osd_visible=1, osd_alpha=255；之后保持 OSD_HOLD（2 s），
//          再用 OSD_FADE（250 ms）把 alpha 从 255 渐降，降到 0 时撤回。
//      期间再操作 → 立即重新弹出（osd_alpha 回 255，重新计时）。
//
//      注意：A 的 layer_mixer 第 7 级是 (osd_alpha == 255) 的二选一 mux，
//      不是混合器 —— 所以这里的"渐降"在屏上表现为**中途突然消失**，
//      不是真正的淡出。要真淡出得改 layer_mixer（加 3 个乘法器），
//      这一点在 osd_bar.v 的文件头也写了。
// ============================================================================

`timescale 1ns / 1ps

module param_regs #(
    parameter integer CLK_HZ         = 25_000_000,
    parameter integer SYNC_STAGES    = 2,
    parameter integer KEY_DEBOUNCE   = 18,            // 2^18/25MHz = 10.5 ms
    parameter [22:0]  RATE_FAST      = 23'd4_000_000, // 160 ms 首次步进
    parameter [22:0]  RATE_SLOW      = 23'd1_000_000, //  40 ms 重复步进
    parameter [25:0]  OSD_HOLD       = 26'd50_000_000,// 2 s OSD 保持
    parameter [22:0]  OSD_FADE       = 23'd6_250_000, // 250 ms 淡出
    parameter [15:0]  DWELL_MIN      = 16'd200,
    parameter [15:0]  DWELL_MAX      = 16'd10000,
    parameter [15:0]  DWELL_STEP     = 16'd250,
    parameter [15:0]  DWELL_DEFAULT  = 16'd3000
)(
    input  wire        clk,              // video_clk 25 MHz
    input  wire        rst,              // 高有效

    // ---------------- 物理输入 ----------------
    //  ★ 2026-10-14 交互改版（用户要求）：
    //      KEY4(C1) 短按 → 当前参数按【方向】±1 步
    //      KEY4(C1) 长按 → 换参数项（按住继续每 0.4 s 再换一项）
    //      C7(SW2)       → 方向选择：拨上 = +，拨下 = −（原来是"参数 −"）
    //      C8(SW1)       → 按住 = 按【当前方向】连续步进（原来是"参数 +"）
    input  wire        key_next_param,   // C1 KEY4：短按改数值 / 长按换参数项（低有效）
    input  wire        sw_up,            // C8：按住 = 按当前方向连续步进
    input  wire        dir_up,           // C7：方向选择，1 = +，0 = −

    // ---------------- 参数输出 → A 的图层 ----------------
    output reg  [7:0]  brightness,       // 128 = 中性
    output reg  [7:0]  contrast,         // 128 = 中性
    output reg  [1:0]  transition,       // 0 = 硬切, 1 = 淡入淡出
    output reg  [15:0] dwell_ms,         // 200 ~ 10000
    output reg  [1:0]  preset,           // 0 标准 1 应急 2 夜间 3 高对比
    output reg         preset_en,        // 用户是否真的选过预设
    // ★ 本工程新增（冻结接口之外）：音量。见文件头"新增第 6 项"的说明，
    //   替换成 B 的原件之前必须把这一项同步过去。
    output reg  [7:0]  volume,           // 128 = 原音量，0 = 静音

    // ---------------- OSD 控制 → osd_bar ----------------
    output reg  [2:0]  cur_param,        // 0..5（5 = 音量）
    output reg         osd_visible,
    output reg  [7:0]  osd_alpha,
    output reg  [7:0]  osd_value,        // 当前项的 8 位码（见下方注释）

    // ---------------- 状态 ----------------
    output reg         activity,         // 每次操作一个单周期脉冲
    output reg  [15:0] step_count        // 累计步进次数（调试用）
);

    // ==================================================================
    //  常量
    // ==================================================================
    localparam [7:0]  BRIGHT_DEFAULT   = 8'd128;
    localparam [7:0]  CONTRAST_DEFAULT = 8'd128;
    localparam [7:0]  VOLUME_DEFAULT   = 8'd255;   // [FIX 2026-10-25b] boot at maximum.
                                                   // SD 语音的峰值因数很高，4x 样点软增益后
                                                   // 仍比满幅测试音低约 4~6 dB；255 让开机音量
                                                   // 再乘约 2x，与测试音听感相当。
                                                   // 仍可用 KEY4/SW1/SW2 把音量调低。
    localparam [1:0]  TRANS_DEFAULT    = 2'd1;
    localparam [7:0]  STEP8            = 8'd8;
    localparam [2:0]  PARAM_LAST       = 3'd5;      // 参数项 0..5（5 = 音量）

    // 每次淡出步进之间的间隔：256 步覆盖 OSD_FADE，约 0.98 ms/步。
    // 用移位而不是除法，避免引入除法器。
    localparam [22:0] FADE_TICK = (OSD_FADE >> 8) == 23'd0 ? 23'd1 : (OSD_FADE >> 8);

    // ==================================================================
    //  饱和加减（所有参数都不回绕）
    // ==================================================================
    function [7:0] sat_add8;
        input [7:0] v;
        input [7:0] s;
        begin
            sat_add8 = (v > (8'd255 - s)) ? 8'd255 : (v + s);
        end
    endfunction

    function [7:0] sat_sub8;
        input [7:0] v;
        input [7:0] s;
        begin
            sat_sub8 = (v < s) ? 8'd0 : (v - s);
        end
    endfunction

    function [15:0] sat_add16;
        input [15:0] v;
        input [15:0] s;
        input [15:0] hi;
        begin
            sat_add16 = ((v + s) > hi) ? hi : (v + s);
        end
    endfunction

    function [15:0] sat_sub16;
        input [15:0] v;
        input [15:0] s;
        input [15:0] lo;
        begin
            sat_sub16 = (v < (lo + s)) ? lo : (v - s);
        end
    endfunction

    // ==================================================================
    //  输入同步：C1 按键 + C8 拨码 + C7 方向拨码（都是异步输入）
    // ==================================================================
    reg  [SYNC_STAGES-1:0] key_sync;
    reg  [SYNC_STAGES-1:0] up_sync, dir_sync;

    wire key_in = key_sync[SYNC_STAGES-1];   // 低有效
    wire up_in  = up_sync[SYNC_STAGES-1];    // C8：按住 = 连续步进
    wire dir_in = dir_sync[SYNC_STAGES-1];   // C7：1 = 方向 +

    // ---------------- C1 消抖：电平必须连续稳定 KEY_DEBOUNCE 位 ----
    reg  [KEY_DEBOUNCE-1:0] key_db_cnt;
    reg  key_db, key_db_d;

    // ==================================================================
    //  C1(KEY4) 短按 = 切换参数项 / 长按 = 只弹出 OSD
    //
    //    按下 <  KEY_LONG_TICKS(0.6 s) 松手 → 短按：cur_param 前进一项（并弹 OSD）
    //    按下 ≥  KEY_LONG_TICKS            → 长按：**只把 OSD 弹出来**（显示当前项），
    //                                        不换项、不改数值
    //    长按触发过之后松手【不再】执行短按动作
    //    （所以"长按弹 OSD、松手又切走一项"这种来回跳不会发生）
    //
    //  ⚠️ 与 C8 对称的"上电先确认空闲"门：复位后必须先看到按键【松开】一次，
    //     才认按键事件（否则上电时 C1 被按住/卡低会一直换项、把 OSD 一直点亮）。
    // ==================================================================
    localparam [23:0] KEY_LONG_TICKS = CLK_HZ * 6 / 10;   // 0.6 s（随 CLK_HZ 推导）

    reg [23:0] key_hold;          // 按住还差多少拍到"长按"；到 0 触发
    reg        key_long_fired;    // 这一轮按下的长按已经触发过
    reg        key_idle_seen;     // 复位后见过一次"松开"（见上面的 ⚠️）

    // 低有效按键：松手沿 = 上一拍为低、这一拍为高
    wire key_release = ~key_db_d & key_db;
    wire key_short   = key_release && !key_long_fired && key_idle_seen;
    // 长按事件：按住到阈值那一次（带 !key_long_fired → 只触发一次、不重复）。
    // 它的唯一作用是让 OSD 弹出来（见 act），**不改任何参数**。
    wire key_long    = (!key_db) && (!key_long_fired) && (key_hold == 24'd0) && key_idle_seen;

    // ==================================================================
    //  C8：按住按【当前方向】连续步进
    //    - 松开时把计时器装成 RATE_FAST → 下次按住要等 160 ms 才走第一步
    //    - 走完一步装成 RATE_SLOW    → 之后每 40 ms 一步
    //    - 到 0 的那一拍就是"步进拍"，同时立刻重装（所以实际间隔是 N+1 拍）
    //
    //  ⚠️ "武装"门：复位后先等 SETTLE_TICKS(≈84 ms)，且确认 C8 在 0 位，
    //     才允许连续步进。否则"复位时 C8 正停在 1 位"会被当成用户一直按着，
    //     约 0.7 s 把参数推到端点，而且每次复位都这样。
    //  ⚠️ C7 是【方向电平】，不参与武装判断（不会自己扫参数）。
    // ==================================================================
    localparam [20:0] SETTLE_TICKS = CLK_HZ / 12;     // ≈83 ms（随 CLK_HZ 推导）

    reg  [22:0] up_tmr;
    reg  [20:0] settle_tmr;
    reg         armed;
    //  循环参数「本轮按下已经步进过」标志（2026-10-16 加）
    //  只对 cur_param 2(转场类型) / 4(配色预设) 生效：它们是循环选择，
    //  40 ms 一步会把整圈瞬间转完，根本选不中。
    //  置位后本轮不再步进，松手清零。
    reg         stepped_this_press;

    //  循环参数：转场类型(2) / 配色预设(4)。档数少，不能自动重复。
wire cyclic_param = (cur_param == 3'd2) || (cur_param == 3'd4);

//  循环参数在本轮按下里已经走过一步 -> 不再走（松手清零，见下面 always 块）
wire up_step = armed && up_in && (up_tmr == 23'd0)
                && !(cyclic_param && stepped_this_press);

    // ---------------- "改数值"的唯一入口：C8 上拨/按住 ----------------
    //   ★ KEY4 不再改数值（用户要求"只有拨码上拨时才改数据"）
    wire step_evt = up_step;
    wire step_up  = dir_in;        // C7 拨上 = +，拨下 = −

    // 任何一次操作 → 重新弹出 OSD
    wire act = key_short || key_long || step_evt;

    // ==================================================================
    //  主状态机：参数项 / 参数值 / step_count
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            key_sync   <= {SYNC_STAGES{1'b1}};      // 按键空闲为高
            up_sync    <= {SYNC_STAGES{1'b0}};
            dir_sync   <= {SYNC_STAGES{1'b0}};
            key_db     <= 1'b1;
            key_db_d   <= 1'b1;
            key_db_cnt <= {KEY_DEBOUNCE{1'b0}};
            key_hold       <= KEY_LONG_TICKS;
            key_long_fired <= 1'b0;
            key_idle_seen  <= 1'b0;
            up_tmr     <= RATE_FAST;
            stepped_this_press <= 1'b0;
            settle_tmr <= 21'd0;
            armed      <= 1'b0;

            brightness <= BRIGHT_DEFAULT;
            contrast   <= CONTRAST_DEFAULT;
            volume     <= VOLUME_DEFAULT;
            transition <= TRANS_DEFAULT;
            dwell_ms   <= DWELL_DEFAULT;
            preset     <= 2'd0;
            preset_en  <= 1'b0;
            cur_param  <= 3'd0;
            step_count <= 16'd0;
            activity   <= 1'b0;
        end else begin
            // ---------------- 同步 ----------------
            key_sync <= {key_sync[SYNC_STAGES-2:0], key_next_param};
            up_sync  <= {up_sync[SYNC_STAGES-2:0],  sw_up};
            dir_sync <= {dir_sync[SYNC_STAGES-2:0], dir_up};

            // ---------------- C1 消抖 ----------------
            // 与当前电平一致 → 计数清零；连续 KEY_DEBOUNCE 位不同 → 认账。
            // 计数上限写成 {KEY_DEBOUNCE{1'b1}}，正好是 2^N-1，没有截断。
            if (key_in == key_db) begin
                key_db_cnt <= {KEY_DEBOUNCE{1'b0}};
            end else if (key_db_cnt == {KEY_DEBOUNCE{1'b1}}) begin
                key_db     <= key_in;
                key_db_cnt <= {KEY_DEBOUNCE{1'b0}};
            end else begin
                key_db_cnt <= key_db_cnt + 1'b1;
            end
            key_db_d <= key_db;

            // ---------------- C8 连续步进计时（方向由 C7 决定）--------
            // ---------------- 循环参数「本轮已步进」标志 ----------------
            //  松手清零 -> 下次按下可以再走一步；
            //  走完一步就置位 -> 按住不放不会继续转圈。
            //  连续量（亮度/对比度/停留/音量）不看这个标志，照样自动重复。
            if (!up_in)          stepped_this_press <= 1'b0;
            else if (step_evt)   stepped_this_press <= 1'b1;

            if (!up_in)                 up_tmr <= RATE_FAST;
            else if (up_tmr != 23'd0)   up_tmr <= up_tmr - 23'd1;
            else                        up_tmr <= RATE_SLOW;

            // ---------------- C1 短按 / 长按 计时 ----------------
            // 上电先确认空闲：见过一次"松开"（key_db=1）才认按键事件
            if (key_db) key_idle_seen <= 1'b1;

            if (key_db) begin
                // 松手：把长按计时装回阈值，并清"已触发"标志
                key_hold       <= KEY_LONG_TICKS;
                key_long_fired <= 1'b0;
            end else if (key_hold != 24'd0) begin
                key_hold <= key_hold - 24'd1;
            end else begin
                // 到点：key_long 这一拍为 1（只弹 OSD，见 act）；保持置位直到松手，
                // 所以长按只触发一次、不重复（用户要可预测，不要"按住一直跳"）
                key_long_fired <= 1'b1;
            end

            // ---------------- 武装：电平稳定且 C8 在 0 位 ----------------
            // （C7 是方向电平，不参与武装判断 —— 它不会自己扫参数）
            if (settle_tmr != SETTLE_TICKS)
                settle_tmr <= settle_tmr + 21'd1;
            if ((settle_tmr == SETTLE_TICKS) && !up_in)
                armed <= 1'b1;

            // ---------------- 换参数项：只由短按触发 ----------------
            //  ★ 长按【不换项、不改值】，它的唯一作用是让 OSD 弹出来（见上面的 act）
            if (key_short) begin
                cur_param <= (cur_param == PARAM_LAST) ? 3'd0 : (cur_param + 3'd1);
            end

            // ---------------- 改当前参数的数值（只由 C8 触发）----------------
            // 方向 = C7。独立的一个 if（不是 else if）：万一同一拍既换项又改值，
            // 两件事都做，不会互相吞掉（改的是不同寄存器）。
            if (step_evt) begin
                case (cur_param)
                    3'd0: brightness <= step_up ? sat_add8(brightness, STEP8)
                                                : sat_sub8(brightness, STEP8);
                    3'd1: contrast   <= step_up ? sat_add8(contrast,   STEP8)
                                                : sat_sub8(contrast,   STEP8);
                    // 转场类型：双向都切换（0 硬切 ←→ 1 淡入淡出）。
                    // 契约只定义 0/1；A 的 layer_mixer 里那个 2（径向光圈）
                    // 因为 USE_IRIS=0 本来就显示不出来，所以不开放。
                    3'd2: transition <= (transition == 2'd0) ? 2'd1 : 2'd0;
                    3'd3: dwell_ms   <= step_up ? sat_add16(dwell_ms, DWELL_STEP, DWELL_MAX)
                                                : sat_sub16(dwell_ms, DWELL_STEP, DWELL_MIN);
                    3'd4: begin
                        // 配色预设 0..3 循环；一旦用户真的动过它就置 preset_en，
                        // A 那边只有 preset_en=1 才会采用用户选的预设
                        // （应急模式下 A 仍然强制用应急配色，这是安全规则）。
                        preset    <= step_up ? ((preset == 2'd3) ? 2'd0 : (preset + 2'd1))
                                             : ((preset == 2'd0) ? 2'd3 : (preset - 2'd1));
                        preset_en <= 1'b1;
                    end
                    3'd5: volume     <= step_up ? sat_add8(volume, STEP8)
                                                : sat_sub8(volume, STEP8);
                    default: begin end   // cur_param 只会在 0..PARAM_LAST 内
                endcase

                if (step_count != 16'hFFFF)
                    step_count <= step_count + 16'd1;
            end

            // ---------------- activity：每次操作一个单周期脉冲 ----
            // 这里【不用】"先清零后置位"的写法（B 的文档把它列为坑 2），
            // 而是先算好组合条件 act，再只赋一次值。
            activity <= act;
        end
    end

    // ==================================================================
    //  osd_value：当前参数项的 8 位码
    //  0/1 亮度对比度 → 值本身（OSD 用它显示十进制 + 进度条）
    //  2   转场类型   → 0/1（OSD 自己按 CUT/FADE 显示）
    //  3   停留时间   → dwell_ms 的高 8 位（只是粗码，OSD 显示 5 位十进制
    //                    用的是 dwell_ms 端口，不是这个值）
    //  4   配色预设   → 0..3（OSD 自己按 STD/EMG/NIGHT/HI 显示）
    //  5   音量       → 值本身（OSD 显示十进制 + 进度条）
    // ==================================================================
    always @(*) begin
        case (cur_param)
            3'd0:    osd_value = brightness;
            3'd1:    osd_value = contrast;
            3'd2:    osd_value = {6'd0, transition};
            3'd3:    osd_value = dwell_ms[15:8];
            3'd4:    osd_value = {6'd0, preset};
            3'd5:    osd_value = volume;
            // 五项显式列出，不要用 default 承载"音量"语义：
            // 将来再加第 7 项时，default 会让它静默按音量显示。
            default: osd_value = 8'd0;
        endcase
    end

    // ==================================================================
    //  OSD 弹出 / 保持 / 淡出
    //
    //  任何操作 → 立刻弹出（alpha=255），并重装保持计时器；
    //  保持计时器走完 → 进入淡出：每 FADE_TICK 拍把 alpha 减 1；
    //  alpha 减到 0 → 撤回（osd_visible=0）。
    //
    //  淡出期间再操作 → !osd_fading 分支把它拉回 255 并重新保持。
    // ==================================================================
    reg [25:0] osd_tmr;
    reg        osd_fading;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            osd_visible <= 1'b0;
            osd_alpha   <= 8'd0;
            osd_tmr     <= 26'd0;
            osd_fading  <= 1'b0;
        end else if (act) begin
            osd_visible <= 1'b1;
            osd_alpha   <= 8'd255;
            osd_tmr     <= OSD_HOLD;
            osd_fading  <= 1'b0;
        end else if (osd_visible) begin
            if (!osd_fading) begin
                if (osd_tmr != 26'd0) begin
                    osd_tmr <= osd_tmr - 26'd1;
                end else begin
                    osd_fading <= 1'b1;
                    osd_tmr    <= FADE_TICK;
                end
            end else begin
                if (osd_alpha == 8'd0) begin
                    osd_visible <= 1'b0;      // 淡完了，撤回
                end else if (osd_tmr != 26'd0) begin
                    osd_tmr <= osd_tmr - 26'd1;
                end else begin
                    osd_alpha <= osd_alpha - 8'd1;
                    osd_tmr   <= FADE_TICK;
                end
            end
        end
    end

    // dwell_ms 目前没有下游：A 侧的轮播计时在 sd_card_bmp 里（auto_cnt / auto_tick），
    // 那个模块还没有"停留时间"输入端口。要让 C7/C8 真的改变轮播间隔，
    // 需要 A 在 sd_card_bmp 上加一个 dwell 输入（属于 A 的模块，接口契约 §6）。
    // 在此之前本端口会悬空 —— 这是有意的，不是漏接。

endmodule
