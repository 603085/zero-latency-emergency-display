
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [3:0]   SCAN_TARGET_COUNT = 4'd15   // 4 位上限就是 15
)(
    input                       clk,
    input                       rst,
    // ---- added by Task A: the next-image request now comes from the FSM ----
    //
    // It used to be key_next, read directly here. That made a deferred
    // transition impossible: the moment the key was pressed this module began
    // reading the card, so the picture changed before anything could fade out.
    //
    // next_req is a LEVEL from the layer FSM, raised only once the current
    // picture has finished fading to black. key_next is kept as an input for
    // compatibility but no longer starts a load on its own.
    input                       next_req,
    input                       key_next,
    input                       key_auto,
    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    // ---- added by Task A: sequencing for a two-phase transition ----
    //
    // hold_load pauses the START of a new image load. The layer stack raises it
    // when the user asks for the next picture, fades the CURRENT picture to
    // black, and only then releases it. That is what makes a真 fade-to-black
    // possible: the display buffer swaps while the screen is already dark, so
    // the change itself is never visible.
    //
    // Without this the swap happens whenever the card read finishes, which can
    // be long after the user pressed the key -- and the only thing left to
    // animate is the NEW picture appearing, which reads as a hard cut with a
    // fade tacked on.
    input                       hold_load,
    // busy_stack: the layer stack is in the middle of a transition (fading out,
    // waiting for the swap, or fading in). Auto rotation must not raise a new
    // request while this is high -- see the note on the auto-rotation condition
    // below.
    input                       busy_stack,
    // load_req: a LEVEL from the layer FSM, high for the whole of its
    // WAIT_SWAP phase, meaning "start the load now".
    //
    // It is a level, not a pulse, on purpose. As a pulse it had to be caught
    // within a few clocks and was sometimes missed outright, leaving the load
    // never started and the FSM waiting forever -- measured on the board as
    // want_cnt = 1, owed_cnt = 1, state_code = 1, black screen.
    input                       load_req,
    // load_busy exposed so the caller can tell "the load has actually started"
    // and release hold_load at the right moment.
    output                      load_busy_o,
    // next_req_pending is the REQUEST half of the two-phase handshake: high
    // while this module wants to load the next picture (raised by the key
    // press or by the auto-rotation tick), clearing once the load starts.
    // The layer FSM watches it and answers with hold_load / next_req.
    output                      next_req_pending,
    // auto_play_en exposed for the 7-segment diagnostic: digit 4 shows '1'
    // while rotation is running and '0' when it is stopped, so a press of the
    // rotate key can be confirmed by looking at the board.
    output                      auto_play_state,
    // how many pictures the scan found. Exposed because rotation can
    // only move between pictures if there is more than one, and with a
    // single picture every rotation reloads the same file -- which looks
    // exactly like rotation not working.
    output [3:0]                img_found_o,

    // ======================================================================
    //  *** THE EMERGENCY PICTURE ***   added by Task A
    //
    //  CONVENTION: THE LAST PICTURE ON THE CARD IS THE EMERGENCY ONE.
    //
    //  While emg_sel is high this module jumps straight to that last picture,
    //  and the normal rotation is made to NEVER reach it (see
    //  next_index_limited). So the emergency map is on the card like any other
    //  file, but it only ever appears when the emergency switch is thrown.
    //
    //  It is a LEVEL, driven from the layer FSM's mode output (mode == 1).
    //
    //  IT IS NOT INSTANT, AND THAT IS A HARDWARE LIMIT. The BMP still has to
    //  be read off the card: 640*480*24 bits = 900 KB, and the SPI runs at
    //  100 MHz / ((0 + 2) * 2) = 25 MHz, so ~0.29 s of pure data plus FAT and
    //  sector overhead -- about half a second in practice.
    //
    //  What IS instant is the SUBTITLE turning red: the layer stack does that
    //  in the same frame the switch is thrown. So the operator sees the red
    //  warning immediately and the map arrives a moment later.
    input                       emg_sel,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    // ---- F2（2026-10-14）：装载看门狗 ----
    // load_timeout_o：单拍脉冲，本次装载已超时放弃 → top 用它复位帧写通路
    // hang_code_o   ：粘滞诊断码（0=没卡过；1=写侧没写满；8|N=卡在读卡链，N=state_code）
    output                      load_timeout_o,
    output [3:0]                hang_code_o,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,

    // ======================================================================
    //  ★ Task C：音频借 SD 总线读扇区（开机一次性，之后永不再用）
    //
    //  迁移顺序（重要）：
    //      SD 初始化完成 -> 音频先把 alarm/ring 搬进 SDRAM（约 0.3 s）
    //      -> aud_hold 释放 -> 才开始扫描图片
    //  这样音频和图片【时间上完全不重叠】，图片通路零干扰。
    // ======================================================================
    input                       aud_req,        // 音频要读一个扇区（电平，保持到 aud_ack）
    input  [31:0]               aud_addr,       // 扇区号
    output                      aud_ack,        // 授权：单拍脉冲（见下方 assign）
    output [7:0]                aud_data,       // 扇区数据
    output                      aud_dvalid,     // 数据有效
    output                      aud_end,        // 本扇区结束
    input                       aud_hold        // 1 = 音频还没搬完，先别开始扫描
);

wire key_next_press;
wire key_auto_press;

wire             aud_bus_owner;   // ★ Task C：音频正占用 SD 总线
wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire [11:0]      bmp_w_parsed;      // bmp_read 解析出的原始宽（缩放器输入）
wire [11:0]      bmp_h_parsed;      // 原始高（同上）
wire             bmp_sof;           // 图像首像素（同上）
wire             scaler_ready;      // scaler -> bmp_read：扇区级反压（直通时恒 1）

// 缩放后的像素流（进 SDRAM 的永远是 640x480）在下面的 generate 块里声明：
// 使能缩放器时由 u_scaler_bilinear 驱动，直通时不用。
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
// 扩到 16 张时这根线【必须】跟着加宽到 4 位，和 bmp_read 的端口一致。
// 漏掉它会得到 vsim-3015 "Port size (4) does not match connection size (3)"，
// 而且症状很迷惑：扫描计数永远是 0，因为高位被截掉了。
wire [3:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg              auto_resume;   // 进入应急前是否正在轮播
reg [31:0]       auto_cnt;
reg [3:0]        img_found_count;
reg [3:0]        img_idx;              // 当前真正显示中的图片编号
reg [3:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg [31:0]       img_sector4;
reg [31:0]       img_sector5;
reg [31:0]       img_sector6;
reg [31:0]       img_sector7;
reg [31:0]       img_sector8;
reg [31:0]       img_sector9;
reg [31:0]       img_sector10;
reg [31:0]       img_sector11;
reg [31:0]       img_sector12;
reg [31:0]       img_sector13;
reg [31:0]       img_sector14;
reg [31:0]       img_sector15;
reg              next_req_pending_r;
// single driver for the output port
assign next_req_pending = next_req_pending_r;

// owed_cnt is the WORK half of the handshake, kept separate from the
// REQUEST half above.
//
// next_req_pending_r is cleared as soon as the FSM acknowledges with
// hold_load, so that the next key press can raise a fresh edge. That is
// correct for the request -- but the load has not happened yet at that
// point, and if the same flag carried the work too, the card would have
// nothing left to start when the FSM finally grants. Measured with the
// unsplit version: 8 of 8 fades but 0 loads and 0 swaps.
//
//     next_req_pending_r : one pulse per press   (the REQUEST)
//     owed_cnt           : counts loads still owed; mirrors want_cnt
//                          (the WORK)
// owed_cnt counts loads still owed. It MIRRORS want_cnt and must have the
// same width: a lone flag let the two drift apart, so one press could produce
// two transfers and two presses only one. Measured before this change:
//     single press -> 2 requests, 2 fades
//     press during a transition -> 1 request
// want : pictures asked for but not yet read. Replaces the
// old request-latch plus FSM-grant pair, which could not
// complete without each other.
reg [1:0]        want;
reg [1:0]        owed_cnt;

// wanted_r: a press has been seen and the picture has not been
// fetched yet. It is the durable record of the user's wish, and it is
// what lets a press during a transition survive: the request line is
// derived from it, so it does not matter that hold_load_s clears the
// request the moment it appears.
// want_cnt counts OUTSTANDING wishes. A single flag cannot do this job:
// a second press arriving while the flag is already set changes nothing, so the
// two presses collapse into one transfer. Measured in tb_key2 PHASE A.
//
//    press       -> want_cnt + 1  (saturating at 3)
//    load starts -> want_cnt - 1
//
// The request line is then "any wish outstanding AND not held off", so every
// press produces exactly one transfer even when they are pressed faster than
// the transitions complete.
reg [1:0]        want_cnt;

// req_pulse: a ONE-CYCLE pulse, raised by whatever wants the next picture --
// a key press or an auto tick. It is the request line, and it does NOT depend
// on any counter. The previous form derived the request from want_cnt, which
// meant one missed counter clear left the line stuck high (rotation could not
// be stopped) or stuck low (the load never started). Both were reported.
reg              req_pulse;

// (the wish watchdog was removed: the request is a self-clearing pulse)
reg              load_busy;
// exposed on the port list; the caller uses it to decide when to
// release hold_load (i.e. when the deferred load has actually begun)
assign load_busy_o = load_busy;
assign auto_play_state = auto_play_en;
assign img_found_o     = img_found_count;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
// F2b：缩放器是否已经把【整帧 307200 个像素】发完（装载期间锁存）
reg              scaler_done_seen;

// ==========================================================================
//  frame_written -- REMEMBER the frame-write pulse, do not require it to
//  coincide with source_done_seen.
//
//  WHY THIS IS NEEDED (reproduced in tb_load_latch.v, and it is the freeze
//  seen on the board: "切图有时候会卡死，按键没作用，复位才恢复")
//
//  load_busy is cleared in exactly ONE place, and that place used to demand
//  two UNRELATED signals on the SAME clock:
//
//      if (load_busy && source_done_seen && write_finish_pulse)
//
//  write_finish_pulse is a ONE-CLOCK XOR of a synchronised toggle:
//
//      assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
//
//  source_done_seen comes from bmp_ready, which comes from a different part
//  of the pipeline. NOTHING makes them land together. When the pulse arrives
//  first it is simply DISCARDED -- and no second pulse can come, because the
//  next frame write cannot start while load_busy is still high.
//
//  A stuck load_busy then blocks every load branch (all require !load_busy),
//  freezes auto_cnt (it only counts while !load_busy) and reaches the layer
//  FSM as the fade trigger. Result: the picture stops changing, the buttons
//  appear dead, and only a reset recovers.
//
//  Latching turns the coincidence requirement into a handshake: whichever
//  signal arrives first is REMEMBERED until the other one turns up. The
//  buffer still cannot swap mid-write, because the commit now needs BOTH
//  source_done_seen (the source has finished) AND frame_written (a frame
//  write has actually completed) -- just not on the same clock.
// ==========================================================================
reg              frame_written;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

// 同步 video_clk 域的 hold_load（Task A 新增）
//
// sd_card_bmp 跑在 100MHz，产主 hold_load 的图层跑在 25MHz，直接跨域采样会
// 出现亚稳态。hold_load 是每次转场只变一次的慢速控制信号，两级同步器足够，
// 与上面 write_finish_toggle 的处理方式一致。
reg [1:0]        hold_load_sync;
wire             hold_load_s;

// busy_stack comes from video_clk, so it needs the same treatment. It is only
// used to hold auto rotation off, so a couple of clocks of skew are harmless,
// but sampling it raw would risk metastability.
reg [1:0]        busy_stack_sync;
wire             busy_stack_s;
always @(posedge clk or posedge rst) begin
    if (rst)
        hold_load_sync <= 2'b00;
    else
        hold_load_sync <= {hold_load_sync[0], hold_load};
end
assign hold_load_s = hold_load_sync[1];

always @(posedge clk or posedge rst) begin
    if (rst)
        busy_stack_sync <= 2'b00;
    else
        busy_stack_sync <= {busy_stack_sync[0], busy_stack};
end
assign busy_stack_s = busy_stack_sync[1];

// load_req is a LEVEL, so it only needs synchronising -- no stretching, no
// edge detection, and nothing that depends on the two clocks lining up.
reg [1:0] load_req_sync;
always @(posedge clk or posedge rst) begin
    if (rst)
        load_req_sync <= 2'b00;
    else
        load_req_sync <= {load_req_sync[0], load_req};
end
wire load_req_s = load_req_sync[1];

wire auto_tick;
// *** 必须和 next_index_limited 的返回位宽一致（4 位）。 ***
//
// 这里原来是 wire [1:0]。把图数上限从 4 张扩到 15 张时，img_idx / load_idx /
// img_found_count 都扩到了 4 位，唯独漏了这一根 —— 结果函数返回的 4 位值
// 被截成 2 位，next_from_current 只能是 0~3，
// 所以轮播永远只能在 0,1,2,3 之间转，板上表现就是「工作图只能显示 4 张」。
//
// 教训：扩位宽的时候要把【所有相关信号】一起扫一遍，别只改看得见的那个。
wire [3:0] next_from_current;

// ==========================================================================
//  F2（2026-10-14）：装载看门狗 —— 把"永久卡死"降级为"自愈 + 留下证据"
//
//  为什么必须要有：load_busy 只在
//        (load_busy && source_done_seen && frame_written)
//  同时成立时才清除，而这三者都没有超时。于是一旦任何一环楔住
//  （读卡链停在中途 / 帧写没数满 307200 个像素 / 等 write_req_ack 等不到），
//  装载链就永远停在那里。上板现象（2026-10-14 观察）：
//      OSD、字幕、参数、按键全都正常 —— 唯独【底图再也换不了】。
//  因为那些功能在别的模块里，只有装载链自己卡住了。
//
//  超时取 12 s：报告 §9.6 实测一次读卡 0.45~9 s，12 s 不会误杀正常装载，
//  而"永久卡死"最迟 12 s 后自愈。
//
//  自愈动作（三件事一起做，缺一不可）：
//    1. 清 load_busy / source_done_seen / frame_written / want —— 解开装载链；
//    2. soft_rst 复位 bmp_read + 缩放器 + SD 控制器 —— 把楔住的那一环拉回初始态
//       （只清 load_busy 而不复位它们，下一次装载会立刻撞上同一个楔住状态）；
//    3. 给 top 一个 load_timeout_o 脉冲，让它复位【帧写通路】
//       （否则写侧还停在上一次的半帧计数上，下一帧会提前"写满"、画面撕裂）。
//
//  诊断码 hang_code_o（粘滞，记录第一次卡死的现场，掉电才清）：
//      0    = 从没卡死过
//      1    = 源读完 + 缩放器整帧发完，但写侧没写满 307200（写侧/FIFO 那一环）
//      2    = 源读完了，但【缩放器整帧没发完】⇒ 缩放器被中途打断或自己卡住
//             （这是 2026-10-14 诊断码 1 的后续细分：F2b 修好后应不再出现）
//      8|N  = 超时那刻 bmp_read 不空闲 ⇒ 卡在读卡链上，N 是它的 state_code
//             （10='A' 头阶段、11='b' 等 ack、12='C' 数据阶段）
//  上板时看【数码管第 5 位】就能区分是哪一类（第 6 位仍是 SD 状态码）。
// ==========================================================================
//  [C 2026-10-23] 12 s 墙钟 → 2 s 「无进展」窗口。
//  原来从 load_busy 拉高就开始数 12 s，与装载快慢无关；
//  一但音频借总线让装载变慢，就容易接近阈值。改成"只要还在写像素就清零"，
//  真正卡死（连续 2 s 一个像素都没写）才触发自愈。
localparam [31:0] LOAD_TIMEOUT = CLK_FREQ_HZ * 2;    // 2 s @100MHz = 2e8（32 位内）

reg  [31:0] load_wd_cnt;
reg         load_timeout_pulse;
reg  [3:0]  hang_code_r;
reg  [3:0]  wd_fire_cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        load_wd_cnt        <= 32'd0;
        load_timeout_pulse <= 1'b0;
        hang_code_r        <= 4'd0;
        wd_fire_cnt        <= 4'd0;
    end else if (!load_busy || bmp_data_wr_en) begin
        // 没在装载，或本拍还在往帧缓存写像素 → 有进展，计数器清零
        load_wd_cnt        <= 32'd0;
        load_timeout_pulse <= 1'b0;
    end else if (load_wd_cnt >= LOAD_TIMEOUT) begin
        load_wd_cnt        <= 32'd0;
        load_timeout_pulse <= 1'b1;
        wd_fire_cnt        <= (wd_fire_cnt == 4'hF) ? 4'hF : (wd_fire_cnt + 4'd1);
        if (hang_code_r == 4'd0)
            hang_code_r <= (!bmp_ready)        ? ({1'b1, state_code[2:0]}) :  // 卡在读卡链（8|N）
                           (!scaler_done_seen) ? 4'd2 :                       // 缩放器整帧没发完
                                                 4'd1;                        // 源+缩放器都完了，写侧没满
    end else begin
        load_wd_cnt        <= load_wd_cnt + 32'd1;
        load_timeout_pulse <= 1'b0;
    end
end

assign load_timeout_o = load_timeout_pulse;
assign hang_code_o    = hang_code_r;

// 软复位：正常复位，或者看门狗超时。bmp_read / 缩放器 / SD 控制器都用它。
wire soft_rst = rst | load_timeout_pulse;

// ---------------------------------------------------------------------------
//  ★ 2026-10-13：缩放器旁路/使能开关（一个 bit 切换，便于 A/B 对比）
//
//  背景：u_scaler_bilinear 原实现把【行缓冲 RAM 输出 → 减法 → 乘法 → 加法】
//  塞在一个 100 MHz 周期里，实测 13.389 ns > 10 ns 预算 →
//      sd_card_clk WNS −3.669 ns / 146 个失效端点 / 该域 Fmax 73 MHz → 画面全黑。
//
//  现在 scaler_bilinear.v 已做流水改造（水平/垂直插值各拆成 求差→乘法→加法 三级，
//  位宽与 lerp8 逐位等价），所以这里默认重新【使能】它。
//
//      SCALER_ENABLE = 1'b1 → 用缩放器（支持任意分辨率素材）
//      SCALER_ENABLE = 1'b0 → 直通（640×480 原样写入；仅作回退用）
//
//  ⚠️ 这个 bit 必须和 bmp_read 的 SCALER_ACTIVE 一致（下面例化时已连线）：
//      直通时若非 640×480 的图被放行，像素数不足会让 frame_write 写不满一帧，
//      而 load_busy 没有超时 → 永久卡死。
// ---------------------------------------------------------------------------
localparam SCALER_ENABLE = 1'b1;

// F2b（2026-10-14）：缩放器"整帧发完"必须参与提交握手。
//   这一位在直通模式（没有缩放器）下恒为 1，等价于旧行为。
wire        src_frame_done;

generate
if (SCALER_ENABLE) begin : g_scaler
    wire        dst_wr_en;
    wire [23:0] dst_rgb;

    scaler_bilinear #(
        .OUT_W      (640),
        .OUT_H      (480),
        .MAX_SRC_W  (2048),
        .FRAC_BITS  (16),
        .FIFO_DEPTH (512),
        .EMIT_DIV   (16)
    ) u_scaler_bilinear (
        .clk            (clk),
        .rst            (soft_rst),
        .src_valid      (bmp_data_wr_en),
        .src_rgb        (bmp_data),
        .src_sof        (bmp_sof),
        .src_w          (bmp_w_parsed),
        .src_h          (bmp_h_parsed),
        .src_ready      (scaler_ready),
        .dst_wr_en      (dst_wr_en),
        .dst_rgb        (dst_rgb),
        .dst_frame_done (src_frame_done),   // 整帧 307200 个像素发完的那一拍
        .too_wide       ()
    );

    assign write_en   = dst_wr_en;
    assign write_data = {dst_rgb[23:16], dst_rgb[15:8], dst_rgb[7:0], 8'b0};
end else begin : g_bypass
    assign write_en      = bmp_data_wr_en;
    assign write_data    = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
    assign scaler_ready  = 1'b1;     // 直通：不反压 bmp_read 的扇区读
    assign src_frame_done = 1'b1;    // 直通：没有缩放器，视为"帧已发完"
end
endgenerate
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

// ==========================================================================
//  THE MANUAL BUTTON
//
//  This used to be derived from `next_req`, which is the layer FSM's GRANT --
//  "the old picture has finished fading out, you may load now" -- and NOT the
//  button. The real `key_next` input was not read anywhere, so pressing it did
//  nothing at all, and the derivation formed a loop:
//
//      req_pulse -> next_req_pending -> FSM -> next_req (grant)
//                -> "key_next_press" -> req_pulse ...
//
//  The button now gets a proper debouncer of its own, exactly like key_auto, and
//  the request pulse is raised from that. The grant is `load_req`, a separate
//  port, and is used only as the load condition.
// ==========================================================================
key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

// ==========================================================================
//  next_index_limited -- 轮播下一张
//
//  *** 最后一张被排除，因为它是应急图。 ***
//
//  原来是一张 case 表，按 count 一档一档写死。扩到 16 张要写 16 档，
//  而且每加一档都得改这里。改成通用公式，一劳永逸：
//
//      折返点 = count - 2      （最后一张是应急图，不参与轮播）
//      cur >= count-2   ->  回到 0
//
//  用 >= 而不是 == ：如果轮播是在应急图显示着的时候被重新打开的，
//  此时 cur 就是最后一张（= count-1），>= count-2 成立，会干净地回到 0。
// ==========================================================================
function [3:0] next_index_limited;
    input [3:0] cur;
    input [3:0] count;
    begin
        if (count <= 4'd2)
            next_index_limited = 4'd0;        // 0/1/2 张：没有可转的
        else if (cur >= (count - 4'd2))
            next_index_limited = 4'd0;        // 已到最后一张可转的，回 0
        else
            next_index_limited = cur + 4'd1;
    end
endfunction

// 扇区查表。16 项全部显式列出。
//
// 这里刻意【不用寄存器数组】：这个工具链上"寄存器数组 + 变址读写"踩过大坑
// （uart_text 的 msg_mem 就是这么丢字的），所以老老实实独立寄存器 + case。
function [31:0] sector_lut;
    input [3:0] idx;
    begin
        case (idx)
            4'd0: sector_lut = img_sector0;
            4'd1: sector_lut = img_sector1;
            4'd2: sector_lut = img_sector2;
            4'd3: sector_lut = img_sector3;
            4'd4: sector_lut = img_sector4;
            4'd5: sector_lut = img_sector5;
            4'd6: sector_lut = img_sector6;
            4'd7: sector_lut = img_sector7;
            4'd8: sector_lut = img_sector8;
            4'd9: sector_lut = img_sector9;
            4'd10: sector_lut = img_sector10;
            4'd11: sector_lut = img_sector11;
            4'd12: sector_lut = img_sector12;
            4'd13: sector_lut = img_sector13;
            4'd14: sector_lut = img_sector14;
            4'd15: sector_lut = img_sector15;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

// ===========================================================================
//  [C 2026-10-24] SW3 退出紧急模式 -> 把进紧急模式之前那张图装回来
// ---------------------------------------------------------------------------
//  板上现象：轮播中拨 SW3 进紧急模式（出紧急疏散图），再拨回来 ——
//            声音正常回到普通音源，但【画面还停在紧急疏散图上，永远不走了】。
//
//  原因：
//    紧急分支把 img_idx 改成了"最后一张"（= 紧急图）。
//    退出紧急模式后没有任何东西再请求装载：
//      · want 在紧急模式下被我冻结成 0（上一轮的修法）
//      · auto_tick 还要等一个轮播周期（几秒）才来
//      · 而且在退出那一拍 want 仍然是 0
//    => 画面就一直留在紧急图上。
//
//  修法：
//    1) emg_sel 来自 video_clk 域（layer_mode），先做 2 级同步再做边沿检测；
//    2) 上升沿时记住"当前显示的是第几张"（如果已经在紧急图上就不覆盖）；
//    3) 下降沿时置 restore_pending，让普通装载分支把 idx_saved 那张装回来。
//
//  为什么不用 next_from_current：从紧急图（= 最后一张）算下一张会折返到 0，
//  直接跳回第一张，体验上像"从头开始"。装回原来那张更自然。
// ===========================================================================
//  [C 2026-10-24] ★★ 关键修正：emg_sel 必须去抖，否则恢复逻辑会抖到死 ★★
//  ---------------------------------------------------------------------------
//  上一版只做了"2 级同步 + 边沿检测"，**没有去抖**。而 emg_sel 来自 B 的
//  mode_fsm（再往上就是 C6 的裸拨码），机械抖动 1~10 ms 会被原样放大成
//  很多次 emg_rise / emg_fall。于是：
//      rise -> 记下当前图，启动紧急图装载
//      fall -> restore_pending=1
//      rise -> 清掉
//      fall -> restore_pending=1   ...
//  一连串装载请求把原来的图反复装回来，表现出来就是
//  **"拨到紧急模式，音频切了，图片却没切成紧急图"**，
//  然后装载链被拖乱 -> 2 s 无进展看门狗 soft_rst -> 板上看到"卡死重启"。
//
//  修法：对 emg_sel 本身做 20 ms 积分式去抖，之后再用去抖后的电平做
//        边沿检测和所有判断。机械抖动被彻底吃掉。
// ===========================================================================
localparam [20:0] EMG_DB_CYCLES = 21'd2000000;   // 20 ms @ 100 MHz
reg  [20:0] emg_dbcnt;
reg         emg_s1;
reg         emg_s2;      // [FIX] 2-stage synchroniser: the pin is asynchronous
reg         emg_db;      // ★ 去抖后的紧急模式电平：下面所有判断都用它
reg         emg_db_d;    // 上一拍，用来做边沿

reg  [3:0] idx_saved;
reg  [1:0] buf_saved;               // [FIX v7b] SDRAM buffer that still holds the pre-emergency frame                // 进紧急模式之前显示的那张图
reg        restore_pending;          // 1 = 还有一张"退出紧急"的恢复装载没做

wire emg_rise =  emg_db & ~emg_db_d;
wire emg_fall = ~emg_db &  emg_db_d;

// [FIX 2026-10-25b] Never restore the last (emergency) picture, even if a
// glitch or reset left idx_saved at an invalid value.
wire [3:0] saved_normal_idx =
    ((img_found_count > 4'd1) &&
     (idx_saved < (img_found_count - 4'd1))) ? idx_saved : 4'd0;

// 本次装载的目标下标：恢复装载优先，否则走轮播的"下一张"。
// 单独抽出来是为了只让 sector_lut 被例化一次。
wire [3:0] load_target = restore_pending ? saved_normal_idx : next_from_current;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        emg_s1     <= 1'b0;
        emg_s2     <= 1'b0;
        emg_dbcnt  <= 21'd0;
        emg_db     <= 1'b0;
        emg_db_d   <= 1'b0;
    end else begin
        emg_s1 <= emg_sel;                     // stage 1
        emg_s2 <= emg_s1;                      // stage 2
        if (emg_s2 == emg_db)
            emg_dbcnt <= 21'd0;                // 输入已经稳定
        else if (emg_dbcnt >= EMG_DB_CYCLES) begin
            emg_db     <= emg_s2;              // 稳定够 20 ms 才认账
            emg_dbcnt  <= 21'd0;
        end else
            emg_dbcnt <= emg_dbcnt + 21'd1;
        emg_db_d <= emg_db;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_resume           <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 4'd0;
        img_idx               <= 4'd0;
        load_idx              <= 4'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        req_pulse             <= 1'b0;
        load_busy             <= 1'b0;
        frame_written <= 1'b0;
        source_done_seen      <= 1'b0;
        scaler_done_seen      <= 1'b0;
        display_valid         <= 1'b0;
        idx_saved             <= 4'd0;
        buf_saved             <= 2'd0;
        restore_pending       <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;

        // ==================================================================
        //  THE REQUEST PULSE
        //
        //  Cleared here on EVERY clock, with no condition. Statements later in
        //  this block therefore win, which is how an event raises it. One
        //  unconditional clear is what makes it impossible for the request to
        //  be left stuck high (rotation that will not stop) or stuck low (a
        //  load that never starts).
        // ==================================================================
        req_pulse <= 1'b0;

        // ---- registered copy of the request, which is the output ----
        // next_req_pending is what the layer FSM sees. It is one clock behind
        // req_pulse, which is what makes the pulse exactly one clock wide at the
        // port: req_pulse is high for a cycle, and next_req_pending_r takes that
        // value, so it too is high for a cycle and then goes low again.
        next_req_pending_r <= req_pulse;

        if (!sd_init_done) begin
            // no card yet: forget everything and wait
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_resume           <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 4'd0;
            img_idx               <= 4'd0;
            load_idx              <= 4'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            img_sector4           <= 32'd0;
            img_sector5           <= 32'd0;
            img_sector6           <= 32'd0;
            img_sector7           <= 32'd0;
            img_sector8           <= 32'd0;
            img_sector9           <= 32'd0;
            img_sector10          <= 32'd0;
            img_sector11          <= 32'd0;
            img_sector12          <= 32'd0;
            img_sector13          <= 32'd0;
            img_sector14          <= 32'd0;
            img_sector15          <= 32'd0;
            load_busy             <= 1'b0;
            want                  <= 2'd0;
            restore_pending       <= 1'b0;
            frame_written <= 1'b0;
            source_done_seen      <= 1'b0;
            scaler_done_seen      <= 1'b0;
            display_valid         <= 1'b0;
        end else begin
            // --------------------------------------------------------------
            //  scan bookkeeping: latch the first four sectors and count them
            // --------------------------------------------------------------
            if (scan_found_valid) begin
                // 扫描结果按发现顺序写进扇区表。16 项全部显式列出。
                case (img_found_count)
                    4'd0 : img_sector0  <= scan_found_sector;
                    4'd1 : img_sector1  <= scan_found_sector;
                    4'd2 : img_sector2  <= scan_found_sector;
                    4'd3 : img_sector3  <= scan_found_sector;
                    4'd4 : img_sector4  <= scan_found_sector;
                    4'd5 : img_sector5  <= scan_found_sector;
                    4'd6 : img_sector6  <= scan_found_sector;
                    4'd7 : img_sector7  <= scan_found_sector;
                    4'd8 : img_sector8  <= scan_found_sector;
                    4'd9 : img_sector9  <= scan_found_sector;
                    4'd10: img_sector10 <= scan_found_sector;
                    4'd11: img_sector11 <= scan_found_sector;
                    4'd12: img_sector12 <= scan_found_sector;
                    4'd13: img_sector13 <= scan_found_sector;
                    4'd14: img_sector14 <= scan_found_sector;
                    4'd15: img_sector15 <= scan_found_sector;
                    default: ;
                endcase
                if (img_found_count < 4'd15)
                    img_found_count <= img_found_count + 4'd1;
            end

            // ==============================================================
            //  ROTATION TOGGLE -- one key, one flip-flop.
            //
            //  Ungated on purpose. Every earlier version gated this on
            //  scan_done or the picture count, and whenever one of those was
            //  not what had been assumed the button simply looked dead.
            // ==============================================================
            // KEY3 is ignored while emergency is active; the previous
            // rotation state is restored automatically on exit.
            if (key_auto_press && !emg_db && !emg_s2) begin
                auto_play_en <= ~auto_play_en;
                auto_cnt     <= 32'd0;
            end

            // ==============================================================
            //  ROTATION TIMER -- one tick per second while rotation is on.
            //  Held at zero if a tick could not be served, so ticks cannot
            //  pile up behind a load.
            // ==============================================================
            if (auto_play_en && !load_busy && !emg_db && !emg_s2) begin
                if (auto_tick)
                    auto_cnt <= 32'd0;
                else
                    auto_cnt <= auto_cnt + 32'd1;
            end else begin
                auto_cnt <= 32'd0;
            end

            // ==============================================================
            //  WANT -- how many more pictures have been asked for.
            //
            //  Raised by the two things that can ask: the manual key and the
            //  rotation tick. Saturating, so holding a key down cannot queue
            //  dozens of loads.
            //
            //  THIS IS THE ONLY PLACE A PICTURE CHANGE IS REQUESTED, and the
            //  card acts on it by itself -- there is no grant signal from the
            //  layer FSM anywhere in this path. That is the whole point: the
            //  card can always advance, whatever the animation is doing.
            // ==============================================================
            //  [C 2026-10-23] ★ 紧急模式下必须冻结轮播队列 ★
            //  原来的 bug（板上现象：拨上 SW3 后音频对了，但图片先显示紧急图，
            //  紧接着【紧急图和轮播图来回抢着切换】）：
            //    · auto_play_en 在紧急模式下仍然为 1，auto_tick 每几秒照常来
            //    · 这里没有 emg_sel 保护，want 照样 +1
            //    · 下面的紧急分支因为 img_idx 已经等于 emg_idx 而不成立
            //    · 于是落到普通分支，把轮播图装载进来 → 离开紧急图
            //    · 下一拍 img_idx != emg_idx → 紧急分支又把紧急图装回来
            //    ⇒ 两张图每个装载周期（约 0.5 s）互相顶一次，看起来就是"抢"
            //  修法：紧急模式下 want 恒 0（不排队、也把之前攒的清掉）。
            //        退出紧急模式后 auto_tick / KEY2 照常工作。
            //  [C 2026-10-24] 另外加"退出紧急模式要把原来那张图装回来"，
            //  见文件上方 emg_db 那段注释。
            if (emg_rise) begin
                // 刚进紧急模式：记住现在显示的是第几张普通图，并停掉轮播。
                if ((img_found_count > 4'd1)
                    && (img_idx != (img_found_count - 4'd1)))
                    idx_saved <= img_idx;
                buf_saved       <= disp_buf_idx;   // [FIX v7b] remember the buffer to come back to
                auto_resume     <= auto_play_en;
                auto_play_en    <= 1'b0;
                auto_cnt        <= 32'd0;
                want            <= 2'd0;
                restore_pending <= 1'b0;
            end else if (emg_db) begin
                auto_play_en    <= 1'b0;
                auto_cnt        <= 32'd0;
                want            <= 2'd0;
                restore_pending <= 1'b0;
            end else if (emg_fall) begin
                // 刚退出紧急模式：强制装回原来的普通图；轮播状态恢复。
                auto_play_en    <= auto_resume;
                auto_cnt        <= 32'd0;
                want            <= 2'd0;
                restore_pending <= 1'b0;   // [FIX v7b] the 'reload on exit' path is retired
                // [FIX v7b] NO SD RELOAD WHEN LEAVING EMERGENCY.
                // The pre-emergency frame is still in the OTHER SDRAM buffer:
                // the evacuation picture was loaded into the flipped one. So
                // pointing the read buffer back shows it again in one clock --
                // no card read, no black screen. Moving disp_buf_idx is also
                // what makes the layer FSM fade it in.
                if (display_valid
                    && (buf_saved != disp_buf_idx)
                    && (img_found_count > 4'd1)
                    && (idx_saved < (img_found_count - 4'd1))) begin
                    disp_buf_idx <= buf_saved;
                    img_idx      <= idx_saved;
                end
            end else if ((key_next_press || (auto_play_en && auto_tick))
                && !emg_s2 && (want != 2'd3)) begin
                want      <= want + 2'd1;
                // Also tell the layer FSM, so it fades the new picture in.
                // That is the only thing the FSM is asked to do now: it is an
                // animation, not a gate. Whether the picture changes does not
                // depend on the FSM at all, which is what makes a stuck
                // animation harmless instead of fatal.
                req_pulse <= 1'b1;
            end

            // ==============================================================
            //  THE FIRST PICTURE -- loaded once, as soon as the scan is done.
            // ==============================================================
            if (scan_done && !first_image_committed && bmp_ready && !load_busy
                && (img_found_count != 4'd0)) begin
                load_idx         <= 4'd0;
                load_sector      <= img_sector0;
                pending_buf_idx  <= 2'd0;
                write_buf_idx    <= 2'd0;
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                frame_written <= 1'b0;
                source_done_seen <= 1'b0;
                scaler_done_seen  <= 1'b0;
            end

            // ==============================================================
            //  EVERY LATER PICTURE -- started as soon as one is wanted and the
            //  card is free. No FSM involved: the only conditions are "a
            //  picture has been asked for", "the card is idle" and "there is
            //  somewhere to go".
            // ==============================================================
            // ==============================================================
            //  EMERGENCY: JUMP STRAIGHT TO THE LAST PICTURE.
            //
            //  Placed BEFORE the normal branch so it takes priority: while
            //  the emergency switch is on, a queued rotation request must
            //  not win the race and load a normal picture instead.
            //
            //  NOT INSTANT, AND THAT IS THE SD CARD, NOT THE LOGIC. The BMP
            //  still has to be read: 640*480*24 = 900 KB at 25 MHz SPI is
            //  ~0.29 s of data plus FAT and sector overhead, so about half
            //  a second. The subtitle turning red is what happens in the
            //  same frame; the map follows a moment later.
            //
            //  `img_idx != emg_idx` stops it re-loading the same picture
            //  every clock while the switch stays on.
            // ==============================================================
            else if (emg_db && !load_busy && scan_done
                     && (img_found_count > 4'd1)
                     && (img_idx != (img_found_count - 4'd1))) begin
                load_idx         <= img_found_count - 3'd1;
                load_sector      <= sector_lut(img_found_count - 4'd1);
                pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                frame_written    <= 1'b0;
                source_done_seen <= 1'b0;
                scaler_done_seen  <= 1'b0;
                // the queued wishes are dropped: in an emergency a pending
                // 'next picture' is meaningless and would load the wrong one
                want             <= 2'd0;
            end
            // [C 2026-10-23] 再加一道 !emg_sel：即使 want 里还残留着历史值，
            //                紧急模式下也绝不允许装载普通轮播图。
            // [C 2026-10-24] restore_pending 优先：退出紧急模式时先装回
            //                【进紧急模式之前那张】，而不是 next_from_current
            //                （从紧急图算下一张会折返到 0，看起来像从头开始）。
            else if (!emg_db && !emg_s2
                     && (want != 2'd0)
                     && !load_busy && scan_done
                     && (img_found_count > 4'd1)) begin
                // [C 2026-10-24] ★ 只调用一次 sector_lut ★
                //   原来写的是
                //       load_sector <= restore_pending ? sector_lut(idx_saved)
                //                                       : sector_lut(next_from_current);
                //   综合出来是【两个 32bit 16:1 选择器】，12 级逻辑、fanout 76，
                //   post-route 直接 −2.288 ns / 120 个违例端点（sd_card_clk 100MHz）。
                //   先算出目标下标、再喂一次 sector_lut，逻辑量差不多减半。
                load_idx         <= load_target;
                load_sector      <= sector_lut(load_target);
                pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                frame_written <= 1'b0;
                source_done_seen <= 1'b0;
                scaler_done_seen  <= 1'b0;
                if (restore_pending) begin
                    restore_pending <= 1'b0;      // 恢复装载只做一次
                end else begin
                    want            <= want - 2'd1;
                end
            end

            // --------------------------------------------------------------
            //  bmp_read has finished feeding the FIFO. The display must NOT
            //  change yet -- the whole frame has to be written first.
            // --------------------------------------------------------------
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // --------------------------------------------------------------
            //  F2b（2026-10-14）：还要等【缩放器把整帧 307200 个像素发完】。
            //
            //  为什么这一条是必需的：source_done_seen 来自 bmp_read 空闲，
            //  而 bmp_read 只负责发【源】像素 —— 放大时（如 320x240）它把最后
            //  一行源像素发完时，缩放器还有约两行输出（≈1280 个像素）没发。
            //  只等 source_done_seen + frame_written 就提交的话，装载可以在
            //  缩放器【还在发】的时候就翻页；下一次装载的 src_sof 会把缩放器
            //  从帧中间打断 ⇒ 那一帧少发像素 ⇒ 写侧永远凑不满 307200
            //  ⇒ load_busy 永久卡死（上板诊断码 1 就是这个现象）。
            //
            //  dst_frame_done 是缩放器走到 S_DONE（整帧发完）的那一拍脉冲；
            //  直通模式（SCALER_ENABLE=0）下它恒为 1，等价于旧行为。
            // --------------------------------------------------------------
            if (load_busy && src_frame_done)
                scaler_done_seen <= 1'b1;

            // --------------------------------------------------------------
            //  REMEMBER the frame-write pulse.
            //
            //  It is one clock wide and it may arrive BEFORE source_done_seen
            //  (that ordering is what froze the board). Latching it here means
            //  the commit below no longer needs the two to land together --
            //  whichever arrives first waits for the other.
            //
            //  Cleared at every load start and on commit, so a pulse from a
            //  previous load can never commit the next one.
            // --------------------------------------------------------------
            if (write_finish_pulse)
                frame_written <= 1'b1;

            // --------------------------------------------------------------
            //  commit the new picture.
            //
            //  三个条件缺一不可：
            //    source_done_seen —— bmp_read 把源像素发完了
            //    scaler_done_seen —— 缩放器把【整帧 307200】发完了（F2b，2026-10-14 加）
            //    frame_written    —— 写侧把这 307200 个写进 SDRAM 了
            //  三者分别锁存，谁先到就等谁 —— 这样提交一定发生在
            //  "这一帧的像素全部产生并且全部落盘"之后，装载链不再可能
            //  在帧中间翻页（那正是上板诊断码 1 的成因）。
            // --------------------------------------------------------------
            if (load_busy && source_done_seen && scaler_done_seen && frame_written) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                scaler_done_seen      <= 1'b0;
                frame_written         <= 1'b0;
                disp_buf_idx          <= pending_buf_idx;
                img_idx               <= load_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
            end

            // ==============================================================
            //  F2：看门狗超时 —— 放弃这一次装载，解开装载链
            //
            //  放在提交判断之后，所以它优先级最高（同一拍里以这句为准）。
            //  这里只负责"解锁"；把楔住的那一环拉回初始态由 soft_rst
            //  （bmp_read / 缩放器 / sd_card_top）和 top 的帧写复位负责。
            //  同时清 want：卡死状态下排队的请求已经失去意义，留着只会在
            //  解锁瞬间又冲进同一个坏状态。
            //
            //  注意【不动 display_valid / disp_buf_idx】：屏幕上那张图是好的，
            //  自愈过程不该让它黑掉或错位。
            // ==============================================================
            if (load_timeout_pulse) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                scaler_done_seen  <= 1'b0;
                frame_written    <= 1'b0;
                want             <= 2'd0;
            end
        end

        // ==================================================================
        //  power-on: find the pictures on the card
        // ==================================================================
        // ★ Task C：aud_hold=1 时先不扫描 —— 让音频把数据搬完
        if (!scan_kicked && bmp_ready && !aud_hold) begin
            scan_start_pulse      <= 1'b1;
            scan_kicked           <= 1'b1;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 4'd0;
            img_idx               <= 4'd0;
            load_idx              <= 4'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            req_pulse             <= 1'b0;
            display_valid         <= 1'b0;
            load_busy             <= 1'b0;
            frame_written <= 1'b0;
            source_done_seen      <= 1'b0;
            scaler_done_seen      <= 1'b0;
        end
    end
end

// ======================================================================
//  ★ Task C：SD 总线二选一（音频优先，因为图片还没开始）
//
//  授权条件：音频要读 && bmp 处于 IDLE && bmp 当前没在发扇区请求
//            && SD 初始化已完成
//  一旦授权就保持到本扇区 aud_end（audio_sd_loader 内部自己保持 req）。
// ======================================================================
wire        bmp_sd_sec_read;
wire [31:0] bmp_sd_sec_read_addr;

// ★ 授权必须【原子化】：一旦借出，就保持到本扇区真正读完（sd_sec_read_end），
//   绝不在事务中途收回。
//
//   为什么不能是组合授权（这是板上真实踩到的死锁）：
//     组合授权时，1.34 s 闸门放行 -> bmp_read 离开 IDLE -> bmp_ready 掉到 0
//     -> aud_bus_owner 立刻变 0 -> sd_sec_read_addr 在事务进行中被切到 bmp 的地址
//     -> sd_card_top 状态机被撕裂 -> loader 和 bmp_read 都等不到 sd_sec_read_end
//     -> 两边一起死锁：屏幕上没图片，LED3（= aud_load_busy）一直常亮。
//
//   现在的规则：
//     · 没占用时：满足"音频要读 + bmp 空闲 + 卡就绪"就占用
//     · 已占用时：只有 sd_sec_read_end（本扇区结束）才归还；
//                 另加一个超时，防止 SD 卡异常时永久占着总线。
// ★★★ 音频借 SD 总线的总开关 ★★★
//   1 = 允许音频借（流式播放需要）
//   0 = 完全禁止（用来隔离：图片通路到底有没有被音频干扰）
localparam AUD_LEND_EN = 1'b1;

// SD 控制器状态机空闲指示（1 = 停在 S_WAIT_READ_WRITE，没有任何事务在途）
wire       sd_fsm_idle;

reg        aud_owner_r;
reg        aud_owner_d;   // previous owner flag, used to make a one-cycle grant pulse
reg [25:0] aud_owner_wd;                 // [FIX v7d] 26bit = 671 ms @100MHz (was 2.6 ms)                 // 约 2.6 ms @100MHz 的超时保护
wire       aud_owner_timeout = (aud_owner_wd == 26'h3FF_FFFF);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        aud_owner_r  <= 1'b0;
        aud_owner_d  <= 1'b0;
        aud_owner_wd <= 26'd0;
    end else if (!aud_owner_r) begin
        aud_owner_d  <= 1'b0;
        aud_owner_wd <= 26'd0;
        // ★ 只在【音频加载窗口】内授权（aud_hold=1）。
        //   闸门一打开（音频搬完 / 超时），音频就彻底让路，
        //   保证扫描一定能拿到总线 —— 图片优先是硬要求。
        // ★ 流式方案：不再用 aud_hold 门控。音频随时可以借，但
        //   必须等到 bmp_read 空闲、且当前没有图片装载在跑，
        //   而且一旦借出去就保持到本扇区读完（原子授权）。
        // ★★ 音频借总线的条件 ★★
        //
        //  保留：scan_done  -> 开机扫描（约 2.5 s）期间音频绝对不许打扰
        //  保留：!bmp_sd_sec_read -> 图片没有在途事务时才借（安全）
        //
        //  ★ 去掉了 bmp_ready 和 !load_busy ★
        //    原来这两个条件意味着"图片装载的 0.3 秒里音频一个扇区都拿不到"
        //    -> 64 ms 缓冲撑不住 -> 板上就是"卡卡卡"的断续声。
        //    实测证据：按 KEY3 让装载停一下，音频立刻变正常。
        //    现在允许音频插进【图片扇区之间的间隙】，每次只借一个扇区
        //    （约 200 us）就归还，图片装载略微变慢，但音频不断。
        // ★ 撤回"插队"方案：实测插队会破坏 SD 控制器的在途事务（板上变成全噪声）。
        //   回到"图片装载期间音频让路"，这样音频数据是正确的（只是会断续）。
        //   断续的根因是缓冲只有 64 ms，而一次图片装载约 300 ms —— 需要加深缓冲。
        // Interleave one audio sector between BMP sector transactions.
        //
        // ★ [C 2026-10-23] 加了 sd_fsm_idle：
        //   bmp_sd_sec_read 是【电平】，正常情况下它高就代表事务在途；
        //   但在 backpressure 下它会短暂拉低，此时 SD 状态机仍在 S_READ
        //   里读 bmp 的扇区。如果这拍把总线交给音频，bmp 的 data/end 会被
        //   mask 掉，整个扇区丢失 → bmp_read 扇区计数错位 → 装载永远完不成
        //   → 12 s 看门狗 soft_rst → 板上"切 2~3 张就卡死重启、C0x→C00"。
        //   加上 sd_fsm_idle 后：只有在 SD 状态机【确实停在 S_WAIT】、
        //   且 bmp 没在请求时才授权 —— 不可能有在途事务被抢。
        if (AUD_LEND_EN && aud_req && scan_done
            && !bmp_sd_sec_read && sd_fsm_idle && sd_init_done)
            aud_owner_r <= 1'b1;
    end else begin
        aud_owner_d <= aud_owner_r;
        // [FIX v7d] THE BUS MAY ONLY BE HANDED BACK AT A TRANSACTION BOUNDARY.
        // The old form dropped the owner purely on the 2.6 ms timeout. When an
        // SD sector took longer than that (and switching SW3 makes the card do
        // many short reads, so it happens), the owner vanished in the middle of
        // S_READ: the address mux jumped back to bmp_read while sd_card_top was
        // still streaming the AUDIO sector, so bmp_read swallowed a stray sector
        // and its byte stream shifted by 512. That corrupts the BMP parse, the
        // load never finishes, and the picture chain wedges -> the board hangs.
        // Now the timeout can only release the bus once the SD FSM is back in
        // S_WAIT (sd_fsm_idle), i.e. never with a transaction in flight, and it
        // is 671 ms so a slow sector cannot trigger it at all.
        if (sd_sec_read_end || (aud_owner_timeout && sd_fsm_idle))
            aud_owner_r <= 1'b0;
        else
            aud_owner_wd <= aud_owner_wd + 26'd1;
    end
end

// ===========================================================================
//  [C 2026-10-23] 总线移交（修 "切几张图后卡死重启"）
// ---------------------------------------------------------------------------
//  旧版有两个漏洞，都会让 bmp_read 的扇区计数错位 → 装载永远完不成 →
//  12 s 装载看门狗 soft_rst → 数码管 C0x 掉到 C00、画面回到第一张
//  （板子上看就是"切 2~3 张就卡死重启"）：
//
//  漏洞①  授权条件只看 !bmp_sd_sec_read。
//          bmp_read 的 sd_sec_read 是【电平】，但在 backpressure 下会
//          瞬间拉低，此时 SD 状态机其实还在 S_READ 里读 bmp 的扇区。
//          音频在这拍拿到授权后，bmp 的 data/end 被 mask 掉，那一整个
//          扇区就丢了。→ 必须再加 sd_fsm_idle（状态机真的停在 S_WAIT）。
//
//  漏洞②  归还那一拍没有立刻交还地址。
//          旧写法用 aud_owner_r 直接做 mux，而 aud_owner_r 是在
//          sd_sec_read_end 的【下一拍】才清零。于是状态机从 S_READ_END
//          进 S_WAIT 时看到的是"音频还要读"，就会用音频地址多发一笔读；
//          等这笔读结束，owner 已经撤销，它的 data/end 又回到 bmp_read
//          头上 —— bmp_read 白吃一个扇区。
//          → mux 必须用 aud_bus_sel = aud_owner_r & ~sd_sec_read_end。
// ===========================================================================
wire aud_bus_sel = aud_owner_r & ~sd_sec_read_end;

assign aud_bus_owner     = aud_owner_r;
// The stream module uses this as a one-cycle grant pulse.  Holding ack high
// for the whole sector would make a source-switch request look granted before
// the new sector actually starts.
assign aud_ack           = aud_owner_r & ~aud_owner_d;
assign aud_data          = sd_sec_read_data;
assign aud_dvalid        = aud_bus_owner & sd_sec_read_data_valid;
assign aud_end           = aud_bus_owner & sd_sec_read_end;

// 送进 sd_card_top 的请求：音频优先，否则回落到 bmp_read
// ★ 用 aud_bus_sel（不是 aud_bus_owner）：见上面"漏洞②"的说明。
assign sd_sec_read       = aud_bus_sel ? 1'b1       : bmp_sd_sec_read;
assign sd_sec_read_addr  = aud_bus_sel ? aud_addr   : bmp_sd_sec_read_addr;

bmp_read #(
    .SCALER_ACTIVE  (SCALER_ENABLE)     // 与上面的缩放器开关保持一致
) bmp_read_m0(
    .clk                    (clk),
    .rst                    (soft_rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sd_sec_read),
    .sd_sec_read_addr       (bmp_sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    // ★ 音频占用总线时，把回读的 data_valid / end 对 bmp_read 屏蔽，
    //   否则它会把这些当成自己的扇区数据，扫描/装载就乱了。
    .sd_sec_read_data_valid (aud_bus_owner ? 1'b0 : sd_sec_read_data_valid),
    .sd_sec_read_end        (aud_bus_owner ? 1'b0 : sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .src_ready              (scaler_ready),
    .bmp_w_o                (bmp_w_parsed),
    .bmp_h_o                (bmp_h_parsed),
    .bmp_sof_o              (bmp_sof)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (soft_rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_fsm_idle            (sd_fsm_idle),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
