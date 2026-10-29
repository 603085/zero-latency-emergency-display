// ============================================================================
//  video_layers_top.v -- integration wrapper for the Task A layer stack
//
//  WHY A WRAPPER INSTEAD OF EDITING top_tf_hdmi_audio.v IN PLACE
//  ------------------------------------------------------------
//  The example's top module belongs to everybody on the team. Keeping the
//  whole layer stack behind one instantiation means:
//    - the insertion is three lines in top, not eighty
//    - it can be removed again by deleting those three lines
//    - person B and person C each add their piece by wiring one port
//
//  WHAT IT CONTAINS
//  ----------------
//    pixel coordinate counters   (derived from de_r, so they are pixel aligned)
//    text_layer                  (subtitle band + live counter)
//    scroll_timer                (message fill + scrolling offset)
//    mode_fsm                    (normal / emergency / self-test)
//    layer_mixer                 (the actual compositing)
//
//  WHAT IT DOES NOT CONTAIN
//  ------------------------
//    the 7-segment timestamp display: that needs seg_scan, which the example
//    already drives from sd_card_bmp's state_code. Adding a second driver to
//    the same pins would conflict, so the counter digits are exported on
//    time_digit_* for whoever re-wires the 7-segment display (person B owns
//    the pin constraints). Nothing here touches seg_sel / seg_data.
// ============================================================================

`timescale 1ns / 1ps

module video_layers_top #(
    parameter H_ACTIVE   = 640,
    parameter V_ACTIVE   = 480,

    // subtitle geometry
    parameter ROW1_Y     = 424,
    parameter ROW2_Y     = 448,
    parameter CHAR_W     = 10,
    parameter TEXT_MAX   = 32,
    parameter MSG_LEN    = 16,

    // Counter row is always 10 cells: "TIME " plus 5 BCD digits.
    parameter CNT_LEN    = 10,

    // Horizontal origins. Each row is centred on its own width:
    //     BASE1_X = (H_ACTIVE - MSG_LEN*CHAR_W) / 2
    //     BASE2_X = (H_ACTIVE - CNT_LEN*CHAR_W) / 2
    // These replaced a single MARGIN_X, which left the message against the
    // left edge instead of centred.
    parameter BASE1_X    = (H_ACTIVE - MSG_LEN * CHAR_W) / 2,
    parameter BASE2_X    = (H_ACTIVE - CNT_LEN * CHAR_W) / 2,

    // scroll speed: video clocks per pixel step (25 MHz -> 5e6 = 0.2 s)
    parameter SCROLL_TICKS = 32'd5_000_000,

    // How long the subtitle RESTS CENTRED before the bounce begins, in video
    // clocks. 25 MHz x 2.5 s = 62_500_000.
    //
    // The bounce starts at the right wall, which is entirely off screen, and
    // at 0.2 s per pixel the line needs about 32 s to become visible. Without
    // this hold the subtitle is invisible for the first half minute after
    // power-up and looks broken. Exposed as a parameter so a simulation can
    // shorten it without changing production behaviour.
    parameter [31:0] HOLD_TICKS = 32'd62_500_000,

    // audio spectrum area (person C fills these in)
    parameter SPEC_X     = 192,
    parameter SPEC_Y     = 360,
    parameter SPEC_W     = 256,
    parameter SPEC_H     = 48,

    // Power-on self test length, in video clocks. The hardware default is
    // 25 MHz x 1.5 s. A testbench can shorten it.
    parameter [25:0] SELFTEST_CYCLES = 26'd37_500_000,

    // Emergency debounce window: 2^20 / 25 MHz = 42 ms on hardware.
    // Exposed so a simulation can shorten it.
    parameter DEBOUNCE_BITS = 20,

    // Fade-through-black length, in video clocks: 25 MHz x 0.3 s = 7_500_000.
    // 0.1 s was too short to read as a transition on the board -- it looked
    // like a plain cut.
    parameter [31:0] FADE_LEN   = 33_554_432,
    parameter        FADE_SHIFT = 17,

    // Radial iris: how long the aperture takes to open, in video clocks.
    // 25 MHz x 1.0 s = 25_000_000. The metric is tight near the centre, so a
    // short ramp finishes before the eye catches the wipe.
    parameter [31:0] IRIS_LEN   = 25_000_000
)(
    input  wire        clk,                 // video_clk, 25 MHz
    input  wire        rst,                 // rst_all

    // ---------------- video timing from video_delay ----------------
    input  wire        de,                  // de_r
    input  wire [23:0] vout_data_raw,       // picture from SDRAM
    input  wire        display_valid,       // example's first-image flag

    // ---------------- physical inputs for the FSM ----------------
    input  wire        sw_emergency,        // slide switch, 1 = emergency
    input  wire        key_next,            // the "next image" key
    input  wire        key_auto,
    input  wire        key_mode,

    // ---------------- serial text entry ----------------
    // The RX pin of the operator's serial link, idle HIGH. Leaving this port
    // undriven in a testbench is harmless: the receiver's synchroniser then
    // never sees the low-going start edge it needs, so it simply stays idle
    // and the message memory keeps its power-on contents.
    input  wire        uart_rx_pin,

    // ---------------- two-phase transition handshake ----------------
    // hold_load gates the START of the next image load in sd_card_bmp. The
    // sequence it enables:
    //     1. the user asks for the next picture
    //     2. the CURRENT picture fades smoothly to black
    //        (hold_load = 1, so the load is not allowed to begin yet)
    //     3. the screen is black -> hold_load released, the load runs
    //     4. the buffer swaps while nothing is visible
    //     5. the NEW picture fades smoothly up
    //
    // Without step 2/3 the swap happens whenever the card read finishes, which
    // has nothing to do with when the key was pressed, and the only thing left
    // to animate is the new picture arriving -- a hard cut with a fade stuck
    // on the end, which is exactly what was reported from the board.
    output wire        hold_load,
    input  wire        load_busy,           // the deferred load has begun

    // next_req_pending : sd_card_bmp WANTS to load the next picture. Raised by
    //                    the key press or by the auto-rotation tick, and it
    //                    stays high until the load actually starts.
    // next_grant       : the load MAY begin. Held high for the whole of
    //                    WAIT_SWAP, i.e. from "the screen is black" until the
    //                    buffer has swapped. A level, not a pulse, because
    //                    sd_card_bmp samples it through a synchroniser on its
    //                    own (100 MHz) clock.
    input  wire        next_req_pending,
    output wire        next_grant,

    // ts_state_o : the transition FSM's state, for the 7-segment
    // diagnostic. 0 = idle, 1 = fade out, 2 = wait swap, 3 = fade in.
    output wire [1:0]  ts_state_o,
    // load_busy_o : the same load_busy the fade-out is triggered by,
    // brought out so the board can show whether the card is reading.
    // Without it, a fade-out that never ends and a card that never
    // finishes looking identical.
    output wire        load_busy_o,
    // load_req: level, high for the whole of WAIT_SWAP -- tells sd_card_bmp to
    // start the load. A level rather than a pulse so it cannot be missed.
    output wire        load_req,

    // busy_stack: a transition is in progress. Passed straight out to
    // sd_card_bmp so auto rotation waits for the previous transition to finish.
    output wire        busy_stack,

    // ---------------- health inputs ----------------
    input  wire        sd_card_ok,
    input  wire        audio_link_ok,
    input  wire [3:0]  sd_state_code,

    // Which SDRAM frame buffer the display is currently reading.
    //
    // THIS IS THE FADE TRIGGER. The FSM's own next_image pulse fires when the
    // user asks for the next picture, which is BEFORE the card has been read
    // and BEFORE the buffer is swapped. Starting the fade there made the ramp
    // finish while the old picture was still on screen, so the new picture
    // simply appeared -- a hard flash with no visible fade.
    //
    // sd_card_bmp commits the new picture with
    //     disp_buf_idx <= pending_buf_idx
    // so a CHANGE in this signal is the real moment the screen changes. That
    // is where the fade must start.
    input  wire [1:0]  disp_buf_idx,

    // ---------------- display parameters (person B drives these) ----------------
    input  wire [7:0]  param_brightness,    // 128 = neutral
    input  wire [7:0]  param_contrast,      // 128 = neutral
    input  wire [1:0]  param_transition,    // 0 = hard, 1 = fade

    // ---------------- colour preset (person B's parameter item 4) ----------------
    // 0 = standard  1 = emergency  2 = night  3 = high contrast
    input  wire [1:0]  param_preset,        // value chosen by the user
    input  wire        param_preset_en,     // 1 = the user actually selected one

    // ---------------- scaled picture (person B) ----------------
    input  wire [23:0] scale_rgb,           // leave tied to vout_data_raw until ready

    // ---------------- audio spectrum (person C) ----------------
    input  wire [23:0] spec_rgb,
    input  wire [7:0]  spec_alpha,

    // ---------------- parameter OSD bar (person B) ----------------
    // Sits ABOVE the brightness/contrast stage on purpose: the OSD shows the
    // very parameters it would otherwise be darkened by.
    input  wire [23:0] osd_rgb,
    input  wire [7:0]  osd_alpha,

    // ---------------- blended pixel to the HDMI path ----------------
    output wire [23:0] vout_data,

    // ---------------- pixel coordinates, exported for the OSD ------------
    // The OSD bar (person B) must be drawn at the SAME pixel the mixer is
    // compositing, and layer_mixer's OSD stage is a plain multiplexer with no
    // spatial gating of its own -- all of the geometry lives in osd_bar. These
    // two counters are derived from de right here, so exporting them is the
    // only way to guarantee one coordinate source. Do not count x/y again in
    // top: a second counter is a second truth, and the overlay would drift the
    // moment either one changed.
    output wire [9:0]  pix_x,
    output wire [9:0]  pix_y,

    // ---------------- status / debug outputs ----------------
    output wire [1:0]  mode,
    output wire        mode_changed,        // wire to led4 for the latency test
    output wire        fill_done,
    output wire [3:0]  time_d0,             // BCD counter digits, for the
    output wire [3:0]  time_d1,             // 7-segment display (unused here)
    output wire [3:0]  time_d2,
    output wire [3:0]  time_d3,

    // ---------------- on-chip latency measurement ----------------
    // latency_cycles x 40 ns = delay from the debounced key to the mode
    // register update. No oscilloscope required.
    output wire [31:0] latency_cycles,
    output wire        latency_valid,
    output wire        latency_measurement,
    output wire        latency_one_frame,    // 1 if it exceeded one frame

    // ---------------- colour preset status ----------------
    output wire [1:0]  cur_preset,           // preset actually in effect
    output wire [7:0]  preset_brightness,    // suggested brightness for that preset
    output wire [7:0]  preset_contrast,      // suggested contrast  for that preset
    // 生效值（预设基准 + 参数偏移）—— 送 layer_mixer 的也是这两个，
    // 所以 OSD 必须显示它们，否则屏上的数字与实际画面不符。
    output wire [7:0]  eff_brightness,
    output wire [7:0]  eff_contrast,

    // ---------------- serial text status ----------------
    // Exported so the board can show something useful while the operator is
    // typing. Without this there is no way to tell "the FPGA never received
    // anything" apart from "it received the text but did not render it",
    // which are completely different faults.
    output wire [5:0]  uart_len,             // characters typed so far
    output wire [7:0]  uart_last,            // most recent byte received
    output wire        uart_commit,          // 1-clock pulse per new message

    // The FIRST character of the message memory itself. This is the tap that
    // answers "did the received byte actually land?" on the board, which is
    // the question simulation kept getting wrong.
    output wire [7:0]  uart_mem0
);

    // ==================================================================
    //  pixel coordinate counters
    //  de is high for exactly 640 consecutive clocks per line, so counting
    //  on de gives the pixel position that matches the data being read out
    //  of the frame buffer.
    // ==================================================================
    reg [9:0] cx, cy;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cx <= 10'd0;
            cy <= 10'd0;
        end else if (de) begin
            if (cx == (H_ACTIVE[9:0] - 10'd1)) begin
                cx <= 10'd0;
                cy <= (cy == (V_ACTIVE[9:0] - 10'd1)) ? 10'd0 : (cy + 10'd1);
            end else begin
                cx <= cx + 10'd1;
            end
        end
    end

    // exported as ports (see the port list): one coordinate source for the
    // whole layer stack, including the OSD
    assign pix_x = cx;
    assign pix_y = cy;

    // ==================================================================
    //  mode state machine
    // ==================================================================
    wire        text_en;
    wire [2:0]  text_color;
    wire [7:0]  trans_alpha;
    wire        auto_play;
    wire        next_image;
    wire        iris_en;
    wire [31:0] iris_aperture;

    // ==================================================================
    //  image swap detector -- the real fade trigger
    //
    //  A change in disp_buf_idx means sd_card_bmp has just committed a new
    //  picture. Rising edge only, so the pulse is one video clock wide and the
    //  FSM cannot retrigger on it.
    //
    //  THE COMPARISON MUST USE THE PREVIOUS VALUE.
    //
    //  The first version was:
    //
    //      disp_buf_idx_d <= disp_buf_idx;
    //      swap_seen      <= (disp_buf_idx != disp_buf_idx_d);
    //
    //  which compares the CURRENT index against the OLD one -- and because
    //  swap_seen is a register, the comparison that matters is evaluated one
    //  clock later, by which time disp_buf_idx_d has already caught up. Both
    //  sides are then equal and swap_seen settles at 0.
    //
    //  MEASURED ON THE BOARD: image_swap therefore never went high. The mode
    //  FSM faded the picture out, entered WAIT_SWAP, and waited for a swap that
    //  could never be reported. The screen stayed black and the FSM never
    //  returned to idle, so neither KEY2 nor KEY3 did anything afterwards --
    //  exactly the reported behaviour.
    //
    //  Comparing against the registered previous value gives a real one-clock
    //  pulse per swap. Note swap_d is updated from swap_q, not from
    //  disp_buf_idx directly: comparing a register against something that
    //  updates on the same edge is what made the first version always zero.
    //
    //  AND THE INPUT NEEDS SYNCHRONISING FIRST.
    //
    //  disp_buf_idx is driven by sd_card_bmp in the 100 MHz domain and sampled
    //  here in the 25 MHz domain. It was being fed to the detector raw, with no
    //  synchroniser at all -- so the sampler could catch it mid-transition and
    //  return a value that is neither 0 nor 1. Whenever that happened the
    //  detector compared a garbage value against a good one and either missed
    //  the change or reported one that never occurred. Every other cross-domain
    //  signal in this design (hold_load, busy_stack, load_busy) has a two-stage
    //  synchroniser; this one was the exception, and it is the one the whole
    //  transition depends on.
    //
    //  Two flops make the sampled value a clean 0 or 1, and the detector then
    //  sees a real step rather than a smear.
    // ==================================================================
    reg  [1:0]  disp_idx_sync;
    reg  [1:0]  swap_q, swap_d;
    reg         image_swap;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            disp_idx_sync <= 2'd0;
            swap_q        <= 2'd0;
            swap_d        <= 2'd0;
            image_swap    <= 1'b0;
        end else begin
            disp_idx_sync <= disp_buf_idx;           // 2-stage synchroniser
            swap_q        <= disp_idx_sync;          // current, registered
            swap_d        <= swap_q;                 // previous
            image_swap    <= (swap_q != swap_d);     // one clock wide
        end
    end

    // (image_swap is the register declared above; there is no separate wire for
    //  it any more, and no swap_seen -- that name belonged to the version whose
    //  comparison never fired.)
    wire        self_test_done;
    wire        emg_debounced;

    // load_busy arrives from sd_card_bmp, which runs on sd_card_clk (100 MHz),
    // while everything here runs on video_clk (25 MHz). Two-stage synchroniser:
    // it is only an informational flag, so a couple of clocks of skew are
    // harmless, but sampling it unsynchronised would risk metastability.
    reg  [1:0] load_busy_sync;
    always @(posedge clk or posedge rst) begin
        if (rst) load_busy_sync <= 2'b00;
        else     load_busy_sync <= {load_busy_sync[0], load_busy};
    end
    wire load_busy_s = load_busy_sync[1];

    // next_req_pending arrives from sd_card_bmp on sd_card_clk (100 MHz). The
    // FSM wants a single-cycle pulse in video_clk, so synchronise and
    // edge-detect. Three stages: two to settle the level, one to remember the
    // previous level for the edge.
    reg  [2:0] req_sync;
    always @(posedge clk or posedge rst) begin
        if (rst) req_sync <= 3'b000;
        else     req_sync <= {req_sync[1:0], next_req_pending};
    end
    wire next_req_pulse = req_sync[1] & ~req_sync[2];

    mode_fsm #(
        .SYNC_STAGES   (2),
        .DEBOUNCE_BITS (DEBOUNCE_BITS),
        .SELFTEST_CYCLES (SELFTEST_CYCLES),
        .FADE_LEN_PARAM  (FADE_LEN),
        .FADE_SHIFT      (FADE_SHIFT),
        .IRIS_LEN_PARAM  (IRIS_LEN)
    ) u_mode_fsm (
        .clk            (clk),
        .rst            (rst),
        .sw_emergency   (sw_emergency),
        .key_next       (key_next),
        .key_auto       (key_auto),
        .key_mode       (key_mode),
        .sd_card_ok     (sd_card_ok),
        .audio_link_ok  (audio_link_ok),
        .sd_state_code  (sd_state_code),
        .mode           (mode),
        .mode_changed   (mode_changed),
        .text_en        (text_en),
        .text_color     (text_color),
        .trans_alpha    (trans_alpha),
        .transition_run (),
        // ================================================================
        //  param_transition GOES IN. transition_type COMES OUT -- and is left
        //  open on purpose, which used to be mistaken for the same thing.
        //
        //  The old note here said "DO NOT CONNECT transition_type", because
        //  wiring the OUTPUT back onto this module's INPUT puts two drivers on
        //  one signal and TD warns "assignment to input 'param_transition'".
        //  That is still true and still the reason the output stays open.
        //
        //  What was WRONG was the conclusion drawn from it: the input was then
        //  left unconnected to anything at all, so the type could never be
        //  chosen and the acceptance clause "拨码可切换效果类型" could not be
        //  met by any switch. The input is an input; it has to come FROM
        //  somewhere. It now goes straight into mode_fsm, which is the module
        //  that owns the effect choice.
        // ================================================================
        .param_transition(param_transition),
        .transition_type(),
        .auto_play      (auto_play),
        .next_image     (next_image),
        .self_test_done (self_test_done),
        .emg_debounced  (emg_debounced),
        .image_swap     (image_swap),
        .iris_en        (iris_en),
        .iris_aperture  (iris_aperture),
        // two-phase transition handshake. hold_load is asserted while the
        // CURRENT picture fades out, which stops sd_card_bmp from starting the
        // next load until the screen is black.
        .hold_load      (hold_load),
        .load_busy      (load_busy_s),
        .next_req       (next_req_pulse),
        .next_grant     (next_grant),
        .busy_stack     (busy_stack),
        .load_req       (load_req),
        .ts_state_o     (ts_state_o)
    );

    // ==================================================================
    //  latency measurement
    //  start : the debounced emergency signal
    //  stop  : the mode register update
    //  result: latency_cycles x 40 ns
    // ==================================================================
    latency_meter u_latency_meter (
        .clk                 (clk),
        .rst                 (rst),
        .start_evt           (emg_debounced),
        .stop_evt            (mode_changed),
        .latency_cycles      (latency_cycles),
        .latency_valid       (latency_valid),
        .latency_measurement (latency_measurement),
        .latency_one_frame   (latency_one_frame)
    );

    // ==================================================================
    //  colour presets  (the A contribution to parameter item 4)
    //
    //  The preset decides the text colour, the backdrop colour and a
    //  suggested brightness. It exists because an emergency panel has to be
    //  re-configured instantly: a site that is normally "standard" may need
    //  "night" the moment the mains fail, and nobody is going to walk over
    //  and dial a value in.
    //
    //  PRIORITY (this is a safety rule, not a preference)
    //  ---------------------------------------------------
    //  EMERGENCY MODE ALWAYS WINS. If the site had been left on the "night"
    //  preset, an evacuation must not be dim and low contrast. So the
    //  effective preset is forced to "emergency" whenever mode == EMERGENCY,
    //  regardless of what the user selected.
    // ==================================================================
    localparam PRESET_STD  = 2'd0;
    localparam PRESET_EMG  = 2'd1;
    localparam PRESET_NIGHT= 2'd2;
    localparam PRESET_HI   = 2'd3;

    // Emergency mode is checked FIRST here rather than through a nested
    // ternary. Same logic, but written as one always block so there is only
    // one place that decides the effective preset.
    reg [1:0] eff_preset_r;

    always @(*) begin
        if (mode == 2'd1)
            eff_preset_r = PRESET_EMG;              // emergency forces it
        else if (param_preset_en)
            eff_preset_r = param_preset;            // user selected one
        else
            eff_preset_r = PRESET_STD;              // default signage
    end

    wire [1:0] eff_preset = eff_preset_r;

    assign cur_preset = eff_preset;

    reg [23:0] preset_fg;
    reg [23:0] preset_bg;
    reg [7:0]  preset_br;
    reg [7:0]  preset_con;

    // ------------------------------------------------------------------
    //  预设的画面参数（亮度 / 对比度）
    //
    //  2026-10-13 按要求给预设加"效果"。数值一律用【百分比】，线性换算成
    //  混合器码值（layer_mixer 里 128 = 中性，正好对着 50%）：
    //
    //      code = pct * 255 / 100        （四舍五入）
    //
    //      预设            亮度%  →码    对比度% →码      来源
    //      工作 STD         40   →102     60    →153     ← 按要求（40% / 60%）
    //      应急 EMG         50   →128    100    →255     ← 按要求（50% / 100%）
    //      夜间 NIGHT       25   → 64     45    →115     A 补：更暗 + 更柔和
    //      高对比 HI        78   →199     80    →204     A 补：更亮 + 更锐利
    //
    //  ⚠️ 应急 100% 对比度 = 增益 2.0 ⇒ v ≥ 129 会被整片吹白。文字最醒目，
    //     但地图的中间调会丢；若地图要看细节，把 EMG 的对比度降到 70~80%
    //     （码 179~204）即可，改下面一个数字。
    //
    //  语义：预设定"基调"，B 的参数（C1 选 BRIGHT/CONTRA + C8/C7 加减）
    //  变成【相对基调的偏移】，128 = 完全不偏移 —— 所以开机默认恰好就是
    //  102/153（工作）与 128/255（应急），而手动微调仍然有效。
    //  换算表见《组装与校验报告》§10.1。
    // ------------------------------------------------------------------
    always @(*) begin
        case (eff_preset)
            PRESET_EMG: begin
                // white on red: maximum salience at distance
                preset_fg  = 24'hFFFFFF;
                preset_bg  = 24'hC01010;
                preset_br  = 8'd128;                // 50%
                preset_con = 8'd255;                // 100%
            end
            PRESET_NIGHT: begin
                // amber on black: avoids glare, and a low brightness saves
                // battery when the panel is running on a UPS
                preset_fg  = 24'hFFB000;
                preset_bg  = 24'h000000;
                preset_br  = 8'd64;                 // 25%
                preset_con = 8'd115;                // 45%
            end
            PRESET_HI: begin
                // pure white on pure black for smoke or very low visibility
                preset_fg  = 24'hFFFFFF;
                preset_bg  = 24'h000000;
                preset_br  = 8'd199;                // 78%
                preset_con = 8'd204;                // 80%
            end
            default: begin
                // standard signage = 工作模式
                preset_fg  = 24'hFFFFFF;
                preset_bg  = 24'h101820;
                preset_br  = 8'd102;                // 40%
                preset_con = 8'd153;                // 60%
            end
        endcase
    end

    assign preset_brightness = preset_br;
    assign preset_contrast   = preset_con;

    // ==================================================================
    //  生效的亮度 / 对比度 = 预设基准 + 参数偏移（128 = 不偏移），各自饱和
    //
    //      eff = clamp( 预设值 + 参数 - 128 , 0, 255 )
    //
    //  加法用 9 位（0..510），再减去 128；下溢钳到 0、上溢钳到 255，
    //  不会回绕。layer_mixer 收的就是这两根，所以"应急 50/100、工作 40/60"
    //  在参数不动时是精确成立的。
    // ==================================================================
    wire [8:0] br_sum = {1'b0, preset_br} + {1'b0, param_brightness};
    wire [8:0] br_off = (br_sum >= 9'd128) ? (br_sum - 9'd128) : 9'd0;

    wire [8:0] co_sum = {1'b0, preset_con} + {1'b0, param_contrast};
    wire [8:0] co_off = (co_sum >= 9'd128) ? (co_sum - 9'd128) : 9'd0;

    // ------------------------------------------------------------------
    //  打一拍再送出去（2026-10-13，审查意见 M2）
    //
    //  eff_* 只随"人的操作"变化，打一拍零代价；但能把上面这组
    //  "9 位加法 + 比较 + mux" 从 video_clk 的像素路径上摘掉 ——
    //  那条路径（→ layer_mixer 的 8×8 乘法 → vout_rgb）整编后只剩
    //  +2.722 ns 余量（见《组装与校验报告》§5.3），不能再往前堆逻辑。
    //
    //  复位值取 128（中性）而不是预设值：上电头一拍画面还没 validity，
    //  用中性值不会闪黑。
    // ------------------------------------------------------------------
    reg [7:0] eff_bright_r, eff_contrast_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            eff_bright_r   <= 8'd128;
            eff_contrast_r <= 8'd128;
        end else begin
            eff_bright_r   <= (br_off > 9'd255) ? 8'd255 : br_off[7:0];
            eff_contrast_r <= (co_off > 9'd255) ? 8'd255 : co_off[7:0];
        end
    end

    assign eff_brightness = eff_bright_r;
    assign eff_contrast   = eff_contrast_r;

    // preset-driven text colour codes (matches text_layer's color_sel encoding)
    reg [2:0] preset_color_sel;
    always @(*) begin
        case (eff_preset)
            PRESET_EMG:   preset_color_sel = 3'd1;   // red
            PRESET_NIGHT: preset_color_sel = 3'd3;   // yellow/amber
            PRESET_HI:    preset_color_sel = 3'd0;   // white
            default:      preset_color_sel = 3'd0;   // white
        endcase
    end

    // ==================================================================
    //  serial text entry
    //
    //  Types a new subtitle over the serial port. The module owns the receive
    //  side and a staging buffer; it writes the finished line into
    //  scroll_timer's message memory and then pulses msg_reload, which is what
    //  actually puts it on the screen.
    //
    //  It sits in THIS clock domain (video_clk) on purpose: the message
    //  memory it writes is read by scroll_timer on the same clock, so there
    //  is no clock crossing anywhere in the text path.
    // ==================================================================
    wire        ut_we;
    wire [4:0]  ut_addr;
    wire [7:0]  ut_data;
    wire        ut_reload;
    wire [5:0]  ut_len;
    wire [7:0]  uart_last_byte;
    wire [5:0]  uart_line_len;
    wire        uart_ok;

    // How long the power-on message is. uart_text reports 0 until the first
    // message is committed, and the bounce must still be sized for the message
    // that is actually on screen during that time.
    //
    // KEEP IN STEP WITH scroll_timer's MSG_RESET, whose byte 0..15 are the
    // "KEEP CLEAR EXIT " default; everything past that is padding.
    localparam [5:0] DEFAULT_MSG_LEN = 6'd16;

    wire [5:0] eff_len = (ut_len == 6'd0) ? DEFAULT_MSG_LEN : ut_len;

    uart_text #(
        .MSG_LEN  (MSG_LEN),
        .CLK_FREQ (25_000_000),
        .BAUD     (115200)
    ) u_uart_text (
        .clk        (clk),
        .rst        (rst),
        .rx         (uart_rx_pin),
        .msg_we     (ut_we),
        .msg_waddr  (ut_addr),
        .msg_wdata  (ut_data),
        .msg_reload (ut_reload),
        .msg_len    (ut_len),
        .rx_last    (uart_last_byte),
        .rx_len     (uart_line_len),
        // Left OPEN on purpose. It is driven and then never read, and a net
        // with a driver but no load is exactly what makes a synthesiser emit
        // "net has no load". An empty connection says "unused, and I know".
        .rx_idle    (),
        .rx_ok      (uart_ok)
    );

    assign uart_len    = uart_line_len;
    assign uart_last   = uart_last_byte;
    assign uart_commit = uart_ok;

    // ==================================================================
    //  scroll timer: fills the message, then walks the offset
    // ==================================================================
    wire        sc_we;
    wire [4:0]  sc_addr;
    wire [7:0]  sc_data;
    // SIGNED and 11 bits: the line bounces, so this goes negative as it leaves
    // the left edge. See the header of scroll_timer.v.
    wire signed [10:0] scroll_x;

    scroll_timer #(
        .MSG_LEN      (MSG_LEN),
        .TEXT_MAX     (TEXT_MAX),
        .CHAR_W       (CHAR_W),
        .SCROLL_TICKS (SCROLL_TICKS),
        .FILL_HOLD    (10'd64),
        .HOLD_TICKS   (HOLD_TICKS),
        // let the timer decide: a message that fits stays still and centred
        .SCROLL_ALWAYS(0),
        .H_ACTIVE     (H_ACTIVE)
    ) u_scroll_timer (
        .clk        (clk),
        .rst        (rst),
        .scroll_en  (text_en),          // scroll in every mode
        .scroll_rst (rst),
        .msg_we     (ut_we),
        .msg_waddr  (ut_addr),
        .msg_wdata  (ut_data),
        .msg_reload (ut_reload),
        .used_len   (eff_len),
        .char_we    (sc_we),
        .char_addr  (sc_addr),
        .char_data  (sc_data),
        .scroll_x   (scroll_x),
        .fill_done  (fill_done),
        .msg0_dbg   (uart_mem0)
    );

    // ==================================================================
    //  text layer
    // ==================================================================
    wire [23:0] text_rgb;
    wire [7:0]  text_alpha;
    wire [19:0] time_digits;        // {d4, d3, d2, d1, d0}, 4 bits each

    text_layer #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE),
        .CHAR_W   (CHAR_W),
        .CHAR_H   (16),
        .TEXT_MAX (TEXT_MAX),
        .ROW1_Y   (ROW1_Y),
        .ROW2_Y   (ROW2_Y),
        .BASE1_X  (BASE1_X),
        .BASE2_X  (BASE2_X),
        .COLOR_BG (24'h101820)
    ) u_text_layer (
        .clk         (clk),
        .rst         (rst),
        .x           (pix_x),
        .y           (pix_y),
        .de          (de),
        .text_en     (text_en),
        .color_sel   (preset_color_sel),   // driven by the colour preset
        .scroll_x    (scroll_x),
        .bg_color    (preset_bg),          // backdrop from the preset (runtime)
        .char_we     (sc_we),
        .char_addr   (sc_addr),
        .char_data   (sc_data),
        .char_clr    (1'b0),
        .counter_en  (1'b1),
        .counter_rst (rst),
        .text_rgb    (text_rgb),
        .text_alpha  (text_alpha),
        .time_digits (time_digits)
    );

    // export the counter digits for whoever wires the 7-segment display
    assign time_d0 = time_digits[3:0];
    assign time_d1 = time_digits[7:4];
    assign time_d2 = time_digits[11:8];
    assign time_d3 = time_digits[15:12];

    // ==================================================================
    //  layer mixer
    // ==================================================================
    layer_mixer #(
        .H_ACTIVE (H_ACTIVE),
        .V_ACTIVE (V_ACTIVE),
        .SPEC_X   (SPEC_X),
        .SPEC_Y   (SPEC_Y),
        .SPEC_W   (SPEC_W),
        .SPEC_H   (SPEC_H),
        .USE_TEXT (1),
        .USE_SPEC (1),
        .USE_ENHANCE (1),
        // Fade-through-black. Enabled here so the layer mixer's own default
        // stays the safe one (0) and this is the single place that turns the
        // effect on for the real design.
        //
        // mode_fsm now has a watchdog and a lockout around the ramp, so a
        // misbehaving trigger can no longer leave the picture dark: the fade
        // is cosmetic and must never be able to blank the display.
        .USE_FADE (1),
        // Radial iris: the picture is revealed from the middle outwards.
        //
        // *** LEFT OFF, AND THIS IS A MEASURED DECISION, NOT AN OVERSIGHT. ***
        //
        // It was switched on, and that BROKE the 25 MHz timing:
        //
        //     before  video_clk SWNS  +0.131 ns   0 negative endpoints
        //     after   video_clk SWNS  -0.472 ns  14 negative endpoints
        //
        // The failing path is inside u_layer_mixer, and it is the iris:
        //
        //     Begin : u_layer_mixer/dxr[9]              <- the x difference
        //     End   : u_layer_mixer/vout_rgb_reg[10]
        //     Data Path Delay : 40.120 ns  vs budget 40.000 ns
        //
        // because `after_iris = (USE_IRIS && iris_out) ? 0 : dimmed` makes the
        // whole metric -- |dx|*240 + |dy|*320, a 32-bit compare, and the final
        // mux -- into real logic on the pixel path. With USE_IRIS = 0 the
        // expression constant-folds to `dimmed` and the synthesiser deletes all
        // of it, which is why the path did not exist before.
        //
        // The acceptance clause "拨码可切换效果类型" does NOT need the iris: a
        // hard cut and a fade are two effect types and param_transition now
        // genuinely selects between them (proved in tb_mode_fsm PART 6b). The
        // iris stays fully implemented and selectable as value 2; it just
        // cannot be DISPLAYED until its metric is pipelined.
        //
        // TO TURN IT BACK ON: pipeline dxr/dyr (and probably the compare) in
        // layer_mixer first, then re-measure. Do NOT just set this to 1 and
        // ship -- a negative setup slack on video_clk is a gating failure, and
        // 27 ps of margin is not margin.
        .USE_IRIS (0)
    ) u_layer_mixer (
        .clk         (clk),
        .rst         (rst),
        .x           (pix_x),
        .y           (pix_y),
        .de          (de),
        .base_rgb    (display_valid ? vout_data_raw : 24'd0),
        .scale_rgb   (display_valid ? scale_rgb   : 24'd0),
        .text_rgb    (text_rgb),
        .text_alpha  (text_alpha),
        .spec_rgb    (spec_rgb),
        .spec_alpha  (spec_alpha),
        .osd_rgb     (osd_rgb),
        .osd_alpha   (osd_alpha),
        .trans_alpha (trans_alpha),
        .iris_en     (iris_en),
        .iris_aperture   (iris_aperture),
        .bypass      (1'b0),
        // 送混合器的是【生效值】（预设基准 + 参数偏移），不是原始参数：
        // 这样"应急 50/100、工作 40/60"在参数不动时精确成立。
        .brightness  (eff_brightness),
        .contrast    (eff_contrast),
        .vout_rgb    (vout_data)
    );



    // the synchronised load_busy, for the 7-segment diagnostic
    assign load_busy_o = load_busy_s;

endmodule
