
// ============================================================================
//  KEY MAPPING -- taken from the board manual's pin table
//
//      KEY1 A2   KEY2 B2   KEY3 B1   KEY4 C1
//
//      key2 (B2) : manual next       -- the middle button
//      key3 (B1) : rotation on/off   -- the button to its left
//      key4 (C1) : 短按 = 切换到下一参数项 / 长按(0.6s) = 只弹出 OSD（不换项、不改值）
//                  数值改动只由 C8 拨码触发（方向由 C7 定）
//                  -- Task B (param_regs.key_next_param)
//                  ★ 2026-10-14 交互改版；原来是"按一下就换参数项"
//                  (mode_fsm also takes it as `key_mode`, but kmode_press there
//                   is computed and never read, so there is no double action.)
//
//  Every earlier attempt named these key1=B2, key2=B1, key3=C1 -- one off from
//  the manual throughout. The pins were wired correctly; the NAMES were wrong,
//  and since the names are what get assigned to "next" and "rotate", the
//  functions landed on the wrong buttons. B1 (the manual's KEY3) was being read
//  as `key2` and fed to "next", which is why it kept changing the picture.
//
//  rst_n is still on A2, which the manual calls KEY1. KEY1 is therefore the
//  reset button and is not used as a normal input.
// ============================================================================
// ============================================================================
module top(
    input                       clk,
    input                       rst_n,
    // Physical button names come from the board manual:
    //     KEY1 A2   KEY2 B2   KEY3 B1   KEY4 C1
    input                       key2,           // KEY2 B2  手动下一张（中间键）
    input                       key3,           // KEY3 B1  自动轮播 开/关（左边键）
    input                       key4,           // KEY4 C1  短按=切换下一项 / 长按=只弹OSD（Task B / param_regs）
    input                       sw_emergency,   // 应急触发拨码 C6（SW3，勿挪）
    input                       sw_param_up,    // C8（SW1）：按住 = 按当前方向连续步进
    input                       sw_dir,         // C7（SW2）：方向选择，拨上 = +，拨下 = −
    input                       sw_audio_en,    // C5（SW4）：音频频谱显示开关（Task C）
    input                       uart_rx,        // 串口接收（PC 改字幕）
    output                      dbg_latency,    // LED4 B12 延迟脉冲
    output                      dbg_one_frame,  // LED1 A4  延迟合格指示
    output                      led_param_act,  // LED2 A3  参数激活（Task B）
    output                      led_audio_act,  // LED3 C10 音频有效（Task C）

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire [23:0] vout_data_layers;    // Task A: pixel after layer mixing

// Task A: outputs of the layer wrapper
wire [1:0]  layer_mode;
wire        layer_mode_changed;

// [FIX 2026-10-25b] Single authoritative SW3 state.
// Audio, picture-source selection and the layer FSM must consume the SAME
// value.  The board input is active-low, while all three consumers use
// active-high emergency semantics, so invert exactly once here.
// [FIX 2026-10-27 v7] THE BOARD NEEDS TWO POLARITIES.
//   picture / subtitle : emergency when the pin is HIGH  (same as v3)
//   audio              : emergency when the pin is LOW   (same as v3)
// One shared value cannot work: with emg_active only, one of the two ends
// up reversed whichever way it is defined.  v3 shipped exactly this split
// and both the picture and the sound were correct on the board.
wire        emg_ui    =  sw_emergency;
wire        emg_audio = ~sw_emergency;
wire [1:0]  layer_cur_preset;
// 预设的"基准亮度/对比度"不再导出：video_layers_top 内部已经把
// "预设基准 + 参数偏移"算成了 eff_*（见那份文件的预设表），顶层只收生效值。
wire [7:0]  layer_eff_bright;        // 生效亮度（送 OSD 显示，与画面一致）
wire [7:0]  layer_eff_contrast;
wire [31:0] layer_latency_cycles;
wire        layer_latency_valid;
wire        layer_latency_run;
wire        layer_latency_one_frame;
wire        display_valid;

// ----------------------------------------------------------------------
//  Task B：OSD 显示条（osd_bar）用到的线网
//
//  这些【必须在使用之前声明】—— u_video_layers 的例化在下面用到 osd_rgb /
//  osd_alpha / vl_pix_x / vl_pix_y。Verilog 遇到未声明的标识符会先建一个
//  隐式 1 位 wire，后面再显式声明就报 'already declared'，而且位宽会静默
//  变 1 位（和上面 img_found_cnt 那个坑同一类）。
// ----------------------------------------------------------------------
wire [9:0]  vl_pix_x;             // 图层栈导出的像素坐标（唯一坐标源）
wire [9:0]  vl_pix_y;
wire [23:0] osd_rgb;              // osd_bar -> layer_mixer 第 7 级
wire [7:0]  osd_alpha;            // 二值：0 透明 / 255 不透明

// ----------------------------------------------------------------------
//  Task B：参数字典（param_regs.v）发布出来的参数
//
//  这是【全系统唯一的参数源】（《接口契约与分工表》§4.2 / §6）。
//  A 只读；也不得在别处再声明一份 —— 两个"真值源"联调时必翻车。
//  这些是 param_regs 的输出，例化见下面"公共参数接口模块"那一段。
//
//  亮度取值参考（都是实测过的）：128 = 中性；64 = out = saturate(v-64)，
//  中间调减半、暗部 v<64 压死；32 = out = saturate(v-96)，v<96 全黑
//  （上板表现就是"图片整幅发黑"，只留字幕和 OSD 还亮）。
//  要按比例调暗、不压死暗部应该动 contrast（乘系数），不是 brightness。
// ----------------------------------------------------------------------
wire [7:0]  param_brightness;
wire [7:0]  param_contrast;
wire [1:0]  param_transition;
wire [15:0] param_dwell_ms;
wire [1:0]  param_preset;
wire        param_preset_en;
wire [7:0]  param_volume;          // 音量（Task B 参数项 5）：128 = 原音量，0 = 静音
wire [2:0]  param_cur_param;
wire        param_osd_visible;
wire [7:0]  param_osd_alpha;
wire [7:0]  param_osd_value;
wire        param_activity;

wire [3:0]  state_code;

// ---- F2（2026-10-14）：装载看门狗 ----
// sd_load_timeout : sd_card_bmp 报"这次装载超时放弃"（单拍脉冲）
// sd_hang_code    : 粘滞诊断码，显示在【数码管第 5 位】
//                   0=从没卡过；1=源读完但帧写没写满；8|N=卡在读卡链（N=SD 状态码）
wire        sd_load_timeout;
wire [3:0]  sd_hang_code;
wire [6:0]  seg_hang;
// 看门狗超时时帧写通路也要复位：否则写侧停在上一次的半帧计数上，
// 下一帧会提前"写满"（画面撕裂）而不是干净地重新开始。
wire [6:0]  seg_data_0;
// 临时诊断：扫描找到几张图（来自 sd_card_bmp 的 img_found_o）。
// 这两个必须在使用之前声明 —— 否则 Verilog 会先建一个隐式 1 位 wire，
// 后面再显式声明就会报 'already declared'。
wire [3:0]  img_found_cnt;
wire [6:0]  seg_found_cnt;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

// ---- two-phase transition handshake between the layer stack and the card ----
// sdr_load_hold : layer stack -> sd_card_bmp, "do not start the next load yet"
// sdr_load_busy : sd_card_bmp -> layer stack, "the deferred load has begun"
// Both cross between video_clk (25 MHz) and sd_card_clk (100 MHz).
// sdr_load_hold is synchronised inside sd_card_bmp; sdr_load_busy is only used
// as an informational flag by the layer stack, which also synchronises it.
wire        sdr_load_hold;
wire        sdr_load_busy;
// sdr_next_req   : sd_card_bmp -> FSM, "I want to load the next picture"
// sdr_next_grant : FSM -> sd_card_bmp, "the screen is black, go ahead"
wire        sdr_next_req;
wire        sdr_next_grant;
// busy_stack : FSM -> sd_card_bmp, "a transition is in progress"
wire        sdr_busy_stack;
// sdr_load_now : FSM -> sd_card_bmp, one-clock pulse: start the load
wire        sdr_load_req;
// NOTE: sdr_auto_state / sdr_img_count / sdr_ts_state / sdr_load_busy_dbg were
// removed with the 7-segment diagnostics that displayed them. See the
// allocation table in the seg_scan instantiation below before adding anything
// back -- those digits belong to person B and person C.

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
// [C 2026-10-23] 这三个改由 audio_mclk -> video_clk 的跨时钟域逻辑驱动，
//                所以从 wire 改成 reg（输出节拍仍是 48 kHz 单拍）。
reg         audio_valid;
reg  [23:0] audio_left_data;
reg  [23:0] audio_right_data;

// ======================================================================
//  ★ Task C：音频链路的全部信号声明
//
//  ⚠️ 必须放在这里（首次使用之前）。
//  上一版犯过这个错：在模块例化里先用、几百行后才声明，
//  Verilog 会先建【隐式 1 位线网】，32 位地址被砍成 1 位
//  （HDL-7225 CRITICAL-WARNING 'already implicitly declared'）。
// ======================================================================
// --- SD 借总线 ---
wire        aud_sd_areq;
wire [31:0] aud_sd_aaddr;
wire        aud_sd_aack;
wire [7:0]  aud_sd_adata;
wire        aud_sd_advalid;
wire        aud_sd_aend;

// --- loader (SD -> SDRAM) ---
wire        aud_load_start;
wire        aud_load_done;
wire [20:0] aud_wr_base;
wire [15:0] aud_wr_l;
wire [15:0] aud_wr_h;
wire        aud_wr_req;
wire        aud_wr_ack;
wire [3:0]  aud_ld_state;
wire [15:0] aud_ld_secdone;

// --- player (SDRAM -> I2S) ---
wire        aud_pl_rd_req;
wire [20:0] aud_pl_rd_addr;
wire        aud_pl_rd_valid;
wire [31:0] aud_pl_rd_data;
wire        aud_spec_en;
wire        aud_emg_act;

// --- viz (频谱 / 电平 / LED3) ---
wire [23:0] aud_spec_rgb;
wire [7:0]  aud_spec_alpha;
wire [7:0]  aud_level_peak;
wire [15:0] aud_level_disp;
wire        aud_led_act;
wire        aud_buf_low;     // FIFO low-water mark from audio stream
wire        aud_data_ok;     // I2S player data-valid indication
wire        rst_all;         // common power-on / PLL reset
wire        audio_i2s_data_ok;   // I2S player has a valid FIFO word
// [C 2026-10-23] 音频并行样点（audio_mclk 域）→ 跨到 video_clk 域
wire [23:0] audio_pcm_data_a;
wire        audio_pcm_toggle_a;

// --- 迁移闸门 + 安全超时 ---
wire        aud_hold;
reg  [24:0] aud_por_cnt;      // 上电延时（约 0.67 s @ 50 MHz）
reg         audio_load_kicked;   // 启动脉冲只打一次（必须先声明再用）
reg  [27:0] aud_hold_wd;      // 超时计数（约 1.34 s @ 50 MHz）
reg         aud_gate_open;    // ★ 一旦放行就【永久锁存】，不再回退

wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路
// ======================================================================
//  上电复位（POR）：必须等【所有】PLL 都稳了才放开复位。
//
//  原来只等音频 PLL。但 sys_pll（100 MHz 给 SD、125 MHz 给 SDRAM）
//  是另一个 PLL，它没稳的时候复位就放开了，SD 的 CMD 状态机会锁死在
//  错误状态、再也回不来 —— 板上就是「数码管第 6 位恒 0、图像黑、字幕正常」。
//
//  这里用【只依赖外部 50 MHz 晶振】的计数器做延时。这颗晶振上电就有、
//  不经过任何 PLL，所以 PLL 再慢它也照样计时：2^20 / 50 MHz ≈ 21 ms。
//
//  不用 sys_pll 的 lock：sys_pll.v 里 .extlock(open) 把它丢了，而工程引用的是
//  sys_pll.ipc，TD 会按配置重新生成 .v，直接改生成文件会被覆盖。
//
//  ----------------------------------------------------------------------
//  注（2026-10-12，回退记录）：本文件上做过两轮"复位门控"实验 ——
//    · E1：把 POR 从 2^20（21 ms）延长到 2^22（≈84 ms）；
//    · R1：再加一段"数 sd_card_clk 边沿"的时钟活动检测（sdclk_ok）并联进 rst_all。
//  **上板实测两者都没有解决"开机偶发读卡卡住"（数码管 00）**，因此全部回退，
//  复位逻辑恢复成下面这个原始写法（POR + 音频 PLL lock）。
//  现象、排除过程与下一步见 doc/组装与校验报告.md §9.6。
//  ----------------------------------------------------------------------
reg [19:0] por_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            por_cnt <= 20'd0;
    else if (!por_cnt[19]) por_cnt <= por_cnt + 20'd1;
end
wire por_done = por_cnt[19];

// Audio PLL lock is used only as a power-on condition.  If the lock output
// glitches after startup, resetting the whole image/audio system produces a
// visible spontaneous restart.  Latch the first stable lock instead.
reg audio_lock_s1, audio_lock_s2, audio_lock_ok;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        audio_lock_s1 <= 1'b0;
        audio_lock_s2 <= 1'b0;
        audio_lock_ok <= 1'b0;
    end else begin
        audio_lock_s1 <= audio_pll_lock;
        audio_lock_s2 <= audio_lock_s1;
        if (audio_lock_s2)
            audio_lock_ok <= 1'b1;
    end
end

assign rst_all = ~rst_n | ~por_done | ~audio_lock_ok;

//  frame_rst_all 原来放在文件上方（约 151 行），但那里 rst_all 还没声明，
//  Verilog 会先建一个隐式 1 位 wire，ModelSim 直接报错（TD 只是容忍）。
//  挪到 assign rst_all 之后：无前向引用、无重复声明。
//  它只在下面 frame_read_write 的 .rst 用到（约 793 行），往下搬安全。
wire        frame_rst_all = rst_all | sd_load_timeout;

// ======================================================================
//  公共参数接口模块（Task B 骨架件：param_regs.v）
//
//  契约地位：全系统【唯一的参数源】。A 只读（《接口契约与分工表》§4.2 / §6）。
//  它替换掉了之前那段手写的"亮度加减"临时实现 —— 参数只能有一个来源。
//
//  ★ 接线（2026-10-14 交互改版，第二次）：
//      C1 (key4)     → key_next_param  短按 = 切换到下一参数项 / 长按(0.6s) = 只弹出 OSD
//                                        （低有效，内部消抖 10.5 ms）
//      C8 (sw_param_up) → sw_up        按住 = 按【当前方向】连续步进
//                                        ★ 数值改动【只】由它触发
//      C7 (sw_dir)      → dir_up       方向选择：拨上 = +，拨下 = −
//
//  ⚠️ C8 快速拨一下不会有反应：第一步要【按住 160 ms】才走（之后每 40 ms 一步），
//     松手重新计时（原《TaskB开发指南》§3 的手感，未改）。"按住 160 ms"这件事
//     本身就把机械抖动滤掉了，所以 C8 不需要额外消抖，只过同步器。
//  ⚠️ C7 现在是【方向电平】，不再是"参数 −"：它停在 1 位只代表方向为 +，
//     不会自己扫参数，所以不参与"武装"判断。
//
//  ⚠️ 《TaskB开发指南》§8 的接线片段写的是 `.key_next_param (key3)  // C1`，
//     但本工程里 key3 = B1（轮播）、key4 = C1。照那段抄会把参数项键接到
//     轮播键上，所以这里接的是 key4 —— 契约 §5.1 分配的就是 C1。
// ======================================================================
param_regs u_param_regs (
    .clk            (video_clk),
    .rst            (rst_all),

    .key_next_param (key4),          // KEY4 = C1
    .sw_up          (sw_param_up),   // C8：按住连续步进
    .dir_up         (sw_dir),        // C7：方向 +/−

    .brightness     (param_brightness),
    .contrast       (param_contrast),
    .transition     (param_transition),
    .dwell_ms       (param_dwell_ms),// 送给 OSD 显示；轮播计时还没有这个端口
    .preset         (param_preset),
    .preset_en      (param_preset_en),
    .volume         (param_volume),  // 音量：送到音频缩放（见 vol_scale）

    .cur_param      (param_cur_param),
    .osd_visible    (param_osd_visible),
    .osd_alpha      (param_osd_alpha),
    .osd_value      (param_osd_value),

    .activity       (param_activity),
    .step_count     ()               // 调试用；接出来只是多一根没负载的线
);
// （原先这里有一段手写的"亮度加减 + 消抖 + 武装"临时实现，
//   参数接口模块 param_regs.v 接进来之后已删除：
//   参数只能有一个来源，见上面「公共参数接口模块」那一段。）


// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    // 扫描上限。原来写 131071 扇区 = 67 MB，而卡上十几张图才 10 MB 左右。
    //
    // 这个值【直接决定开机黑屏多久】：bmp_read 在"未命中"时一次只前进 1 个扇区
    // （next_scan_sector_if_miss = scan_sector + 1），每读一个扇区约 200 us，
    // 所以扫 67 MB 要 26 秒 —— 板子上看就是"数码管卡在 2，一直黑屏"。
    //
    // 32767 扇区 = 16 MB，够放 17 张 900 KB 的图；扫完只要约 3 秒。
    // 如果以后图片总大小超过 16 MB，把这里按 图数*1800 扇区 再放大。
    // 扫描上限：恢复成整个扫描区（131071 扇区 = 67 MB）。
    //
    // 之前为了治「卡在 2、黑屏」把它缩到 32767（16 MB），那是治标：
    // 16 MB 以外的图就找不到了，卡一碎片化就会「只显示几张」。
    // 真正的修法在 bmp_read 里 —— 未命中步进从 1 个扇区改成 8 个，
    // 扫描快 8 倍（约 2.5 秒），所以这里可以放心用全盘范围。
    .SCAN_MAX_SECTOR   (32'd131071),
    //  2026-10-20：15 -> 12，并配合 bmp_read 里新加的【冷却】。
    //
    //  停止条件是 scan_found_total >= scan_target_count-1，而 scan_found_total
    //  是非阻塞赋值（判断时是旧值），所以 target=12 意味着「找到第 12 张就达标」。
    //  原来写 15 要找到 14 张，可卡上只有 12 张 —— 永远达不到，于是一路扫到
    //  SCAN_MAX_SECTOR（64 MB）末尾才停，这是复位后出图慢的主因。
    //
    //  达标之后不会立刻 scan_done：bmp_read 里先空走 COOL_STEPS(64) 步，
    //  让 SD 卡从「连续读」平滑过渡到「空闲」，然后才开始装载首图。
    //
    //  ⚠️ 这个值和卡上实际张数【必须一致】：卡上 12 张 -> 写 12。
    .SCAN_TARGET_COUNT (4'd12)   // 卡上 12 张：前 11 张轮播，第 12 张是应急图
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key2),      // KEY2 B2
    .key_auto          (key3),      // KEY3 B1
    .state_code        (state_code),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    // F2：装载看门狗（超时脉冲 + 粘滞诊断码）
    .load_timeout_o    (sd_load_timeout),
    .hang_code_o       (sd_hang_code),
    // see the note on u_video_layers: hold_load defers the START of the next
    // load so the old picture can fade out first
    .hold_load         (sdr_load_hold),
    .load_busy_o       (sdr_load_busy),
    // The load now starts on the layer FSM's GRANT, not on the key press.
    // next_req_pending tells the FSM "I want to load the next picture" (raised
    // by the key or by the auto-rotation tick); the FSM fades the current
    // picture out and then raises next_grant, which is what actually starts the
    // load -- so the buffer swaps while the screen is black.
    .next_req_pending  (sdr_next_req),
    .next_req          (sdr_next_grant),
    .busy_stack        (sdr_busy_stack),
    .load_req          (sdr_load_req),
    .auto_play_state   (),
    .img_found_o       (img_found_cnt),
    // ==================================================================
    //  应急图选择
    //
    //  SD 卡上【最后一张】图就是应急疏散图。这个信号为高时，
    //  sd_card_bmp 直接跳到那一张，而且普通轮播永远转不到它
    //  （折返点已经改成 count-2，见 sd_card_bmp 里的 next_index_limited）。
    //
    //  这里不能再看 layer_mode 的寄存器输出：图层 FSM 还负责自检/转场，
    //  一旦它的 mode 因复位窗口或极性判断停在 emergency，图片源就会一直
    //  选应急图，而声音已经按 SW3 切回来了。现在直接使用和音频完全相同的
    //  emg_active，保证“声音切哪一路，图片就选哪一路”。
    //
    //  关于「立即」：字幕变红是【真正的一帧之内】，拨上开关当场就变；
    //  但【图】要等 SD 卡读进来 —— 640x480x24 的 BMP 是 900 KB，
    //  SPI 跑 25 MHz，纯数据 0.29 秒，加上 FAT 和扇区开销约 0.5 秒。
    //  这是 SD 卡的物理限制，不是逻辑延迟。
    // ==================================================================
    .emg_sel           (emg_ui),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),

    // ★ Task C：音频借总线读扇区（开机一次性）
    .aud_req           (aud_sd_areq),
    .aud_addr          (aud_sd_aaddr),
    .aud_ack           (aud_sd_aack),
    .aud_data          (aud_sd_adata),
    .aud_dvalid        (aud_sd_advalid),
    .aud_end           (aud_sd_aend),
    .aud_hold          (aud_hold)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);



// =====================================================================
//  8 位数码管 —— 严格按《接口契约与分工表》§5.3 / 《板级接口与显示资源》
//  §2.4.4 的分配表接线。**A 只占第 6 位，其余一律留空。**
//
//      第 1 位  空                        ——            没分给任何人
//      第 2~3 位 B 的参数值（2 位十进制）    B            等 B 接
//      第 4~5 位 C 的音频电平（2 位十进制）  C            等 C 接
//      第 6 位  SD 卡状态码                  A / 例程      保留不变
//
//  `seg_data_N` 对应第 (N+1) 位（见 seg_scan 内部的 case）。
//
//  ---------------------------------------------------------------------
//  为什么把 A 自己的诊断位全删了
//
//  这里原来占了 5 位做 A 的现场诊断：串口已输入字符数、SD 读卡忙、
//  转场状态、扫描到的图片数、轮播开关。**那 5 位里第 2~5 位是 B 和 C 的**，
//  留着就等于长期占着别人的资源 —— B 的参数显示和 C 的音频电平接进来时
//  没有位置，而且这种占用在代码里看不出来，只有对着分配表才发现。
//
//  第 1 位属于"空"，也不该被填。
//
//  以后要加诊断，**不要动这 5 位**。真需要探针就：
//    · 用 4 个独立 LED（A4 / A3 / C10 / B12，不受 SW6 影响，也不与人共用）
//    · 或者先改分配表、三方签字，再加
//
//  需要判断"串口字节有没有真的写进消息存储器"时，`video_layers_top` 上
//  还留着 `uart_mem0` / `uart_last` / `uart_commit` 这几个口，接出来就能看，
//  比占数码管更合适。
// =====================================================================
// ---- 临时诊断：img_found_cnt / seg_found_cnt 的声明见文件上方 wire 区 ----
seg_decoder u_seg_decoder_found (
    .bin_data          (img_found_cnt),
    .seg_data          (seg_found_cnt)
);

// F2：装载卡死诊断码 → 数码管第 5 位
//     0 = 从没卡过；1 = 源读完但帧写没写满；8|N = 卡在读卡链（N = SD 状态码）
seg_decoder u_seg_decoder_hang (
    .bin_data          (sd_hang_code),
    .seg_data          (seg_hang)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    // 临时诊断：第 1 位显示【扫描找到几张图】。
    // 第 1 位本来就分配给「空」，而且是 A 自己那一行，不占 B/C 的位。
    // 诊断完把这一行改回 .seg_data_0 ({1'b1,7'b1111_111}), 即可。
    .seg_data_0        ({1'b1,seg_found_cnt}),  // 第 1 位：img_found_count（临时）
    .seg_data_1        ({1'b1,7'b1111_111}),   // 第 2 位：留空，等 B 的参数值
    .seg_data_2        ({1'b1,7'b1111_111}),   // 第 3 位：留空，等 B 的参数值
    .seg_data_3        (aud_level_disp[15:8]),  // 第 4 位：C 音频电平十位
    .seg_data_4        (aud_level_disp[7:0]),   // 第 5 位：C 音频电平个位
    .seg_data_5        ({1'b1,seg_data_0})     // 第 6 位：SD 状态码（A 保留）
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
video_layers_top #(
    .H_ACTIVE       (640),
    .V_ACTIVE       (480),
    .ROW1_Y         (424),
    .ROW2_Y         (448),
    // MARGIN_X was removed: the two text rows are now centred individually, and
    // their origins (BASE1_X / BASE2_X) are derived inside video_layers_top from
    // H_ACTIVE, MSG_LEN and CHAR_W. Override them there if ever needed.
    .CHAR_W         (10),
    .TEXT_MAX       (32),
    // 32 characters: double the old 16. This is the largest value that needs
    // NO change to text_layer -- character indices above TEXT_MAX-1 (32) do
    // not render, because char_addr is 5 bits there. At full length the line
    // still sweeps 320 px. See 串口改字说明.md.
    .MSG_LEN        (32),
    .SCROLL_TICKS   (32'd5_000_000),
    .SPEC_X         (192),
    .SPEC_Y         (336),
    .SPEC_W         (256),
    .SPEC_H         (72)
) u_video_layers (
    .clk              (video_clk),
    .rst              (rst_all),
    .de               (de),
    .vout_data_raw    (vout_data_raw),
    .display_valid    (display_valid),

    // Same active-high emergency state as audio_sd_stream and sd_card_bmp.
    .sw_emergency     (emg_ui),
    .key_next         (key2),      // KEY2 B2 = middle, next
    .key_auto         (key3),      // KEY3 B1 = left, rotation
    // key3 is genuinely unused: mode_fsm derives kmode_press from it and
    // never reads it. Left connected so the input is not floating.
    .key_mode         (key4),      // KEY4 C1, unused

    // 串口改字：接收 PC 终端发来的字幕
    .uart_rx_pin      (uart_rx),

    .sd_card_ok       (Sdr_init_done),
    .audio_link_ok    (1'b1),
    .sd_state_code    (state_code),
    // disp_buf_idx is the FADE TRIGGER. A change in it means sd_card_bmp has
    // just committed a new picture, i.e. the screen content actually changed.
    // The fade must start here, not at the FSM's next_image pulse: reading the
    // card and writing the frame takes tens of milliseconds, so a ramp started
    // at next_image finishes before the picture even appears.
    .disp_buf_idx     (disp_buf_idx),

    // ---- two-phase transition handshake ----
    // hold_load tells sd_card_bmp NOT to start the next image load yet. The
    // layer stack raises it the moment the user asks for the next picture and
    // drops it once the CURRENT picture has finished fading to black:
    //
    //     key pressed -> old picture fades to black (load held)
    //                 -> load released, buffer swaps while the screen is black
    //                 -> new picture fades up
    //
    // sdr_load_busy comes back the other way so the layer stack knows the
    // deferred load has actually begun.
    .hold_load        (sdr_load_hold),
    .load_busy        (sdr_load_busy),
    .next_req_pending (sdr_next_req),
    .next_grant       (sdr_next_grant),
    .busy_stack       (sdr_busy_stack),
    .load_req         (sdr_load_req),
    .ts_state_o       (),
    .load_busy_o      (),

    // 参数全部来自 param_regs（全系统唯一参数源），见上面「公共参数接口模块」。
    // 亮度 : 0..255，128 = 中性（加偏置，注意暗部会被压死）
    // 对比度: 0..255，128 = 中性（乘系数，想按比例变暗该动这个）
    .param_brightness (param_brightness),
    .param_contrast   (param_contrast),
    // HARD CUT (0) is the SAFE default for now.
    //
    // History: this was wired to a testbench signal and, on the board, the
    // picture went completely black while the subtitle stayed visible. That
    // is the signature of trans_alpha being stuck at 0, which makes the
    // fade-through-black dimmer multiply the picture by zero. The subtitle
    // survived because it sits ABOVE the dimmer, which confirms the layer
    // order is right and isolates the fault to the transition source.
    //
    // ------------------------------------------------------------------
    //  param_transition -- WHICH EFFECT PLAYS ON A PICTURE CHANGE
    //
    //      0 = hard cut      1 = fade through black      2 = radial iris
    //
    //  Hard-wired to 1 (fade) because person B's param_regs does not exist yet
    //  and this is the only value available. When B's parameter item 2 lands,
    //  replace the literal with `param_transition`.
    //
    //  === 已经不再是硬接的了 ===
    //  上面两句是历史记录（当时 B 的参数模块还没到，只能写死 2'd1）。
    //  现在这里接的是 param_regs 的 transition 输出。★ 2026-10-14 交互改版后：
    //  用 **C1 短按** 换到"转场"项（长按只弹 OSD、不换项），再**按住 C8**
    //  即按 **C7 选的方向** 在 0(硬切) / 1(淡入淡出) 之间切 —— 验收条款
    //  "拨码可切换效果类型" 从此真的成立。
    //  2(径向光圈) 契约里没开放：A 的 layer_mixer 是 USE_IRIS=0，显示不出来。
    //
    //  IT IS NOW ACTUALLY HONOURED. mode_fsm reads this input and selects the
    //  effect; before that it was a dead port, so no value here could change
    //  anything. The old note in this spot said the input "reads as 'x' ...
    //  still not explained" and then hard-cut the display to be safe -- but the
    //  literal below it was 2'd1, not the 2'd0 the note described, so the
    //  comment and the code disagreed.
    //
    //  The 'x' is explained now: an UNUSED port that vopt optimises away makes
    //  a hierarchical read return 'x'. It was never a wiring fault, and no
    //  safety fallback was ever needed.
    // ------------------------------------------------------------------
    // 转场类型 / 配色预设也来自 param_regs：
    //   ★ 2026-10-14 改版后：用 C1 短按换到"转场"项（长按只弹 OSD），
    //   再按住 C8 按 C7 的方向在 0(硬切) / 1(淡入淡出) 之间切换，
    //   于是验收条款"拨码可切换效果类型"从此真的成立（之前硬接 2'd1）。
    //   preset_en 只有用户真的选过预设才为 1；应急模式下 A 仍强制用应急配色。
    .param_transition (param_transition),
    .param_preset     (param_preset),
    .param_preset_en  (param_preset_en),

    .scale_rgb        (vout_data_raw),
    .spec_rgb         (aud_spec_rgb),
    .spec_alpha       (aud_spec_alpha),
    .osd_rgb          (osd_rgb),
    .osd_alpha        (osd_alpha),

    .vout_data        (vout_data_layers),

    // 像素坐标导出给 OSD 用（见下面的 u_osd_bar）。必须用这一份坐标，
    // 不能在 top 里自己再数一次 —— 那样就有两个"真值"，会漂。
    .pix_x            (vl_pix_x),
    .pix_y            (vl_pix_y),

    .mode             (layer_mode),
    .mode_changed     (layer_mode_changed),
    .fill_done        (),
    .time_d0          (),
    .time_d1          (),
    .time_d2          (),
    .time_d3          (),
    .latency_cycles   (layer_latency_cycles),
    .latency_valid    (layer_latency_valid),
    .latency_measurement (layer_latency_run),
    .latency_one_frame(layer_latency_one_frame),
    .cur_preset       (layer_cur_preset),
    // 预设基准亮度/对比度不在这里取了：wrapper 内部已经把它们和参数偏移
    // 合成 eff_brightness / eff_contrast（预设表现在带"效果"，见该文件）。
    .preset_brightness(),
    .preset_contrast  (),
    .eff_brightness   (layer_eff_bright),
    .eff_contrast     (layer_eff_contrast),

    // 串口诊断口全部留空。
    //
    // 它们原本接在数码管第 1 位上，而按分配表第 1 位是"空"、第 2~5 位是
    // B 和 C 的。诊断更该放在 4 个独立 LED（A4 / A3 / C10 / B12）上 ——
    // 那 4 个不与任何人共用，也不受 SW6 影响。
    //
    // 需要临时抓信号时再接出来：
    //   uart_len    当前这一行已输入几个字符
    //   uart_mem0   消息存储器第 1 个字节（判断"写进去没有"）
    //   uart_last   最近收到的字节
    //   uart_commit 每次提交一个脉冲
    .uart_len         (),
    .uart_last        (),
    .uart_commit      (),
    .uart_mem0        ()
);

assign vout_data = vout_data_layers;

// ======================================================================
//  Task B 扩展项④：参数 OSD 显示条（osd_bar）
//
//  接口按 doc/A包说明/TaskB开发指南.md 第四节冻结，本文件只做接线。
//
//  所有输入现在都来自公共参数接口模块 param_regs（见上面那一段），
//  它负责"弹出 / 保持 2 s / 250 ms 淡出 / 操作时重新弹出"的时序：
//      .osd_visible / .osd_alpha / .cur_param / .osd_value  ← param_regs
//      .brightness / .contrast / .transition / .dwell_ms / .preset ← param_regs
//
//  屏幕上因此是：C1 短按 = 切换参数项 / C1 长按 = 只弹出 OSD；数值靠按住 C8 改（方向由 C7 定）
//  → OSD 条自动弹出显示当前项，停手 2 s 后按 param_regs 的时序淡出（注意
//  layer_mixer 的 OSD 级是二值 mux，见 osd_bar.v 文件头，所以"淡出"实际表现为中途消失）。
//
//  ⚠️ 显示的是【生效值】不是原始参数：预设现在会给亮度和对比度定基调
//     （应急 50/100、工作 40/60，见 video_layers_top 的预设表），而键/拨码
//     是在基调上做 ±偏移。所以第 1、2 项要显示 eff_*，否则屏上的数字
//     会和实际画面不符（"显示骗人"）。
// ======================================================================
// （vl_pix_x / vl_pix_y / osd_rgb / osd_alpha 已在文件上方线网区声明）

wire [7:0] osd_value_shown = (param_cur_param == 3'd0) ? layer_eff_bright :
                             (param_cur_param == 3'd1) ? layer_eff_contrast :
                                                          param_osd_value;

osd_bar u_osd_bar (
    .clk         (video_clk),
    .rst         (rst_all),
    .x           (vl_pix_x),
    .y           (vl_pix_y),
    .de          (de),

    // ---- 全部来自 param_regs（唯一参数源）----
    .osd_visible (param_osd_visible),
    .osd_alpha   (param_osd_alpha),
    .cur_param   (param_cur_param),
    .osd_value   (osd_value_shown),      // 第 1/2 项显示生效值（见上）
    .brightness  (layer_eff_bright),
    .contrast    (layer_eff_contrast),
    .transition  (param_transition),
    .dwell_ms    (param_dwell_ms),
    .preset      (layer_cur_preset),   // 用【生效】预设，不是用户选的那个：
                                       // 应急模式会强制成 EMG，否则屏上会显示错档
                                       // （审查意见 S2）

    .osd_rgb     (osd_rgb),
    .osd_a       (osd_alpha)
);

// ======================================================================
//  临时诊断探针（排查「数码管第 6 位恒 0」用）
//
//  现象：state_code 恒 0 -> sd_init_done 恒 0，但字幕正常（说明 rst_all 已放开、
//        视频 PLL 正常）。板子上【没有任何指示】能区分下面三种情况：
//
//    A. 复位没放开   rst_all 被 ~rst_n 或 ~audio_pll_lock 拉住
//    B. 时钟没跑     sys_pll 的 sd_card_clk 死了
//    C. 卡不响应     FPGA 正常发命令，卡不理
//
//  B 最阴：sys_pll 的锁定信号根本没接出来，100 MHz 死了也不会有任何复位动作。
//
//  读法（⚠️ 以第 743/744 行的代码为准，这里曾写反过）：
//    LED1 A4  = ~rst_all        亮    -> 复位已放开（排除 A）
//                               灭    -> rst_all 一直被拉住
//    LED4 B12 = 100MHz 心跳     闪    -> sd_card_clk 在跑（排除 B）
//                               不闪  -> 100 MHz 时钟死了
//
//  两个都正常（A4 亮 + B12 闪）却还是 0  ->  只剩 C：卡或卡座的问题，换卡试。
//
//  诊断完把这两行改回（原设计语义）：
//      assign dbg_latency   = layer_mode_changed;
//      assign dbg_one_frame = layer_latency_one_frame;
// ======================================================================
reg [22:0] dbg_sdclk_cnt;
always @(posedge sd_card_clk or negedge rst_n) begin
    if (!rst_n) dbg_sdclk_cnt <= 23'd0;
    else        dbg_sdclk_cnt <= dbg_sdclk_cnt + 23'd1;
end

assign dbg_one_frame = ~rst_all;             // A4  LED1: 亮 = 复位已放开
assign dbg_latency   = dbg_sdclk_cnt[22];    // B12 LED4: 100MHz/2^23 约 12 Hz 闪 = 时钟在跑
// LED2 A3：参数激活指示（B 的钩子）。
// 注意：param_regs 的 activity 只有【一拍】宽（40 ns），直接接 LED 肉眼看不见，
// 所以这里做 200 ms 单稳态展宽 —— 只动灯，不动 param_regs 的任何输出。
// 25 MHz × 0.2 s = 5_000_000 拍（23 位放得下）。
reg [22:0] param_led_cnt;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)                     param_led_cnt <= 23'd0;
    else if (param_activity)         param_led_cnt <= 23'd5_000_000;  // 每次操作重新点亮
    else if (param_led_cnt != 23'd0) param_led_cnt <= param_led_cnt - 23'd1;
end

assign led_param_act = (param_led_cnt != 23'd0);
// ★ Task C：LED3 (C10) 兼作【诊断指示灯】
//   搬运阶段（aud_load_busy）也点亮，这样一眼能判断音频链路走到哪一步：
//     · 上电后亮约 0.3 秒再灭  -> 音频搬运成功，问题在播放侧
//     · 一直亮着不灭            -> 搬运卡住了（SD 总线没借到 / 写 SDRAM 没 ack）
//     · 完全不亮                -> loader 根本没启动（start 没送到 / 卡在复位）
// ★ Task C：LED3 (C10) 综合指示
//   aud_led_act : 有声音电平（真的在播）
//   aud_data_ok : I2S 侧成功从 FIFO 取到过字
//   ~aud_buf_low: FIFO 里有足够数据
//   三种任一成立就亮 —— 常亮说明数据在流动但幅度是 0；闪说明在正常播放。
assign led_audio_act = aud_led_act | aud_data_ok;

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (frame_rst_all),   // F2: 装载看门狗超时也复位写通路
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),      // ★ 经 shim 转一手
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：内部 I2S 测试音 =====================
// AMP 恢复成 24'sd2000000（≈23.8% 满量程，约 −12.5 dBFS）：
// 之前为了让现场安静把它置 0，但那样"音量参数"就无从缩放（0 × 任何数 = 0）。
// 现在音量交给下面的 vol_scale 做，最小音量是 0（真静音）。
// ======================================================================
//  ★★★ Task C：真实音频播放（SD -> SDRAM -> I2S）★★★
//
//  取代了原来那个 DDS 测试音发生器 hdmi_audio_tone_i2s_64fs。
//  音源是 SD 卡上的真实录音：alarm.mp3 = 紧急铃声、ring.mp3 = 一般铃声。
//
//  三段：
//    audio_sd_loader    开机一次性把两个音频从 SD 搬进 SDRAM（约 0.3 s）
//    audio_sdram_shim   SDRAM 读写仲裁（A 的图片通路一行不改）
//    audio_sd_player    SDRAM -> I2S，SW3 立即切换音源
// ======================================================================
// ======================================================================
//  ★ 音频 I2S：直接用【官方能出声的时序】
//    官方 hdmi_audio_tone_i2s_64fs.v 在板上确实有声，所以把它的 I2S
//    状态机原样搬进 audio_hdmi_i2s，只把样点来源换成 SD 音频 FIFO。
// ======================================================================
// ======================================================================
//  ★ 音频源：官方音阶发生器（板上验证过能出声）
//    先确认这条链路在本工程里能出声，再换回 SD 录音。
// ======================================================================
// 官方音阶发生器：do-re-mi，每音 0.5 秒循环（板上已验证有清晰声音）
// ======================================================================
// ======================================================================
//  Task C final audio source: SD card -> byte/word async FIFO -> I2S
//
//  The image loader and audio reader share the same SD controller.  The
//  arbiter in sd_card_bmp gives audio exactly one complete sector at a time
//  between BMP sectors, so the SD controller is never switched mid-sector.
//  The 8 KB FIFO covers image-load gaps; the player mutes on starvation
//  instead of repeating stale samples.
// ======================================================================
audio_sd_stream #(
    .ALARM_BASE_SECTOR (32'd131072),  // current audio_sd.bin: 471 sectors
    .ALARM_SECTORS     (32'd471),
    .RING_BASE_SECTOR  (32'd131543),  // 131072 + 471
    .RING_SECTORS      (32'd249)
) u_audio_sd_stream (
    .clk_sd       (sd_card_clk),
    .rst_sd       (rst_all),
    .sd_areq      (aud_sd_areq),
    .sd_aaddr     (aud_sd_aaddr),
    .sd_aack      (aud_sd_aack),
    .sd_adata     (aud_sd_adata),
    .sd_advalid   (aud_sd_advalid),
    .sd_aend      (aud_sd_aend),
    .emg          (emg_audio),          // C6 active-low pin -> active-high emergency
    .state_code   (aud_ld_state),
    .sec_done     (aud_ld_secdone),
    .clk_a        (audio_mclk),
    .rst_a        (rst_all),
    .rd_req       (aud_pl_rd_req),
    .rd_valid     (aud_pl_rd_valid),
    .rd_data      (aud_pl_rd_data),
    .buf_low      (aud_buf_low)
);

// 48 kHz I2S transmitter.  It consumes one 32-bit FIFO word (two 16 kHz
// samples) at a time, holding each sample for three 48 kHz frames.
audio_hdmi_i2s u_audio_hdmi_i2s (
    .I_mclk      (audio_mclk),
    .I_rst       (rst_all),
    .rd_req      (aud_pl_rd_req),
    .rd_valid    (aud_pl_rd_valid),
    .rd_data     (aud_pl_rd_data),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout),
    .data_ok     (audio_i2s_data_ok),
    // ★ 并行样点旁路（audio_mclk 域），供下面的跨时钟域逻辑使用
    .O_pcm_data  (audio_pcm_data_a),
    .O_pcm_toggle(audio_pcm_toggle_a)
);

// ----------------------------------------------------------------------
//  ★ 迁移闸门：音频没搬完之前，先不让 sd_card_bmp 开始扫描图片
//
//  aud_hold = 1  =>  不扫描
//  释放条件 = 音频加载完成  【或者】 3 秒看门狗超时
//
//  ⚠️ 那个 3 秒超时是【必须】的：
//     上一版没有它，音频一旦加载不成功，图片就永远不出现
//     （板上现象就是"黑屏、一张图都没有"）。
//     有它兜底：最坏情况是没声音，但画面一定出来。
// ----------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                 aud_por_cnt <= 25'd0;
    else if (!aud_por_cnt[24])  aud_por_cnt <= aud_por_cnt + 25'd1;
end

// ----------------------------------------------------------------------
//  ★★★ 迁移闸门（重要，这里踩过坑）★★★
//
//  aud_hold = 1 => 先不开始扫描图片，让音频把 SD 总线独占用完
//  释放条件 = 音频搬完 【或】 超时（约 1.34 s）
//
//  ⚠️ 旧写法是：
//        else if (!aud_hold) aud_hold_wd <= 0;
//        assign aud_hold = ~(aud_load_done | aud_hold_wd_hit);
//     计数器一旦触发就把自己清零 -> hit 拉高一拍后立刻回 0
//     -> aud_hold 每 2.7 s 只放开 1 个时钟（20 ns）
//     -> 扫描永远启动不了 -> 屏幕一张图都没有（板上就是这个现象）。
//
//  现在改成【锁存】：一旦放行就永久保持，绝不回退。
// ----------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        aud_gate_open <= 1'b0;
        aud_hold_wd   <= 28'd0;
    end else begin
        // 音频搬完，或者等了约 1.34 s（50 MHz 下 2^26 拍）还没搬完 -> 放行
        if (aud_load_done || aud_hold_wd[27])
            aud_gate_open <= 1'b1;
        // 还没放行才继续计数
        if (!aud_gate_open)
            aud_hold_wd <= aud_hold_wd + 28'd1;
    end
end

// ★★★ 音频 SD 搬运总开关 ★★★
//   1 = 打开（loader 会去借 SD 总线搬音频）
//   0 = 关闭（音频完全不碰 SD 卡，图片 100% 不受影响）
//
//   板上实测：音频借 SD 总线后，loader 卡在 S_FLUSH，且把 sd_card_top 的
//   内部状态搞坏，之后连图片扫描也起不来。在解决这个耦合之前，先把开关
//   关掉——保证 A+B 的演示不受影响。音频通路单独收敛后再置 1。
localparam AUDIO_SD_EN = 1'b0;

// ★ 流式方案下不再需要闸门：扫描随时可以开始，音频自己找空档填缓冲。

// The streaming design has no boot-time loader.  Keep the old status names
// tied to benign constants so the rest of A/B's top-level code remains
// untouched.
assign aud_hold       = 1'b0;
assign aud_load_start = 1'b0;
assign aud_load_done  = 1'b0;
assign aud_data_ok    = audio_i2s_data_ok;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                 audio_load_kicked <= 1'b0;
    else                        audio_load_kicked <= 1'b0;
end


// ----------------------------------------------------------------------
//  SDRAM 仲裁器：插在 frame_read_write 和 sdram 之间
// ----------------------------------------------------------------------
//  ★ 已移除 audio_sdram_shim：音频不再走 SDRAM，SDRAM 100% 归 A 的图片通路。


// ----------------------------------------------------------------------
//  频谱 / 电平 / LED3
// ----------------------------------------------------------------------
// ★★★ 频谱模块开关 ★★★
//   1 = 打开频谱（占 1804 LUT + 13 个 DSP）
//   0 = 关掉（DSP 从 29 降到 16，让布局能过）
//
//   为什么要关：板上实测 phy 阶段报
//       RUN-8102 Incremental route failed / #slices 7360
//   综合资源 LUT 12317(63%)、DSP 29/29(100%) —— DSP 打满把布局挤爆了。
//   先把"图片 + 声音"跑通，频谱之后再以精简版加回来。
localparam SPECTRUM_EN = 1'b1;

generate
if (SPECTRUM_EN) begin : g_viz
    audio_viz #(
        .SPEC_Y_TOP (336)
    ) u_audio_viz (
        .clk            (video_clk),
        .rst            (rst_all),
        .audio_valid    (audio_valid),
        .audio_left_data(audio_left_data),
        .audio_right_data(audio_right_data),
        .sw_audio_en    (sw_audio_en),
        .self_test      (1'b0),
        .de             (de),
        .x              (vl_pix_x),
        .y              (vl_pix_y),
        .spec_rgb       (aud_spec_rgb),
        .spec_alpha     (aud_spec_alpha),
        .level_peak     (aud_level_peak),
        .level_disp     (aud_level_disp),
        .led_audio_act  (aud_led_act)
    );
end else begin : g_noviz
    assign aud_spec_rgb   = 24'd0;
    assign aud_spec_alpha = 8'd0;
    // 没有频谱时，LED3 用"有音频样点"粗略指示
    assign aud_led_act    = (audio_left_data != 24'sd0)
                          | (audio_left_data[23:16] != 8'h00);
end
endgenerate

// ======================================================================
//  [C 2026-10-23] 音频样点跨时钟域：audio_mclk(12.288MHz) -> video_clk(25MHz)
// ----------------------------------------------------------------------
//  为什么把 I2S_receiver 从数据通路里拿掉
//  ---------------------------------------------------------------------
//  旧结构是：并行 16bit -> audio_hdmi_i2s 串行化成 I2S ->
//            I2S_receiver 在 video_clk 域把 BCLK 采回来还原成 24bit。
//
//  问题是 video_clk / BCLK = 25MHz / 6.144MHz = **4.07 个时钟/位**，
//  而 I2S_receiver 用的是 3 级同步器 + 上升沿检测 —— 只有 1 个时钟的余量。
//  结果是周期性"滑码"：某一位被多采或少采一次，整个 24bit 字就整体移位 N 位。
//
//  板上表现（就是这次报的两个问题）：
//    · 声音内容完全正确、但小得离谱      —— 移位相当于除以 2^N
//    · 频谱往最高频堆、只有最右边偶尔跳一下
//                                       —— 滑码是冲激，频谱被推成宽带噪声
//
//  现在改成：
//    数据在 audio_mclk 域里稳定保持一个 48 kHz 帧（20.8 us），
//    用"翻转标志"过 3 级同步器，在 video_clk 域检测跳变后【再取已稳定的数据】。
//    20.8 us / 40 ns = 520 个 video_clk 的建立时间，物理上不可能采错。
//
//  I2S 串行口仍然保留（引到 audio_i2s_* 这三个线上），只是不再参与数据通路，
//  需要示波器看波形时还能用。
// ======================================================================
reg  [2:0]  aud_tog_sync;      // 3 级同步器（翻转标志）
reg  [23:0] aud_pcm_s1, aud_pcm_s2;
reg         aud_frame_tick;    // video_clk 域，48 kHz，单拍

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        aud_tog_sync     <= 3'd0;
        aud_pcm_s1       <= 24'sd0;
        aud_pcm_s2       <= 24'sd0;
        aud_frame_tick   <= 1'b0;
        audio_valid      <= 1'b0;
        audio_left_data  <= 24'sd0;
        audio_right_data <= 24'sd0;
    end else begin
        // 翻转标志 + 数据各自打拍。数据比标志晚 2 拍被采，
        // 而标志要 3 拍才被认出来，所以取到的数据一定已经稳定。
        aud_tog_sync <= {aud_tog_sync[1:0], audio_pcm_toggle_a};
        aud_pcm_s1   <= audio_pcm_data_a;
        aud_pcm_s2   <= aud_pcm_s1;

        aud_frame_tick <= aud_tog_sync[2] ^ aud_tog_sync[1];

        // audio_valid 保持单拍、48 kHz，和旧的 I2S_receiver 输出节拍完全一致，
        // 所以 audio_viz 的 TICK_RATE=48000、ACR 计算、HDMI 打包都不用改。
        audio_valid <= aud_frame_tick;
        if (aud_frame_tick) begin
            audio_left_data  <= aud_pcm_s2;
            audio_right_data <= aud_pcm_s2;   // 音源是单声道，左右同值
        end
    end
end

// ======================================================================
//  音量（Task B 参数项 5 / param_regs.volume）
//
//  位置选择：缩放放在【I2S_receiver 解出来的 PCM 上】，而不是改音源或接收器。
//  理由：
//    · 契约 §4.3 写明 I2S_receiver / HDMI 发送器只负责搬数据，不要动；
//    · 放在这里，将来 C 把音源换成真正的提示音/语音之后，音量照样有效 ——
//      音量属于"数据"，不属于某一个音源；
//    · audio_valid / ACR 完全不受影响（只改样点幅度，不改采样率与有效性），
//      所以音量=0 时是"数字静音"，HDMI 音频流依旧有效。
//
//  换算：out = saturate( pcm × volume / 128 )，volume 128 = 原音量（×1.0），
//        volume 0 = 静音，volume 255 ≈ ×2（音源只有 23.8% 满量程，×2 仍不会削顶）。
//  有符号注意：PCM 是 24 位有符号数，verilog 里靠 $signed 把形参声明成有符号，
//  否则会被当成无符号数做乘法、正负号全乱。中间量取 33 位：
//  |pcm| ≤ 2^23，×255 = 2,139,095,040，其实连 32 位有符号（上限 2,147,483,647）
//  都放得下，33 位只是留一点余量 —— 而"位宽 = 两操作数之和"这条规则要求乘法
//  放在足够宽的上下文里，否则会先截断再乘。
//  两端的饱和值写成位型（24'sh7FFFFF / 24'sh800000）：-2^23 在 24 位有符号里
//  恰好是 0x800000，写成 -24'sd8388608 要靠两次溢出相抵才对，容易被工具报
//  "literal out of range"。
// ======================================================================
function signed [23:0] vol_scale;
    input signed [23:0] s;          // 24 位有符号 PCM（形参是 signed，调用处的
    input        [7:0]  v;          // $signed 只是把位型说清楚，不是必需的）
    reg   signed [32:0] p;
    reg   signed [32:0] q;
    begin
        p = s * $signed({1'b0, v}); // 24 位有符号 × 9 位有符号 → 33 位有符号
        q = p;                      // [FIX v7] UNITY AT VOLUME 255: the 16-bit source is only sign-extended into 24 bits above, so full scale 16-bit is +-32767 out of +-2^23. The extra >>7 threw away 42 dB. volume 255 = exactly the sample, 128 = -6 dB. // ÷128，算术右移保符号（负数向 −∞ 取整）
        if      (q >  33'sd8388607) vol_scale =  24'sh7FFFFF;   // +2^23−1（正满量程）
        else if (q < -33'sd8388608) vol_scale =  24'sh800000;   // −2^23  （负满量程）
        else                        vol_scale = q[23:0];
    end
endfunction

wire signed [23:0] audio_l_scaled = vol_scale($signed(audio_left_data),  param_volume);
wire signed [23:0] audio_r_scaled = vol_scale($signed(audio_right_data), param_volume);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs),
    .I_de          (de),
    .I_rgb         (vout_data),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    // 音量：给核送的是【缩放后】的 PCM（见上面的 vol_scale）。
    // audio_valid / ACR 用原始信号，不受音量影响。
    .I_audio_left_data  (audio_l_scaled),
    .I_audio_right_data (audio_r_scaled),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

// ---------------------------------------------------------------------
//  (seg_digit_from0 / from2 / from3 were removed with the 7-segment
//   diagnostics they decoded -- those digits belong to B and C. See the
//   allocation table at the seg_scan instantiation near the top.)
// ---------------------------------------------------------------------

endmodule
