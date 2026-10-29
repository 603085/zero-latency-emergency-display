// ============================================================================
//  osd_bar.v -- parameter OSD bar  (Task B / extension item (4))
//
//  Interface FROZEN by doc/A包说明/TaskB开发指南.md section 4. Do not change
//  the port list without telling B and A.
//
//  Draws, in the top-left corner of the 640x480 frame:
//
//      +--------------------------------------+
//      | BRIGHT           32                  |   <- name + current value
//      | #########--------------------------- |   <- value bar
//      +--------------------------------------+
//
//  GEOMETRY (agreed so that B, C and A never overlap -- see 图 3 of
//  doc/功能分块与框图.md):
//      B's OSD        y =  16 ..  79   x = 20 .. 275
//      C's spectrum   y = 360 .. 407
//      A's subtitles  y = 424 / 448
//
//  TEXT comes from A's font_rom (8x16 glyph cell, rows 0..6 carry the glyph
//  body, bit[7] of row_bits is the LEFT-most pixel). Do NOT add a second font.
//
//  PIXEL ALIGNMENT AND TIMING
//  --------------------------
//  PIPELINE
//  --------
//      stage 1 (pos)     : x/y -> panel test, relative position, row/column
//      stage 1 (content) : value/param/preset -> raw latched values
//      stage 2 (content) : -> BCD digits and bar length
//      output (comb)     : glyph lookup, bit select, colour mux
//
//  Position stops at stage 1; content goes one stage deeper. That is not a
//  mistake: position says WHERE a pixel is drawn and must match the pixel
//  layer_mixer is presenting, content says WHICH GLYPH appears there and may
//  lag without moving anything.
//
//  THE POSITION PATH CARRIES EXACTLY ONE REGISTER, AND THAT COSTS ONE PIXEL.
//
//  layer_mixer pipes the picture combinationally through all seven of its
//  stages and registers the result once, with the OSD muxed in at the very
//  end. So the OSD is consumed in the SAME cycle as the pixel it must sit on,
//  and every register between x/y and that mux shifts the whole overlay one
//  pixel to the right (and one line down).
//
//  One stage is kept anyway, because the alternative is a single combinational
//  chain from x/y through the glyph lookup to the mixer's output register --
//  which is what text_layer measured at 29 logic levels before it had to add
//  its own register (video_clk SWNS -0.185 ns, 8 failed endpoints). The
//  content path (digits, bar length) is registered one stage deeper because it
//  does not depend on x/y at all: it may lag without moving anything.
//
//  The one-pixel offset is deliberately NOT compensated by pre-advancing x.
//  text_layer explains why (:208-226) and this module follows the same rule:
//  advancing x would make the module correct only while x is counting, which
//  is a hidden coupling to the video timing and would silently break any test
//  that holds x still. Position is therefore "agreed geometry + 1 px", which
//  is invisible on an overlay that has nothing to be compared against.
//
//  ALPHA IS BINARY IN THIS DESIGN
//  ------------------------------
//  layer_mixer's OSD stage is a multiplexer on (osd_alpha == 255), not a
//  blender -- deliberate, it keeps 3 multipliers out of the pixel path. A
//  partial osd_alpha therefore CANNOT blend here. This module treats
//  osd_alpha >= 128 as "on" and anything below as "off", so param_regs'
//  250 ms fade-out becomes a hard cut at its midpoint. For a true fade either
//  (a) make layer_mixer's stage 7 a blend8() like the transition stage, or
//  (b) fade this panel's colours towards the picture -- (a) is the honest fix.
//
//  COST: LUT logic only (two font_rom instances, small comparators, constant
//  divisions). No BRAM.
// ============================================================================

`timescale 1ns / 1ps

module osd_bar #(
    // ---------------- panel box (absolute frame coordinates) ----------------
    parameter [9:0] OSD_X = 10'd20,
    parameter [9:0] OSD_Y = 10'd16,
    parameter [9:0] OSD_W = 10'd256,      // 20 .. 275
    parameter [9:0] OSD_H = 10'd64,       // 16 ..  79

    // ---------------- layout inside the panel ------------------------------
    parameter [9:0] NAME_X  = 10'd24,     // left edge of the 6-character name
    parameter [9:0] TEXT_Y  = 10'd24,     // top of the 16-row text line
    parameter [9:0] VALUE_R = 10'd268,    // RIGHT edge of the 5-character value
    parameter [9:0] BAR_X   = 10'd24,     // left edge of the bar
    parameter [9:0] BAR_Y   = 10'd56,     // top of the bar
    parameter [9:0] BAR_W   = 10'd248,    // bar length, pixels (24 .. 271)
    parameter [9:0] BAR_H   = 10'd10,

    // ---------------- colours ---------------------------------------------
    // These are FINAL colours: layer_mixer puts the OSD ABOVE the
    // brightness/contrast stage on purpose, so the bar stays readable while
    // the operator turns the picture down. Do not pre-dim them here.
    parameter [23:0] COLOR_PANEL = 24'h101820,
    parameter [23:0] COLOR_FRAME = 24'h96A0AC,
    parameter [23:0] COLOR_TEXT  = 24'hFFFFFF,
    parameter [23:0] COLOR_TRACK = 24'h39424E,
    parameter [23:0] COLOR_FILL  = 24'h3FD0FF
)(
    input  wire        clk,               // video_clk, 25 MHz
    input  wire        rst,               // rst_all, active high
    input  wire [9:0]  x,                 // pixel coordinates, same domain as de
    input  wire [9:0]  y,
    input  wire        de,

    // ---------------- from param_regs (person B) --------------------------
    input  wire        osd_visible,       // 1 = the bar should be on screen
    input  wire [7:0]  osd_alpha,         // 0..255; >= 128 counts as on (see header)
    input  wire [2:0]  cur_param,         // 0 亮度 1 对比度 2 转场 3 停留 4 预设 5 音量
    input  wire [7:0]  osd_value,         // the value of cur_param, as an 8-bit code
    input  wire [7:0]  brightness,        // context, reserved for a future 2-row layout
    input  wire [7:0]  contrast,          // context, reserved
    input  wire [1:0]  transition,        // 0 = cut, 1 = fade, 2 = iris
    input  wire [15:0] dwell_ms,          // 200 .. 10000
    input  wire [1:0]  preset,            // 0 std 1 emg 2 night 3 high contrast

    // ---------------- to layer_mixer --------------------------------------
    output reg  [23:0] osd_rgb,
    output reg  [7:0]  osd_a              // 0 = transparent, 255 = opaque
);

    // ==================================================================
    //  relative layout inside the panel (constants, in panel coordinates)
    // ==================================================================
    localparam [9:0] NAME_REL   = NAME_X  - OSD_X;   //  4
    localparam [9:0] VALUE_REL  = (VALUE_R - 10'd40) - OSD_X;  // 208 (5 chars x 8)
    localparam [9:0] TEXT_REL   = TEXT_Y  - OSD_Y;   //  8
    localparam [9:0] BAR_REL_X  = BAR_X   - OSD_X;   //  4
    localparam [9:0] BAR_REL_Y  = BAR_Y   - OSD_Y;   // 40

    // ==================================================================
    //  GEOMETRY (combinational, straight from x / y)
    // ==================================================================
    wire in_panel = (x >= OSD_X) && (x < (OSD_X + OSD_W)) &&
                    (y >= OSD_Y) && (y < (OSD_Y + OSD_H));

    // Kept relative so the output stage only needs cheap compares, and so the
    // whole position bundle can be registered in one go.
    wire [9:0] rx = x - OSD_X;
    wire [9:0] ry = y - OSD_Y;

    // dwell_ms is clamped here, once, so nothing downstream has to worry about
    // param_regs ever presenting 0 or something above its documented ceiling.
    wire [15:0] dwell_clamped = (dwell_ms > 16'd10000) ? 16'd10000 : dwell_ms;

    wire visible_now = osd_visible && (osd_alpha >= 8'd128);

    // ---------------- stage 1 registers --------------------------------
    reg        de_r1, panel_r1, vis_r1;
    reg [9:0]  rx_r1, ry_r1;
    reg [7:0]  val_r1;
    reg [15:0] dwell_r1;
    reg [2:0]  param_r1;
    reg [1:0]  trans_r1, preset_r1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            de_r1    <= 1'b0;
            panel_r1 <= 1'b0;
            vis_r1   <= 1'b0;
            rx_r1    <= 10'd0;
            ry_r1    <= 10'd0;
            val_r1   <= 8'd0;
            dwell_r1 <= 16'd0;
            param_r1 <= 3'd0;
            trans_r1 <= 2'd0;
            preset_r1<= 2'd0;
        end else begin
            de_r1    <= de;
            panel_r1 <= in_panel;
            vis_r1   <= visible_now;
            rx_r1    <= rx;
            ry_r1    <= ry;
            val_r1   <= osd_value;
            dwell_r1 <= dwell_clamped;
            param_r1 <= cur_param;
            trans_r1 <= transition;
            preset_r1<= preset;
        end
    end

    // ==================================================================
    //  CONTENT, stage 1 -> 2 : decimal digits and bar length
    //
    //  Divisor is always a constant, so this is synthesizable shift-and-add
    //  logic (never a variable divider). It is deliberately the ONLY heavy
    //  logic in this stage.
    // ==================================================================
    // 8-bit value -> 3 decimal digits (brightness, contrast)
    wire [7:0]  v8   = val_r1;
    wire [7:0]  h    = v8 / 8'd100;
    wire [7:0]  hr   = v8 % 8'd100;
    wire [7:0]  t    = hr / 8'd10;
    wire [7:0]  u    = hr % 8'd10;

    // 16-bit value -> 5 decimal digits (dwell_ms, 0..10000)
    wire [15:0] dw   = dwell_r1;
    wire [15:0] tt   = dw / 16'd10000;
    wire [15:0] tr   = dw % 16'd10000;
    wire [15:0] th   = tr / 16'd1000;
    wire [15:0] thr  = tr % 16'd1000;
    wire [15:0] hu   = thr / 16'd100;
    wire [15:0] hur  = thr % 16'd100;
    wire [15:0] te   = hur / 16'd10;
    wire [15:0] one  = hur % 16'd10;

    // bar length, 0..255. Two of the five parameters are NOT quantities, so
    // the bar is an INDICATOR for them rather than a scale -- say so here so
    // nobody later "fixes" the numbers:
    //   bright / contrast : the value itself, 0..255 (a real scale)
    //   dwell             : 200..10000 mapped to 5..250, so the bar covers the
    //                       parameter's whole range instead of sitting empty
    //   transition        : three discrete marks 0 / 128 / 255 = cut / fade /
    //                       iris. NOTE iris draws a full bar but this build has
    //                       USE_IRIS = 0, so that mode is selectable and shown
    //                       yet cannot be displayed -- that is a property of
    //                       the transition, not of the OSD.
    //   preset            : four ordinal marks 0 / 85 / 170 / 255 = which of
    //                       the four colour presets is active. Ordinal, not
    //                       proportional: it answers "which one", not "how
    //                       much".
    wire [15:0] bar_len16 =
        (param_r1 == 3'd3) ? (dw / 16'd40) :
        (param_r1 == 3'd4) ? ({14'd0, preset_r1} * 16'd85) :
        (param_r1 == 3'd2) ? ((trans_r1 == 2'd0) ? 16'd0 :
                              (trans_r1 == 2'd1) ? 16'd128 : 16'd255) :
        {8'd0, v8};
    wire [7:0]  bar_len = bar_len16[7:0];

    // ---------------- stage 2 registers (content only) -----------------
    //  POSITION IS NOT REGISTERED HERE. Position must cross the mux in the
    //  cycle the pixel is presented, so it gets exactly ONE register stage
    //  (stage 1). The CONTENT (digits, bar length, which parameter) does not
    //  depend on x/y at all, so it may lag one more clock -- the only effect
    //  is that for a single pixel clock after a value changes the old glyph is
    //  still latched, which is invisible at human update rates.
    //
    //  WHY POSITION GETS ONLY ONE STAGE: layer_mixer pipes the picture
    //  combinationally through all seven stages and registers the result once
    //  (:403), with the OSD muxed in at the end (:394). Every register between
    //  x/y and that mux therefore delays the overlay by one whole pixel.
    //  text_layer states the same rule and accepts the resulting one-pixel
    //  offset (:208-226) rather than pre-advancing x, because pre-advancing
    //  would make the module correct only while x happens to be counting.
    //  This module follows that decision: ONE stage, ONE pixel, not
    //  compensated.
    reg  [3:0]  h_r2, t_r2, u_r2;
    reg  [3:0]  tt_r2, th_r2, hu_r2, te_r2, one_r2;
    reg  [7:0]  bar_r2;
    reg  [2:0]  param_r2;
    reg  [1:0]  trans_r2, preset_r2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            h_r2 <= 4'd0; t_r2 <= 4'd0; u_r2 <= 4'd0;
            tt_r2 <= 4'd0; th_r2 <= 4'd0; hu_r2 <= 4'd0; te_r2 <= 4'd0; one_r2 <= 4'd0;
            bar_r2 <= 8'd0;
            param_r2 <= 3'd0; trans_r2 <= 2'd0; preset_r2 <= 2'd0;
        end else begin
            h_r2 <= h[3:0]; t_r2 <= t[3:0]; u_r2 <= u[3:0];
            tt_r2 <= tt[3:0]; th_r2 <= th[3:0]; hu_r2 <= hu[3:0];
            te_r2 <= te[3:0]; one_r2 <= one[3:0];
            bar_r2 <= bar_len;
            param_r2 <= param_r1; trans_r2 <= trans_r1; preset_r2 <= preset_r1;
        end
    end

    // ==================================================================
    //  OUTPUT (combinational) : glyph decode and colour mux
    // ==================================================================
    // ---------------- which cell of which row are we in? ----------------
    // NOTE: these use the ONE-stage position (rx_r1/ry_r1), not the two-stage
    // content registers. See the stage-2 block above for why.
    wire [9:0] name_dx = rx_r1 - NAME_REL;
    wire [9:0] value_dx= rx_r1 - VALUE_REL;

    // TEXT_REL + 16 is the 16-ROW CELL, not the visible glyph: font_rom only
    // fills rows 0..6 of the cell and leaves 7..15 blank for line spacing, so
    // the ink actually covers relative y 8..14 (absolute 24..30). That is the
    // same 5x7-in-an-8x16-cell arrangement text_layer uses; the lower half of
    // the band is intentionally empty.
    wire in_text_row = (ry_r1 >= TEXT_REL) && (ry_r1 < (TEXT_REL + 10'd16));
    wire in_name     = (name_dx  < 10'd48) && in_text_row;   // 6 chars x 8 px
    wire in_value    = (value_dx < 10'd40) && in_text_row;   // 5 chars x 8 px

    // NAME_REL / VALUE_REL are multiples of 4 but not of 8, so the subtract
    // has to happen BEFORE the column bits are taken -- otherwise the glyph
    // would be cut in half.
    //
    // [5:3] rather than [8:3] on purpose: the cell index only ever reaches 5
    // (6 characters) and 4 (5 characters), and taking the narrower slice keeps
    // the assignment the same width as the target, so no tool has to warn
    // about a truncated literal.
    wire [2:0] name_col = name_dx[2:0];
    wire [2:0] name_idx = name_dx[5:3];
    wire [2:0] val_col  = value_dx[2:0];
    wire [2:0] val_idx  = value_dx[5:3];

    // glyph row inside the 16-row cell. 4-bit arithmetic wraps on purpose:
    // ry 8..15 -> 0..7 and ry 16..23 -> 8..15, both correct.
    wire [3:0] grow = ry_r1[3:0] - 4'd8;

    // bit[7] of row_bits is the LEFT-most pixel, so column c uses bit (7-c).
    // The subtraction cannot underflow: name_col / val_col are 0..7.
    wire [2:0] name_bit = 3'd7 - name_col;
    wire [2:0] val_bit  = 3'd7 - val_col;

    // ---------------- parameter name, 6 characters ----------------
    // 0..4 是《TaskB开发指南》冻结的五项；5 = 音量是本工程新增的第 6 项
    // （param_regs 里同步加了 volume 端口，见那份文件头的说明）。
    reg [47:0] name_bits;
    always @(*) begin
        case (param_r2)
            3'd0:    name_bits = "BRIGHT";
            3'd1:    name_bits = "CONTRA";
            3'd2:    name_bits = "TRANS ";
            3'd3:    name_bits = "DWELL ";
            3'd4:    name_bits = "PRESET";
            3'd5:    name_bits = "VOLUME";
            // 未定义项给全空格，不要用 default 冒充某一项 —— 将来加第 7 项时
            // default 会让它静默显示成"音量"。
            default: name_bits = "      ";
        endcase
    end

    reg [7:0] name_ch;
    always @(*) begin
        case (name_idx)
            3'd0:    name_ch = name_bits[47:40];
            3'd1:    name_ch = name_bits[39:32];
            3'd2:    name_ch = name_bits[31:24];
            3'd3:    name_ch = name_bits[23:16];
            3'd4:    name_ch = name_bits[15:8];
            default: name_ch = name_bits[7:0];
        endcase
    end

    // ---------------- value, 5 characters, right aligned ----------------
    // 0 亮度 / 1 对比度 / 5 音量 : 3 位十进制，前面补空格
    // 2 transition          : CUT / FADE / IRIS
    // 3 dwell               : 5 digits, leading zeros blanked
    // 4 preset              : STD / EMG / NIGHT / HI
    reg [7:0] value_ch;
    always @(*) begin
        case (param_r2)
            3'd0, 3'd1, 3'd5: begin
                case (val_idx)
                    3'd0:    value_ch = 8'h20;
                    3'd1:    value_ch = 8'h20;
                    3'd2:    value_ch = 8'h30 + {4'd0, h_r2};
                    3'd3:    value_ch = 8'h30 + {4'd0, t_r2};
                    default: value_ch = 8'h30 + {4'd0, u_r2};
                endcase
            end
            3'd2: begin
                case (val_idx)
                    3'd0:    value_ch = (trans_r2 == 2'd0) ? 8'h43 : ((trans_r2 == 2'd1) ? 8'h46 : 8'h49); // C F I
                    3'd1:    value_ch = (trans_r2 == 2'd0) ? 8'h55 : ((trans_r2 == 2'd1) ? 8'h41 : 8'h52); // U A R
                    3'd2:    value_ch = (trans_r2 == 2'd0) ? 8'h54 : ((trans_r2 == 2'd1) ? 8'h44 : 8'h49); // T D I
                    3'd3:    value_ch = (trans_r2 == 2'd1) ? 8'h45 : 8'h20;                                // E or blank
                    default: value_ch = (trans_r2 == 2'd2) ? 8'h53 : 8'h20;                                // S or blank
                endcase
            end
            3'd3: begin
                case (val_idx)
                    3'd0:    value_ch = (tt_r2  == 4'd0) ? 8'h20 : (8'h30 + {4'd0, tt_r2});
                    3'd1:    value_ch = ((tt_r2 == 4'd0) && (th_r2 == 4'd0)) ? 8'h20 : (8'h30 + {4'd0, th_r2});
                    3'd2:    value_ch = ((tt_r2 == 4'd0) && (th_r2 == 4'd0) && (hu_r2 == 4'd0)) ? 8'h20 : (8'h30 + {4'd0, hu_r2});
                    3'd3:    value_ch = ((tt_r2 == 4'd0) && (th_r2 == 4'd0) && (hu_r2 == 4'd0) && (te_r2 == 4'd0)) ? 8'h20 : (8'h30 + {4'd0, te_r2});
                    default: value_ch = 8'h30 + {4'd0, one_r2};
                endcase
            end
            3'd4: begin
                // STD / EMG / NIGHT / HI
                case (preset_r2)
                    2'd0: begin   // "STD  "
                        case (val_idx)
                            3'd0:    value_ch = 8'h53;
                            3'd1:    value_ch = 8'h54;
                            3'd2:    value_ch = 8'h44;
                            default: value_ch = 8'h20;
                        endcase
                    end
                    2'd1: begin   // "EMG  "
                        case (val_idx)
                            3'd0:    value_ch = 8'h45;
                            3'd1:    value_ch = 8'h4D;
                            3'd2:    value_ch = 8'h47;
                            default: value_ch = 8'h20;
                        endcase
                    end
                    2'd2: begin   // "NIGHT"
                        case (val_idx)
                            3'd0:    value_ch = 8'h4E;
                            3'd1:    value_ch = 8'h49;
                            3'd2:    value_ch = 8'h47;
                            3'd3:    value_ch = 8'h48;
                            default: value_ch = 8'h54;
                        endcase
                    end
                    default: begin // "HI   "
                        case (val_idx)
                            3'd0:    value_ch = 8'h48;
                            3'd1:    value_ch = 8'h49;
                            default: value_ch = 8'h20;
                        endcase
                    end
                endcase
            end
            // 未定义项：整格空白。不要用 default 冒充某一项 ——
            // 将来加第 7 项时 default 会让它静默显示成预设名。
            default: value_ch = 8'h20;
        endcase
    end

    // ---------------- glyph lookups (A's font_rom, combinational) -------
    wire [7:0] name_bits_row;
    wire [7:0] value_bits_row;

    font_rom u_font_name (
        .ch       (name_ch[6:0]),
        .row      (grow),
        .row_bits (name_bits_row)
    );

    font_rom u_font_value (
        .ch       (value_ch[6:0]),
        .row      (grow),
        .row_bits (value_bits_row)
    );

    wire name_px  = name_bits_row[name_bit];
    wire value_px = value_bits_row[val_bit];

    // ---------------- bar ----------------
    wire in_bar  = (rx_r1 >= BAR_REL_X) && (rx_r1 < (BAR_REL_X + BAR_W)) &&
                   (ry_r1 >= BAR_REL_Y) && (ry_r1 < (BAR_REL_Y + BAR_H));
    // Full width, no [7:0] slice: slicing a 10-bit parameter to 8 bits would
    // silently truncate the moment anyone raises BAR_W above 255.
    wire [15:0] fill = (bar_r2 * BAR_W) >> 8;         // 0 .. 247 for BAR_W=248
    wire bar_px  = ((rx_r1 - BAR_REL_X) < fill);

    // ---------------- panel frame (1 px, keeps the box visible on light
    //                  pictures even when the panel colour is dark) --------
    wire frame_px = (rx_r1 == 10'd0) || (rx_r1 == (OSD_W - 10'd1)) ||
                    (ry_r1 == 10'd0) || (ry_r1 == (OSD_H - 10'd1));

    // ---------------- colour mux ----------------
    always @(*) begin
        osd_rgb = 24'h000000;
        osd_a   = 8'd0;

        // de_r1 / vis_r1 / panel_r1 and the *_r2 content registers are one
        // stage apart on purpose -- position decides WHERE, content decides
        // WHICH GLYPH, and those two never need to be in lock-step.
        if (de_r1 && vis_r1 && panel_r1) begin
            osd_a = 8'd255;
            if      (in_name  && name_px)  osd_rgb = COLOR_TEXT;
            else if (in_value && value_px) osd_rgb = COLOR_TEXT;
            else if (in_bar)               osd_rgb = bar_px ? COLOR_FILL : COLOR_TRACK;
            else if (frame_px)             osd_rgb = COLOR_FRAME;
            else                           osd_rgb = COLOR_PANEL;
        end
    end

    // brightness / contrast are part of the frozen port list but the 2-row
    // layout above does not need them yet (the bar and the value both come
    // from osd_value). They are deliberately left unused rather than removed,
    // so that B's wiring does not have to change when a second row is added.

endmodule
