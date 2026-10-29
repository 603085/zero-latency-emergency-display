// ============================================================================
//  text_layer.v -- on-screen text overlay layer  (Task A / extension item (1))
//
//  Draws two rows of text over the video image:
//    row 1 : a free-running message from char_mem (scrolling warning line)
//    row 2 : "TIME xxxxx" -- a live BCD counter
//
//  Why a live counter?  It is the liveness indicator: while the number is
//  changing the viewer knows the screen has not frozen. A static picture on
//  an emergency panel makes people distrust it.
//
//  Readability is guaranteed by drawing a solid dark band behind each text
//  row instead of relying on the background image. This is what real subtitle
//  rendering does, and it works on any picture.
//
//  Geometry (pixels of the 640x480 frame):
//    character cell : CHAR_W (default 10) wide x CHAR_H (16) tall
//    glyph          : 8x16, left aligned inside the cell, from font_rom
//
//  Cost: LUT logic only (font_rom is a case statement, no BRAM, no multiplier).
// ============================================================================

`timescale 1ns / 1ps

module text_layer #(
    parameter H_ACTIVE  = 640,
    parameter V_ACTIVE  = 480,
    parameter CHAR_W    = 10,           // character pitch, pixels
    parameter CHAR_H    = 16,           // character cell height
    parameter TEXT_MAX  = 32,           // characters held in char_mem
    parameter ROW1_Y    = 424,          // top edge of text row 1
    parameter ROW2_Y    = 448,          // top edge of text row 2
    // Horizontal origins, computed by the caller as
    //     (H_ACTIVE - text_width) / 2
    // so each row is centred on its OWN width. MARGIN_X (a single shared
    // left margin) was replaced by these two: with one margin the message
    // sat hard against the left edge and was only centred by luck when it
    // happened to be as wide as the screen.
    parameter BASE1_X   = 240,          // message row   : (640 - 16*10)/2
    parameter BASE2_X   = 270,          // counter row   : (640 - 10*10)/2
    parameter COLOR_BG  = 24'h101820     // backdrop band colour (dark blue-grey)
)(
    input  wire        clk,
    input  wire        rst,

    // ---------------- pixel position (pixel clock domain) ----------------
    input  wire [9:0]  x,
    input  wire [9:0]  y,
    input  wire        de,              // wire this to de_r from video_delay

    // ---------------- runtime display parameters ----------------
    input  wire        text_en,         // master enable for this layer
    input  wire [2:0]  color_sel,       // 0=white 1=red 2=green 3=yellow 4=cyan
    // SIGNED, because the line now bounces and spends half its time off the
    // left edge with a negative offset. An unsigned port would turn that into
    // a very large positive number and the line would disappear instead of
    // scrolling out. See the note on x1_shifted below.
    input  wire signed [10:0] scroll_x, // horizontal scroll offset in pixels

    // backdrop colour, runtime selectable.
    // This used to be the compile-time parameter COLOR_BG. It became an input
    // because the colour presets (standard / emergency / night / high-contrast)
    // need to change the backdrop while the design is running. The parameter
    // below is kept as the reset/default value only.
    input  wire [23:0] bg_color,

    // ---------------- external character memory write port ----------------
    input  wire        char_we,         // write one character this cycle
    input  wire [4:0]  char_addr,       // destination index, 0..TEXT_MAX-1
    input  wire [7:0]  char_data,       // ASCII code
    input  wire        char_clr,        // clear the whole string to spaces

    // ---------------- liveness counter control ----------------
    input  wire        counter_en,      // let the BCD counter run
    input  wire        counter_rst,     // reset the counter to zero

    // ---------------- to the layer mixer ----------------
    output reg  [23:0] text_rgb,
    output reg  [7:0]  text_alpha,      // 0 = transparent, 255 = opaque

    // ---------------- exported counter digits ----------------
    // Given as real ports rather than hierarchical taps, so the 7-segment
    // display can be driven later (or the digits shown elsewhere) without
    // touching this module, and so no synthesis tool has to resolve a
    // cross-module reference. Packed {d4, d3, d2, d1, d0}, 4 bits each.
    output wire [19:0] time_digits
);

    // ==================================================================
    //  liveness BCD counter, 5 digits, ticks at 2 Hz from the pixel clock
    //
    //  WIDTH MATTERS HERE. 25 MHz / 12_500_000 = 2 Hz, and 12_500_000 needs
    //  24 bits (2^23 - 1 = 8_388_607 is not enough). An earlier version
    //  declared both sec_div and TICK_MAX as [22:0], so the constant was
    //  silently truncated to 4_054_656 and the counter ran about 3x too
    //  fast. TD flagged it as
    //      HDL-5007 WARNING: literal value 'd12_500_000 truncated to fit in 23 bits
    //  On the bench this looks like "the counter feels fast" and is very
    //  hard to spot by reading the code.
    // ==================================================================
    reg [24:0] sec_div;
    reg [3:0]  bcd0, bcd1, bcd2, bcd3, bcd4;

    //  2026-10-20：12_500_000(2Hz) 改成 25_000_000(1Hz)，并加宽到 25 位。
    //  25_000_000 需要 25 位（2^24 = 16,777,216 不够），写 24 位会被截断成
    //  其它值，表现就是"计数器走起来不对"。这个文件的注释早就警告过这一点。
    localparam [24:0] TICK_MAX = 25'd25_000_000;   // 25MHz/25M = 1Hz（真正的每秒加一）

    always @(posedge clk or posedge rst) begin
        if (rst || counter_rst) begin
            sec_div <= 25'd0;
            bcd0 <= 4'd0; bcd1 <= 4'd0; bcd2 <= 4'd0; bcd3 <= 4'd0; bcd4 <= 4'd0;
        end else if (counter_en) begin
            if (sec_div >= TICK_MAX) begin
                sec_div <= 25'd0;
                if (bcd0 == 4'd9) begin
                    bcd0 <= 4'd0;
                    if (bcd1 == 4'd9) begin
                        bcd1 <= 4'd0;
                        if (bcd2 == 4'd9) begin
                            bcd2 <= 4'd0;
                            if (bcd3 == 4'd9) begin
                                bcd3 <= 4'd0;
                                if (bcd4 == 4'd9) bcd4 <= 4'd0;
                                else              bcd4 <= bcd4 + 4'd1;
                            end else bcd3 <= bcd3 + 4'd1;
                        end else bcd2 <= bcd2 + 4'd1;
                    end else bcd1 <= bcd1 + 4'd1;
                end else bcd0 <= bcd0 + 4'd1;
            end else begin
                sec_div <= sec_div + 24'd1;
            end
        end
    end

    // ==================================================================
    //  character memory
    // ==================================================================
    reg [7:0] char_mem [0:TEXT_MAX-1];
    integer i;

    always @(posedge clk) begin
        if (char_clr) begin
            for (i = 0; i < TEXT_MAX; i = i + 1) char_mem[i] <= 8'h20;   // space
        end else if (char_we) begin
            char_mem[char_addr] <= char_data;
        end
    end

    // ==================================================================
    //  which text row is this pixel in?
    //
    //  NOTE: the glyph row is (y - ROW_Y), NOT y[3:0]. Taking y[3:0] only
    //  works when ROW_Y happens to be a multiple of 16 (the character cell
    //  height). 424 is not (424 = 26*16 + 8), and using y[3:0] there read
    //  glyph row 13 -- which is blank -- so the text never appeared.
    //  Subtracting keeps the module correct for ANY ROW_Y.
    // ==================================================================
    wire in_row1 = (y >= ROW1_Y[9:0]) && (y < (ROW1_Y[9:0] + CHAR_H[9:0]));
    wire in_row2 = (y >= ROW2_Y[9:0]) && (y < (ROW2_Y[9:0] + CHAR_H[9:0]));
    wire in_band = in_row1 || in_row2;

    // glyph row within the cell: 0..CHAR_H-1, computed by subtraction so it
    // does not depend on ROW_Y being aligned to the cell height
    wire [4:0] row1_sub = {1'b0, y} - {1'b0, ROW1_Y[9:0]};
    wire [4:0] row2_sub = {1'b0, y} - {1'b0, ROW2_Y[9:0]};

    wire [3:0] glyph_row1 = row1_sub[3:0];
    wire [3:0] glyph_row2 = row2_sub[3:0];

    // ==================================================================
    //  horizontal mapping
    //
    //  Each row has its OWN horizontal origin, because the two rows hold
    //  different amounts of text and both should look centred:
    //
    //     row 1  the message, width = MSG_LEN * CHAR_W
    //     row 2  the counter, always 10 characters wide ("TIME " + 5 digits)
    //
    //  BASE1_X / BASE2_X are computed by the caller as
    //     (H_ACTIVE - text_width) / 2
    //  so this module does not need to know the active width.
    //
    //  SIGNED ARITHMETIC ON x1_shifted IS LOAD-BEARING. scroll_x goes
    //  negative as the line leaves the left edge. If this expression were
    //  computed unsigned, the negative offset would be zero-extended to a
    //  huge positive value, cell1 would land past the end of the string, and
    //  the line would blink out instead of scrolling off smoothly. 12 bits
    //  signed covers the full span: x (0..1023) - BASE1_X + scroll_x, whose
    //  worst case is about -1100 .. +1900.
    //
    //  divide by CHAR_W: CHAR_W is a constant, so the synthesiser turns this
    //  into a multiply-shift by a magic constant -- no real divider.
    //  A negative cell1 simply fails the "cell1 < TEXT_MAX" test below and
    //  renders as a space, which is exactly the wanted behaviour off-screen.
    //
    //  WHY frac IS KEPT SIGNED, NOT TAKEN AS A 4-BIT UNSIGNED MODULO.
    //  Verilog's % returns a remainder with the SIGN OF THE DIVIDEND, not a
    //  mathematical modulo. With a negative scroll_x the dividend goes
    //  negative, and forcing the result into an unsigned 4-bit vector wrapped
    //  it: a true column of -3 came out as 13, and a true column of -1 came
    //  out as 15. That shifts the glyph by CHAR_W in the wrong direction at
    //  exactly the moment the line is crossing the left edge, which is the
    //  one moment it must look right.
    //
    //  Keeping frac signed and testing the range explicitly is both correct
    //  and cheaper than normalising: a negative column is simply off the left
    //  of the glyph, and anything >= 8 is off the right, and both are blank.
    // ==================================================================
    // ==================================================================
    //  ONE PIXEL OF OFFSET: ACCEPTED, DELIBERATELY
    //
    //  The pipeline register below delays everything after the cell decode by
    //  one clock. On real video, where x advances by one every clock, that
    //  puts the whole text layer one pixel to the RIGHT of where it used to
    //  be.
    //
    //  It is NOT compensated by advancing x here, and that is a considered
    //  choice rather than an oversight. Advancing x would restore the exact
    //  position, but it would also make this module correct ONLY while x
    //  counts -- a hidden coupling to the video timing that is invisible in
    //  the code and would quietly break any test that holds x still. Leaving
    //  the offset keeps the module a plain function of its inputs.
    //
    //  The cost is one pixel on a 640-pixel-wide overlay. There is no
    //  reference to compare the subtitle against, so it cannot be seen. The
    //  first active pixel of each line is blank either way, because its data
    //  would have to come from the horizontal blanking interval.
    // ==================================================================
    wire signed [11:0] x1_shifted = $signed({1'b0, x})
                                  - $signed({1'b0, BASE1_X[9:0]})
                                  + scroll_x;
    wire signed [11:0] x2_shifted = $signed({1'b0, x})
                                  - $signed({1'b0, BASE2_X[9:0]});

    wire signed [11:0] cell1 = x1_shifted / CHAR_W;
    wire signed [11:0] frac1 = x1_shifted % CHAR_W;
    wire signed [11:0] cell2 = x2_shifted / CHAR_W;
    wire signed [11:0] frac2 = x2_shifted % CHAR_W;

    // ==================================================================
    //  *** THE PIPELINE REGISTER -- THIS IS THE TIMING FIX ***
    //
    //  WHY IT IS HERE. The render path
    //
    //      scroll_x -> x1_shifted -> cell1 (/CHAR_W) -> char_mem -> font_rom
    //               -> text_alpha -> layer_mixer blend -> vout_rgb
    //
    //  was ONE combinational chain of 29 logic levels, and it did not meet
    //  25 MHz on the board:
    //
    //      video_clk  SWNS -0.185 ns   Fmax 24.885 MHz   8 failed endpoints
    //      Begin : u_scroll_timer/pos_reg[2]
    //      End   : u_layer_mixer/vout_rgb_reg[13]
    //      Logic Level : 29        net delay 52% of the total
    //
    //  Cutting it here splits it roughly in half. The front half is the
    //  address arithmetic (the adders plus the divide-by-CHAR_W); the back half
    //  is the glyph lookup and the layer blending. Neither half is close to a
    //  40 ns budget on its own.
    //
    //  WHAT MUST BE REGISTERED TOGETHER, AND WHY. frac (the column inside the
    //  glyph), the glyph row, and de/in_band all describe the SAME pixel. If
    //  only the character code were delayed, the glyph of one pixel would be
    //  combined with the column and row of the next -- the text would shear,
    //  not just shift. So the whole cell/column/row bundle crosses together.
    // ==================================================================
    reg signed [11:0] cell1_q, frac1_q, cell2_q, frac2_q;
    reg [3:0]         grow1_q, grow2_q;
    reg               de_q, band_q, row2_q, en_q;

    always @(posedge clk) begin
        cell1_q <= cell1;
        frac1_q <= frac1;
        cell2_q <= cell2;
        frac2_q <= frac2;
        grow1_q <= glyph_row1;
        grow2_q <= glyph_row2;
        de_q    <= de;
        band_q  <= in_band;
        row2_q  <= in_row2;
        en_q    <= text_en;
    end

    // a column is drawable when it is a real column of the 8-pixel glyph.
    // Computed from the REGISTERED frac, so it belongs to the same pixel as
    // the glyph bits it selects.
    wire col1_ok = (frac1_q >= 12'sd0) && (frac1_q < 12'sd8);
    wire col2_ok = (frac2_q >= 12'sd0) && (frac2_q < 12'sd8);

    // ==================================================================
    //  character codes
    // ==================================================================
    reg [7:0] ch1;
    always @(*) begin
        // ==========================================================
        //  BOTH BOUNDS MUST BE CHECKED. ONLY THE UPPER ONE WAS.
        //
        //  cell1 is SIGNED and goes NEGATIVE as the line moves left: a pixel
        //  by the left edge has cell1 = (x - BASE1_X + scroll_x)/10, which is
        //  negative whenever x is left of the text's current position.
        //
        //  The old test was only `cell1 < TEXT_MAX`, which a negative value
        //  passes. char_mem was then indexed with cell1[4:0], and a negative
        //  index WRAPS: cell1 = -23 becomes (-23 & 31) = 9, so char_mem[9] --
        //  a real character -- was fetched and DRAWN far to the left of the
        //  message.
        //
        //  That is exactly the artifact seen on the board: a short row of thin
        //  vertical marks to the LEFT of "KEEP CLEAR EXIT". A whole-row dump
        //  found them as opaque pixels in cells -23..-17, each wrapping to a
        //  different cell of the message, which is why the marks differed in
        //  width instead of repeating.
        //
        //  Requiring cell1 >= 0 renders those pixels as spaces, which is what
        //  "off the left of the text" must mean.
        // ==========================================================
        if ((cell1_q >= 12'sd0) && (cell1_q < $signed(TEXT_MAX[9:0])))
            ch1 = char_mem[cell1_q[4:0]];
        else
            ch1 = 8'h20;                        // space outside the string
    end

    reg [7:0] ch2;
    always @(*) begin
        case (cell2_q)
            10'd0: ch2 = "T";
            10'd1: ch2 = "I";
            10'd2: ch2 = "M";
            10'd3: ch2 = "E";
            10'd4: ch2 = " ";
            10'd5: ch2 = 8'h30 + {4'd0, bcd4};
            10'd6: ch2 = 8'h30 + {4'd0, bcd3};
            10'd7: ch2 = 8'h30 + {4'd0, bcd2};
            10'd8: ch2 = 8'h30 + {4'd0, bcd1};
            10'd9: ch2 = 8'h30 + {4'd0, bcd0};
            default: ch2 = 8'h20;
        endcase
    end

    // ==================================================================
    //  glyph lookup
    // ==================================================================
    wire [7:0] bits1, bits2;

    font_rom u_font1 (
        .ch       (ch1[6:0]),
        .row      (grow1_q),
        .row_bits (bits1)
    );

    font_rom u_font2 (
        .ch       (ch2[6:0]),
        .row      (grow2_q),
        .row_bits (bits2)
    );

    // pixel is on when the glyph column (0..7) has a set bit.
    // bit[7] of row_bits is the left-most pixel.
    // frac1_q/frac2_q are the within-cell column for each row (they can differ
    // because the two rows have different origins), and they are SIGNED --
    // see the note where they are declared. col1_ok / col2_ok reject the
    // columns that fall outside the glyph instead of wrapping them back in.
    wire px1 = col1_ok ? bits1[3'd7 - frac1_q[2:0]] : 1'b0;
    wire px2 = col2_ok ? bits2[3'd7 - frac2_q[2:0]] : 1'b0;

    // ==================================================================
    //  foreground colour selection
    // ==================================================================
    reg [23:0] fg_col;
    always @(*) begin
        case (color_sel)
            3'd0: fg_col = 24'hFFFFFF;   // white
            3'd1: fg_col = 24'hFF3030;   // red
            3'd2: fg_col = 24'h30FF30;   // green
            3'd3: fg_col = 24'hFFD24A;   // yellow
            3'd4: fg_col = 24'h60D0FF;   // cyan
            default: fg_col = 24'hFFFFFF;
        endcase
    end

    // ==================================================================
    //  export the counter digits
    // ==================================================================
    assign time_digits = {bcd4, bcd3, bcd2, bcd1, bcd0};

    // ==================================================================
    //  output -- GLYPH PIXELS ONLY, no backdrop band
    // ==================================================================
    //  This used to paint a solid dark rectangle across the full width of both
    //  rows and call it readable. It is not: the band covers the picture, and
    //  on a signage panel the picture is the information. Real subtitle
    //  rendering draws only the glyphs.
    //
    //  So text_alpha is now set per PIXEL. It is 255 on a glyph pixel and 0
    //  everywhere else, including the gaps inside a character cell and the
    //  space between characters. The image behind shows through untouched.
    //
    //  The consequence to be aware of: the text no longer has a guaranteed
    //  contrast background, so it can sit on a bright part of a picture and be
    //  harder to read. That is the accepted trade for an unobstructed image.
    // ==================================================================
    always @(*) begin
        text_rgb   = fg_col;
        text_alpha = 8'd0;

        // the _q versions: the whole layer is one pixel deep now, so the
        // gating flags have to be delayed with the rest of the pixel or the
        // band edges would land one pixel off
        if (en_q && de_q && band_q) begin
            if (row2_q) begin
                if (px2) begin
                    text_rgb   = fg_col;
                    text_alpha = 8'd255;
                end
            end else begin
                if (px1) begin
                    text_rgb   = fg_col;
                    text_alpha = 8'd255;
                end
            end
        end
    end

endmodule
