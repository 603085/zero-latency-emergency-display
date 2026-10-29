// ============================================================================
//  scroll_timer.v -- bouncing scroll offset generator  (Task A)
//
//  Produces the horizontal scroll offset for the subtitle line, plus the
//  character-fill handshake that loads the message into text_layer's char_mem.
//
//  Why the fill handshake exists: text_layer exposes a simple character write
//  port (char_we / char_addr / char_data). Something has to walk the message
//  string once and push it in. Doing it here keeps text_layer purely a
//  renderer, and means the message can be replaced later without touching the
//  rendering path.
//
//  Scrolling model -- BOUNCE OFF THE SCREEN EDGES
//  ----------------------------------------------
//  Requested behaviour:
//
//      the line starts in the MIDDLE, travels left until it meets the left
//      edge, reverses, travels right until it meets the right edge, and
//      repeats -- always fully on screen
//
//  So the offset is BIPOLAR and the motion is a triangle wave:
//
//      scroll_x = +BASE_OFF   the text just touches the right edge
//      scroll_x =       0     the text is centred (where it also starts)
//      scroll_x = -BASE_OFF   the text just touches the left edge
//
//  BASE_OFF is (H_ACTIVE - text width) / 2, i.e. exactly the margin the caller
//  leaves on each side, so the line stops at the edges instead of running off
//  them. An earlier version used +/- (H_ACTIVE + TEXT_W), which is where the
//  line is COMPLETELY off screen; the bounce then happened out of sight and the
//  subtitle looked absent for most of every cycle.
//
//  WHY SIGNED ARITHMETIC IS NOT OPTIONAL HERE. A negative offset held in an
//  unsigned vector is a very large positive number. `x` is unsigned, so a
//  negative scroll_x would be extended with zeros during the addition in
//  text_layer, x - BASE1_X + scroll_x would wrap to a huge value, the
//  character index would come out past the end of the string, and the line
//  would simply vanish instead of scrolling off the left edge. Every operand
//  on the path to the character index is therefore signed.
//
//  The old version wrapped at one screen width and jumped back to zero, which
//  reads as a glitch rather than as motion. There is no wrap any more.
//
//  Parameters are compile-time so the synthesiser folds the comparators.
// ============================================================================

`timescale 1ns / 1ps

module scroll_timer #(
    parameter MSG_LEN      = 32,            // characters in the message
    // Depth of the message memory. Must be at least MSG_LEN; kept at
    // text_layer's TEXT_MAX so that the 5-bit write address coming from
    // uart_text indexes this array exactly, with no truncation anywhere.
    // MSG_LEN cannot exceed this without also widening text_layer's
    // char_addr, because character indices above TEXT_MAX-1 do not render.
    parameter TEXT_MAX     = 32,
    parameter CHAR_W       = 10,            // pixels per character cell
    parameter SCROLL_TICKS = 24'd5_000_000, // video clocks per pixel step
    parameter FILL_HOLD    = 10'd64,        // clocks the write port is held per char
    parameter H_ACTIVE     = 640,           // active pixels; sets the travel limit
    // How long the line RESTS CENTRED before the bounce starts, in video
    // clocks. 25 MHz x 2.5 s = 62_500_000.
    //
    // WHY IT RESTS AT ALL. The bounce begins at the right wall, which is
    // entirely OFF SCREEN. At the production scroll rate of 0.2 s per pixel
    // the line needs roughly 32 SECONDS to travel far enough to show a single
    // character -- so for the first half minute after power-up the subtitle is
    // invisible and the screen looks broken. Resting it centred first makes the
    // layer visibly present immediately, and the bounce then reads as a
    // deliberate sweep rather than as a delayed appearance.
    parameter [31:0] HOLD_TICKS = 32'd62_500_000,
    // Kept so existing instantiations still elaborate. The bouncing line
    // always moves, so this no longer selects anything -- see the note on
    // needs_scroll below.
    parameter SCROLL_ALWAYS = 0
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        scroll_en,           // 1 = keep moving
    input  wire        scroll_rst,          // 1 = jump back to the right wall

    // ---------------- message memory write port (from uart_text) ----------
    //
    // LEAVE THESE UNCONNECTED AND NOTHING CHANGES: an undriven input is Z,
    // `if (Z)` is false, so no write and no reload ever happen and the message
    // memory stays at its reset contents -- which are the original hard-coded
    // "KEEP CLEAR EXIT ". That is what keeps every existing testbench and the
    // whole scroll/bounce geometry untouched by the serial feature.
    input  wire        msg_we,          // write one character this cycle
    input  wire [4:0]  msg_waddr,       // index into the message memory
    input  wire [7:0]  msg_wdata,       // ASCII code
    input  wire        msg_reload,      // pulse: re-copy the message out

    // ---------------- how long the message actually is ----------------
    // Characters committed by uart_text (0 = nothing committed yet). THE BOUNCE
    // IS SIZED FROM THIS, not from MSG_LEN.
    //
    // WHY IT MATTERS. The travel limits are where the TEXT touches the screen
    // edges. With a fixed limit sized for a full-length message, a short
    // message would only ever sweep the left-hand part of the screen and would
    // look broken -- and the 16-character power-on message, which used to
    // bounce edge to edge, would stop doing so the moment MSG_LEN grew. Sizing
    // the limits from the real length keeps every message, short or long,
    // sweeping exactly between the two edges.
    input  wire [5:0]  used_len,

    // ---------------- message fill port to text_layer ----------------
    output reg         char_we,
    output reg  [4:0]  char_addr,
    output reg  [7:0]  char_data,

    // ---------------- scroll offset to text_layer ----------------
    // SIGNED. See the note in the header: this goes negative, and text_layer
    // must treat it as signed for the arithmetic to mean anything.
    output reg signed [10:0] scroll_x,

    output wire        fill_done,           // 1 once the whole message is loaded

    // ---------------- diagnostic ----------------
    // The first character of the message, exported so the board can answer
    // "did the byte I just sent actually land in the message memory?".
    // Simulation cannot answer that question on this design -- it has been
    // right twice while the board was wrong.
    output wire [7:0]  msg0_dbg
);

    // ------------------------------------------------------------------
    //  the power-on message
    //
    //  The old `msg_default` per-index case function has been REMOVED. The
    //  string now lives in the single MSG_RESET constant below: one place,
    //  and the reset value of every message bit is a compile-time constant.
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    //  the power-on message, as ONE 128-BIT CONSTANT
    //
    //  Written most-significant character first, so index 15 is the top byte
    //  and "KEEP CLEAR EXIT " reads back in order:
    //      MSG_RESET[7:0]     = "K"    <- first character on screen
    //      MSG_RESET[127:120] = " "    <- last
    //  A single flat constant is the least ambiguous thing you can hand a
    //  synthesiser as a reset value, which is the whole point of this rewrite.
    // ------------------------------------------------------------------
    //  Written as the FULL TEXT_MAX characters, most significant first, and
    //  sliced down to MSG_LEN when used. Written that way so the constant does
    //  not depend on MSG_LEN having one particular value: indices 0..15 are
    //  the original "KEEP CLEAR EXIT " and anything beyond is padding, so a
    //  testbench that instantiates this with a smaller MSG_LEN still gets the
    //  same power-on message in its low bytes.
    // ------------------------------------------------------------------
    localparam [TEXT_MAX*8-1:0] MSG_RESET_FULL = {
        8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20,  // idx 31..24
        8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20, 8'h20,  // idx 23..16
        8'h20, "T", "I",  "X",  "E",  8'h20, "R",  "A",          // idx 15..8
        "E",   "L", "C",  8'h20, "P", "E",  "E",  "K"            // idx  7..0
    };
    localparam [MSG_LEN*8-1:0] MSG_RESET = MSG_RESET_FULL[MSG_LEN*8-1:0];

    // char_data's reset value. MUST be a compile-time constant: inside an
    // asynchronous-reset branch the value assigned IS the flop's reset value,
    // and a register there makes TD report SYN-5079 ("described with both an
    // asynchronous reset and an asynchronous set") for every bit of it.
    localparam [7:0] MSG_FIRST_CHAR = MSG_RESET[7:0];

    // ==================================================================
    //  message memory -- A PACKED VECTOR, NOT AN ARRAY, ON PURPOSE
    //
    //  The obvious way to write this is
    //      reg [7:0] msg_mem [0:MSG_LEN-1];
    //  with a variable-index write in one always block and a variable-index
    //  read in another. That is a textbook simple dual-port memory and it
    //  simulates perfectly -- and this project has now had that shape of
    //  construct go wrong TWICE on real hardware while the simulation stayed
    //  green. A register array with a variable read and a variable write is
    //  the one construct here that a synthesiser is free to build differently
    //  from the way the simulator does.
    //
    //  A packed vector with an EXPLICIT write decoder and an EXPLICIT read
    //  multiplexer has no memory for a tool to infer and no read/write
    //  interaction to reinterpret. It is MSG_LEN*8 flip-flops, an MSG_LEN-way
    //  decoder and an MSG_LEN-way mux. There is nothing left to get wrong.
    // ==================================================================
    reg [MSG_LEN*8-1:0] msg_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            msg_q <= MSG_RESET;
        end else if (msg_we) begin
            case (msg_waddr)
                5'd0:  msg_q[7:0]     <= msg_wdata;
                5'd1:  msg_q[15:8]    <= msg_wdata;
                5'd2:  msg_q[23:16]   <= msg_wdata;
                5'd3:  msg_q[31:24]   <= msg_wdata;
                5'd4:  msg_q[39:32]   <= msg_wdata;
                5'd5:  msg_q[47:40]   <= msg_wdata;
                5'd6:  msg_q[55:48]   <= msg_wdata;
                5'd7:  msg_q[63:56]   <= msg_wdata;
                5'd8:  msg_q[71:64]   <= msg_wdata;
                5'd9:  msg_q[79:72]   <= msg_wdata;
                5'd10: msg_q[87:80]   <= msg_wdata;
                5'd11: msg_q[95:88]   <= msg_wdata;
                5'd12: msg_q[103:96]  <= msg_wdata;
                5'd13: msg_q[111:104] <= msg_wdata;
                5'd14: msg_q[119:112] <= msg_wdata;
                5'd15: msg_q[127:120] <= msg_wdata;
                5'd16: msg_q[135:128] <= msg_wdata;
                5'd17: msg_q[143:136] <= msg_wdata;
                5'd18: msg_q[151:144] <= msg_wdata;
                5'd19: msg_q[159:152] <= msg_wdata;
                5'd20: msg_q[167:160] <= msg_wdata;
                5'd21: msg_q[175:168] <= msg_wdata;
                5'd22: msg_q[183:176] <= msg_wdata;
                5'd23: msg_q[191:184] <= msg_wdata;
                5'd24: msg_q[199:192] <= msg_wdata;
                5'd25: msg_q[207:200] <= msg_wdata;
                5'd26: msg_q[215:208] <= msg_wdata;
                5'd27: msg_q[223:216] <= msg_wdata;
                5'd28: msg_q[231:224] <= msg_wdata;
                5'd29: msg_q[239:232] <= msg_wdata;
                5'd30: msg_q[247:240] <= msg_wdata;
                5'd31: msg_q[255:248] <= msg_wdata;
                default: ;              // msg_waddr is 5 bits: nothing else
            endcase
        end
    end

    // ------------------------------------------------------------------
    //  the read side, as a pure multiplexer
    //
    //  Kept as a FUNCTION of a register so the fill sequence below is
    //  structurally identical to the original working design, which read a
    //  combinational message function. Only the source changed.
    // ------------------------------------------------------------------
    function [7:0] msg_byte;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  msg_byte = msg_q[7:0];
                5'd1:  msg_byte = msg_q[15:8];
                5'd2:  msg_byte = msg_q[23:16];
                5'd3:  msg_byte = msg_q[31:24];
                5'd4:  msg_byte = msg_q[39:32];
                5'd5:  msg_byte = msg_q[47:40];
                5'd6:  msg_byte = msg_q[55:48];
                5'd7:  msg_byte = msg_q[63:56];
                5'd8:  msg_byte = msg_q[71:64];
                5'd9:  msg_byte = msg_q[79:72];
                5'd10: msg_byte = msg_q[87:80];
                5'd11: msg_byte = msg_q[95:88];
                5'd12: msg_byte = msg_q[103:96];
                5'd13: msg_byte = msg_q[111:104];
                5'd14: msg_byte = msg_q[119:112];
                5'd15: msg_byte = msg_q[127:120];
                5'd16: msg_byte = msg_q[135:128];
                5'd17: msg_byte = msg_q[143:136];
                5'd18: msg_byte = msg_q[151:144];
                5'd19: msg_byte = msg_q[159:152];
                5'd20: msg_byte = msg_q[167:160];
                5'd21: msg_byte = msg_q[175:168];
                5'd22: msg_byte = msg_q[183:176];
                5'd23: msg_byte = msg_q[191:184];
                5'd24: msg_byte = msg_q[199:192];
                5'd25: msg_byte = msg_q[207:200];
                5'd26: msg_byte = msg_q[215:208];
                5'd27: msg_byte = msg_q[223:216];
                5'd28: msg_byte = msg_q[231:224];
                5'd29: msg_byte = msg_q[239:232];
                5'd30: msg_byte = msg_q[247:240];
                5'd31: msg_byte = msg_q[255:248];
                default: msg_byte = 8'h20;
            endcase
        end
    endfunction

    assign msg0_dbg = msg_q[7:0];

    // ==================================================================
    //  fill sequence: walk 0..MSG_LEN-1, holding each write for
    //  FILL_HOLD clocks so text_layer has plenty of setup time
    // ==================================================================
    reg [4:0]  fill_idx;
    reg [9:0]  fill_hold;
    reg        filling;

    assign fill_done = ~filling;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            filling   <= 1'b1;      // reset starts the message load
            fill_idx  <= 5'd0;
            fill_hold <= 10'd0;
            char_we   <= 1'b1;
            char_addr <= 5'd0;
            // A COMPILE-TIME CONSTANT -- NOT msg_mem[5'd0].
            //
            // This looks like a harmless bit of tidying and it is not. Inside
            // an asynchronous-reset branch the value assigned IS the flop's
            // reset value, so it has to be a constant. msg_mem is a register,
            // and worse, it is WRITABLE at run time by the serial port, so its
            // contents are not a known constant at all. TD therefore cannot
            // work out what char_data should come up as and describes every
            // bit of it as needing both an asynchronous reset (from the
            // sensitivity list) and an asynchronous set, reporting
            //
            //   SYN-5079 CRITICAL-WARNING: Register
            //   "u_video_layers/u_scroll_timer/char_data[N]" is described with
            //   both an asynchronous reset and an asynchronous set.
            //
            // msg_default is a pure function of a constant index, so TD folds
            // it to 8'h4B and the ambiguity disappears.
            //
            // Behaviour is identical either way: filling is 1 out of reset, so
            // char_data is reloaded from msg_mem on the very next clock and
            // this value only ever reaches text_layer during the reset cycle
            // itself, where the fill sequence immediately overwrites it.
            char_data <= MSG_FIRST_CHAR;
        end else if (msg_reload) begin
            // ----------------------------------------------------------
            //  a new message has just been written into msg_mem. Re-run the
            //  same fill sequence, so text_layer's character memory is
            //  rewritten in one pass and the screen changes in a single step
            //  instead of character by character.
            //
            //  This branch sits ABOVE the `filling` branch on purpose: a
            //  reload that arrives mid-fill must restart the fill, not be
            //  ignored, otherwise the renderer would end up holding a mix of
            //  the old and new messages.
            //
            //  The scroll position is deliberately NOT reset. The message
            //  length is fixed, so the geometry is identical and the line can
            //  simply carry on from wherever it is.
            // ----------------------------------------------------------
            filling   <= 1'b1;
            fill_idx  <= 5'd0;
            fill_hold <= 10'd0;
            char_we   <= 1'b1;
            char_addr <= 5'd0;
            char_data <= msg_byte(5'd0);
        end else if (filling) begin
            char_we   <= 1'b1;
            char_addr <= fill_idx;
            char_data <= msg_byte(fill_idx);

            if (fill_hold >= FILL_HOLD) begin
                fill_hold <= 10'd0;
                if (fill_idx >= (MSG_LEN[4:0] - 5'd1)) begin
                    filling <= 1'b0;        // whole message written
                    char_we <= 1'b0;
                end else begin
                    fill_idx <= fill_idx + 5'd1;
                end
            end else begin
                fill_hold <= fill_hold + 10'd1;
            end
        end else begin
            char_we <= 1'b0;
        end
    end

    // ==================================================================
    //  travel limits -- the line bounces OFF THE SCREEN EDGES
    //
    //  THIS WAS WRONG BEFORE AND IT MATTERED. The limits used to be
    //  +/- (H_ACTIVE + TEXT_W), i.e. about +/-800. That is the offset at which
    //  the line has left the screen COMPLETELY, so the "bounce" happened far
    //  off screen and the subtitle was invisible for most of every cycle --
    //  roughly 32 s of every 2 minutes at the production scroll rate. On the
    //  board that reads as "the subtitle disappeared", and it is what made a
    //  regression test report a blank row about a working scroller.
    //
    //  The requested motion is a line that starts in the middle, travels until
    //  it MEETS the left edge, reverses, travels until it meets the right edge,
    //  and repeats. So the limits are where the TEXT TOUCHES THE EDGE:
    //
    //      screen x of the text's left edge = x_base + scroll_x, where
    //      x_base = BASE1_X = (H_ACTIVE - TEXT_W) / 2
    //
    //      left  edge at screen 0          -> scroll_x = -BASE1_X
    //      right edge at screen H_ACTIVE-1 -> scroll_x = +BASE1_X
    //
    //  which is symmetric and keeps the line fully on screen at all times.
    //  Nothing is ever clipped, so the renderer's off-screen handling is only
    //  needed as a safety net rather than as normal operation.
    // ==================================================================
    //  TRAVEL LIMITS -- sized from the message that is ACTUALLY there
    //
    //  It used to be simply +/- (H_ACTIVE - TEXT_W)/2 with TEXT_W derived from
    //  the compile-time MSG_LEN, which is correct only while every message is
    //  exactly MSG_LEN characters long. The moment the serial port made the
    //  length variable, that stopped being true:
    //
    //    * a SHORT message would only ever sweep the leading part of the
    //      screen, because the limits were computed for a full-length field;
    //    * the 16-character power-on message -- which used to bounce edge to
    //      edge -- would stop touching the right edge the instant MSG_LEN grew.
    //
    //  So the limits are computed from `used_len` instead, and the position at
    //  which the text is centred moves with it.
    //
    //  text_layer draws the text at   x_left = BASE1_X - scroll_x,  where
    //  BASE1_X is still the fixed FIELD origin (H_ACTIVE - MSG_LEN*CHAR_W)/2.
    //  We want x_left to sweep between 0 (text's left edge at the screen's
    //  left edge) and H_ACTIVE - used_w (text's right edge at the screen's
    //  right edge). Substituting gives a range that is symmetric:
    //
    //      centre = BASE1_X - D/2      limits = centre +/- |D|/2
    //      D      = H_ACTIVE - used_w
    //
    //  For a full-length message D = MSG_LEN*CHAR_W offset and the expression
    //  collapses to the old +/- (H_ACTIVE - TEXT_W)/2 exactly, so nothing about
    //  the case that was already verified changes.
    //
    //  Arithmetic is 13-bit signed: H_ACTIVE (640) plus a used width (up to
    //  640) is 1280, which does not fit in the 12 bits used elsewhere here.
    // ==================================================================
    localparam [11:0] TEXT_W = MSG_LEN * CHAR_W;

    wire signed [12:0] H_ACT_S  = $signed({1'b0, H_ACTIVE[10:0]});
    wire signed [12:0] USED_W   = $signed({1'b0, used_len}) * CHAR_W;
    wire signed [12:0] D        = H_ACT_S - USED_W;      // >= 0 for MSG_LEN<=64
    wire signed [12:0] HALF     =  D >>> 1;
    wire signed [12:0] BASE_P   = (H_ACT_S - $signed({1'b0, TEXT_W})) >>> 1;
    wire signed [12:0] CENTRE   =  BASE_P - HALF;

    wire signed [12:0] RIGHT_LIMIT = CENTRE + HALF;  // text touches right edge
    wire signed [12:0] LEFT_LIMIT  = CENTRE - HALF;  // text touches left edge

    // ==================================================================
    //  the triangle wave
    //
    //  It starts with the line RESTING CENTRED, then bounces forever.
    //
    //      reset            pos = CENTRE     the line is genuinely centred
    //      HOLD_TICKS pass  start travelling
    //      ...              triangle wave between LEFT_LIMIT and RIGHT_LIMIT
    //
    //  *** pos = 0 IS NOT THE CENTRE. *** This block used to say
    //
    //      reset   pos = 0   "the line is centred and readable"
    //
    //  and that was simply false. BASE1_X = 160 is the origin of the FULL
    //  32-character field, so parking at pos = 0 puts the line at screen
    //  x = BASE1_X - 0 = 160. The 16-character power-on message is 160 px
    //  wide, so it occupied 160..320 and sat centred on 240 -- 80 px, i.e.
    //  eight characters, LEFT of the screen centre at 320.
    //
    //  CENTRE (= BASE_P - HALF = 160 - 240 = -80) is the value that centres
    //  the message that is actually loaded. It is why the rest position, and
    //  only the rest position, was wrong: the sweep limits were always built
    //  from CENTRE, so the bounce itself touched both edges correctly and its
    //  midpoint was already -80. Parking at 0 also made the first excursion
    //  asymmetric -- 160 px one way and 320 px the other.
    //
    //  scroll_rst restarts the whole sequence, including the hold, so the
    //  caller can put the line back to a readable state on demand.
    //
    //  NOTE ON THE DIR NAMES: they are inverted with respect to what you see.
    //  DIR_LEFT lowers pos, and since x_left = BASE1_X - pos, lowering pos
    //  moves the text to the RIGHT. The names are kept because renaming them
    //  touches the caller and the tests, but read them as "pos direction",
    //  not as the direction the subtitle travels.
    // ==================================================================
    localparam DIR_LEFT = 1'b1;         // pos is DECREASING -> text moves RIGHT
    localparam DIR_RIGHT= 1'b0;         // pos is INCREASING -> text moves LEFT

    reg               dir;
    reg [23:0]        tick_cnt;
    reg signed [12:0] pos;
    reg [31:0]        hold_cnt;
    reg               at_start;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tick_cnt <= 24'd0;
            hold_cnt <= 32'd0;
            // REST AT CENTRE -- see the long note below. CENTRE is -80 for the
            // 16-character power-on message, NOT 0.
            pos      <= CENTRE;
            dir      <= DIR_LEFT;
            at_start <= 1'b1;
        end else if (scroll_rst) begin
            tick_cnt <= 24'd0;
            hold_cnt <= 32'd0;
            pos      <= CENTRE;
            dir      <= DIR_LEFT;
            at_start <= 1'b1;
        end else if (at_start) begin
            // HOLD THE LINE AT CENTRE, TRACKING THE REAL MESSAGE LENGTH.
            //
            // This assignment runs EVERY clock while holding, and that is
            // deliberate: `used_len` is whatever the fill has committed so
            // far, so CENTRE starts at the empty-message value and settles to
            // the real one as the power-on message lands. Assigning it here
            // (rather than only at reset) means the rest position follows the
            // actual text width instead of being frozen at the reset instant.
            pos <= CENTRE;
            if (hold_cnt >= HOLD_TICKS) begin
                at_start <= 1'b0;
                // start travelling LEFT from the centre, so the first thing the
                // viewer sees is the line moving, not a jump
                pos      <= CENTRE;
                dir      <= DIR_LEFT;
                tick_cnt <= 24'd0;
            end else begin
                hold_cnt <= hold_cnt + 32'd1;
            end
        end else if (scroll_en && fill_done) begin
            if (tick_cnt >= SCROLL_TICKS) begin
                tick_cnt <= 24'd0;
                if (dir == DIR_LEFT) begin
                    if (pos <= LEFT_LIMIT) begin
                        pos <= LEFT_LIMIT + 13'sd1;   // turned round at the wall
                        dir <= DIR_RIGHT;
                    end else begin
                        pos <= pos - 13'sd1;
                    end
                end else begin
                    if (pos >= RIGHT_LIMIT) begin
                        pos <= RIGHT_LIMIT - 13'sd1;  // turned round at the wall
                        dir <= DIR_LEFT;
                    end else begin
                        pos <= pos + 13'sd1;
                    end
                end
            end else begin
                tick_cnt <= tick_cnt + 24'd1;
            end
        end
    end

    // Hold the full signed range: text_layer subtracts this from an unsigned
    // pixel coordinate, so the sign has to survive the port.
    always @(*) scroll_x = pos[10:0];

endmodule
