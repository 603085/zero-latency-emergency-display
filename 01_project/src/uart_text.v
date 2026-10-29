// ============================================================================
//  uart_text.v -- type the subtitle over a serial port  (Task A)
//
//  What the operator sees
//  ----------------------
//      open a serial terminal at 115200 8N1, type a line, press Enter
//      -> the subtitle on the HDMI screen changes to that line
//
//      ...or just send the text and wait a moment. If no newline arrives the
//      line is committed automatically after IDLE_TICKS of silence, so a tool
//      that sends a bare string ("FIRE EXIT") still works.
//
//  Protocol
//  --------
//      printable ASCII 0x20..0x7E   append to the line being typed
//      CR (0x0D) or LF (0x0A)       commit now
//      BS (0x08) or DEL (0x7F)      delete the last character
//      silence for ~0.8 s           commit (safety net, see above)
//      buffer full (MSG_LEN chars)  commit
//      anything else                ignored
//
//  An EMPTY line is never committed. Pressing Enter on a blank line does
//  nothing, so a stray newline cannot wipe the sign. To blank the text, send
//  a single space.
//
//  ---------------------------------------------------------------------------
//  WHY THERE IS NO STAGING BUFFER ANY MORE
//  ---------------------------------------------------------------------------
//  The first version of this module kept the line being typed in a separate
//  array and copied it into the message memory on Enter:
//
//      reg [7:0] line [0:MSG_LEN-1];
//      ...
//      line[wr] <= b;                          // written here
//      msg_wdata <= (k < wr) ? line[k] : 8'h20; // ...and READ here
//
//  Both the write and the read were in THE SAME always block. That is legal
//  Verilog and it simulates perfectly, but it is not a construct a synthesiser
//  is obliged to build the way you meant: a variable-index read of a register
//  array that is also written with a variable index in the same block. TD
//  built it such that the read came back blank, so every commit wrote spaces,
//  text_layer rendered font_rom(0), and the subtitle DISAPPEARED.
//
//  On the board that looked like "the serial port receives but nothing is
//  displayed, and after enough characters the subtitle just vanishes" -- with
//  the 7-segment digit cheerfully counting received characters the whole time,
//  because the receiver and the counter were fine and only the copy was not.
//
//  The staging buffer is now gone. Received characters go STRAIGHT into the
//  message memory, one write per character. There is nothing left to copy and
//  therefore nothing left to get wrong.
//
//  WHY WRITING DIRECTLY IS STILL SAFE
//  ----------------------------------
//  The message memory is not what is on screen. text_layer's character memory
//  is, and that is only rewritten by scroll_timer's fill sequence, which only
//  runs on msg_reload -- i.e. after a commit. So a half-typed line sits in the
//  message memory invisibly, and the screen still only ever shows a COMPLETE
//  committed line. Removing the staging buffer costs nothing.
//
//  The part-written line is padded with spaces out to MSG_LEN at commit time,
//  which is what makes a short new message actually replace a long old one.
// ============================================================================

`timescale 1ns / 1ps

module uart_text #(
    parameter MSG_LEN  = 16,            // characters committed to the screen
    parameter CLK_FREQ = 25_000_000,
    parameter BAUD     = 115200,
    // Commit after this many clocks with no byte arriving. 25 MHz x 0.8 s.
    // Exposed so a testbench can shorten it.
    parameter [31:0] IDLE_TICKS = 32'd20_000_000
)(
    input  wire       clk,              // video_clk, 25 MHz
    input  wire       rst,              // active high

    input  wire       rx,               // serial input pin, idle HIGH

    // ---------------- write port into scroll_timer's message memory ----
    output reg        msg_we,
    output reg  [4:0] msg_waddr,
    output reg  [7:0] msg_wdata,
    output reg        msg_reload,       // 1-clock pulse: re-render the message

    // Length of the message just committed, so scroll_timer can size the
    // bounce to the text that is actually there. 0 = nothing committed yet.
    output reg  [5:0] msg_len,

    // ---------------- debug / status ----------------
    output wire [7:0] rx_last,          // last byte received
    output wire [5:0] rx_len,           // characters typed on the current line
    output wire       rx_idle,          // synchronised serial line (idle high)
    output reg        rx_ok             // 1-clock pulse per committed message
);

    // ==================================================================
    //  the receiver
    // ==================================================================
    wire [7:0] b;
    wire       bv;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_rx (
        .clk       (clk),
        .rst       (rst),
        .rx        (rx),
        .data      (b),
        .valid     (bv),
        .line_idle (rx_idle)
    );

    // ==================================================================
    //  bookkeeping
    // ==================================================================
    //  Six bits, not five. MSG_LEN is 32, so `wr` has to be able to hold the
    //  value 32 itself -- the counter reaches MSG_LEN before the buffer is
    //  declared full. A 5-bit counter silently wraps 31 -> 0 there and the
    //  buffer would never fill.
    localparam [5:0] LEN  = MSG_LEN;
    localparam [5:0] LAST = MSG_LEN - 1;

    reg [5:0]  wr;                          // characters typed so far
    reg [7:0]  last_b;
    reg       prev_cr;                      // previous byte was a CR
    reg [31:0] idle_cnt;

    assign rx_last = last_b;
    assign rx_len  = wr;

    // one below wr, pre-computed so backspace never has to bit-slice an
    // expression
    wire [5:0] wr_m1 = wr - 6'd1;

    // ==================================================================
    //  CHARACTER NORMALISATION AND FILTERING
    //
    //  font_rom contains glyphs for exactly these characters:
    //
    //      0-9   A-Z   space   :   -   .   /   >
    //
    //  and its default branch emits 8'h00, which draws NOTHING AT ALL.
    //  Two consequences, both of which the operator hits immediately:
    //
    //  (1) LOWERCASE a-z HAS NO GLYPH.
    //      Type "fire exit" into a terminal and every letter becomes an
    //      invisible blank, while "123" renders perfectly. On the bench that
    //      reads as "digits work, letters don't" and looks like a data-path
    //      fault -- it was reported exactly that way. It is not a fault in
    //      the receiving or the storing: the glyphs simply do not exist.
    //      Lowercase is therefore FOLDED TO UPPERCASE, which is also the
    //      more legible choice on an emergency sign.
    //
    //  (2) Any other punctuation ('!', ',', '?', ...) would occupy a cell and
    //      draw nothing, leaving an invisible hole of unknown width. Those
    //      are DROPPED rather than stored, so the message stays compact and
    //      the character count means what it says.
    //
    //  Filtering here, at the point text ENTERS the design, keeps text_layer
    //  untouched -- it is rendering code that is already verified, and the
    //  font's limits are a property of the input, not of the renderer.
    // ==================================================================
    wire [7:0] ch_up = ((b >= 8'h61) && (b <= 8'h7A))
                       ? (b - 8'h20)                       // a-z -> A-Z
                       : b;

    wire ch_ok = ((ch_up >= 8'h30) && (ch_up <= 8'h39)) || // 0-9
                 ((ch_up >= 8'h41) && (ch_up <= 8'h5A)) || // A-Z
                 (ch_up == 8'h20) ||                       // space
                 (ch_up == 8'h3A) || (ch_up == 8'h2D) ||   // : -
                 (ch_up == 8'h2E) || (ch_up == 8'h2F) ||   // . /
                 (ch_up == 8'h3E);                         // >

    // ==================================================================
    //  control
    // ==================================================================
    localparam [1:0] S_RECV  = 2'd0,    // waiting for a byte
                     S_PAD   = 2'd1,    // space-filling the tail of the line
                     S_PULSE = 2'd2,    // one clock: raise msg_reload
                     S_DONE  = 2'd3;    // one clock: report the commit

    reg [1:0] st;
    reg [5:0] pad_k;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr         <= 6'd0;
            last_b     <= 8'h00;
            prev_cr    <= 1'b0;
            idle_cnt   <= 32'd0;
            st         <= S_RECV;
            pad_k      <= 6'd0;

            msg_we     <= 1'b0;
            msg_waddr  <= 5'd0;
            msg_wdata  <= 8'h20;
            msg_reload <= 1'b0;
            // 0 means "nothing committed yet"; video_layers_top turns that
            // into the length of the power-on message.
            msg_len    <= 6'd0;
            rx_ok      <= 1'b0;

        end else begin
            // all three are one-clock pulses
            msg_we     <= 1'b0;
            msg_reload <= 1'b0;
            rx_ok      <= 1'b0;

            case (st)

            // ------------------------------------------------------
            //  receive
            // ------------------------------------------------------
            S_RECV: begin
                if (bv) begin
                    last_b   <= b;
                    prev_cr  <= (b == 8'h0D);
                    idle_cnt <= 32'd0;

                    if ((b == 8'h0D) || (b == 8'h0A)) begin
                        // Enter. A CR immediately followed by an LF is one
                        // commit, because that is what terminals send.
                        // An empty line is NOT committed -- see the header.
                        if (!((b == 8'h0A) && prev_cr) && (wr != 6'd0)) begin
                            st    <= S_PAD;
                            pad_k <= wr;
                        end

                    end else if ((b == 8'h08) || (b == 8'h7F)) begin
                        // backspace: erase the last character in place
                        if (wr != 6'd0) begin
                            wr        <= wr_m1;
                            msg_we    <= 1'b1;
                            msg_waddr <= wr_m1[4:0];
                            msg_wdata <= 8'h20;
                        end

                    end else if (ch_ok) begin
                        // A WRITE PER CHARACTER, straight into the message
                        // memory, already folded to a character the font can
                        // actually draw. This is the whole point of the
                        // rewrite -- there is nothing left to copy.
                        if (wr < LEN) begin
                            msg_we    <= 1'b1;
                            msg_waddr <= wr[4:0];
                            msg_wdata <= ch_up;
                            wr        <= wr + 6'd1;
                        end else begin
                            // full: commit what we have. The character that
                            // arrived is dropped -- MSG_LEN is the limit.
                            st    <= S_PAD;
                            pad_k <= wr;
                        end
                    end
                    // Any other byte is ignored -- see the character
                    // normalisation note above; those glyphs do not exist.

                end else if (idle_cnt < IDLE_TICKS) begin
                    idle_cnt <= idle_cnt + 32'd1;

                end else if (wr != 6'd0) begin
                    // silence, and there is something to show
                    st    <= S_PAD;
                    pad_k <= wr;
                end
            end

            // ------------------------------------------------------
            //  space-fill from the end of the typed text to MSG_LEN
            //
            //  THIS IS WHAT MAKES A SHORTER MESSAGE REPLACE A LONGER ONE.
            //  Without it the tail of the previous message would still be
            //  sitting in the message memory and would reappear as soon as
            //  the scroll moved far enough to reach it.
            // ------------------------------------------------------
            S_PAD: begin
                if (pad_k >= LEN) begin
                    st <= S_PULSE;              // nothing to pad
                end else begin
                    msg_we    <= 1'b1;
                    msg_waddr <= pad_k[4:0];
                    msg_wdata <= 8'h20;
                    if (pad_k >= LAST) st <= S_PULSE;
                    else               pad_k <= pad_k + 6'd1;
                end
            end

            // ------------------------------------------------------
            //  the last write is in flight during this clock; drop msg_we and
            //  raise the reload, so the renderer starts only after every byte
            //  has landed
            // ------------------------------------------------------
            S_PULSE: begin
                msg_reload <= 1'b1;
                // wr still holds the committed length here
                msg_len    <= wr;
                wr         <= 6'd0;             // next line starts empty
                st         <= S_DONE;
            end

            // ------------------------------------------------------
            //  report and go back to receiving
            // ------------------------------------------------------
            S_DONE: begin
                rx_ok <= 1'b1;
                st    <= S_RECV;
            end

            default: st <= S_RECV;

            endcase
        end
    end

endmodule
