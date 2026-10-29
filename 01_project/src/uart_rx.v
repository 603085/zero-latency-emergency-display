// ============================================================================
//  uart_rx.v -- 115200 8N1 serial receiver  (Task A / serial text entry)
//
//  Receives one byte at a time from a PC serial terminal and hands it to the
//  text layer. There is no processor anywhere in this design, so this is a
//  plain oversampling receiver: it finds the falling edge of the start bit,
//  waits half a bit period to land in the MIDDLE of the bit, and then samples
//  once per bit period.
//
//  WHY THE MIDDLE AND NOT THE EDGE. Sampling on the edge puts the sample
//  exactly where the signal is changing, i.e. where jitter, cable capacitance
//  and the sender's own baud error all pile up. The middle of the bit is the
//  point furthest from both edges, which is why every UART does it. Sampling
//  on the edge "mostly works" and then fails on one board and not another.
//
//  WHY 3 SYNCHRONISER STAGES. rx is an asynchronous pin. It is sampled by a
//  25 MHz clock that has nothing to do with the sender's bit clock, so it can
//  change state arbitrarily close to a clock edge. Feeding that straight into
//  the receiver's state machine lets a single metastable sample fan out to
//  several flip-flops and produce a different value in each -- which shows up
//  as a byte that is wrong in the middle of the bit pattern. Three stages
//  make the probability of that surviving to the state machine negligible.
//  The 2 extra clocks of delay are irrelevant: they are 80 ns out of an 8.7 us
//  bit time.
//
//  CLOCK DOMAIN. This runs on video_clk (25 MHz) on purpose. The character
//  memory it eventually feeds is read by text_layer in that same domain, so
//  keeping the whole receive path in one domain means there is no clock
//  crossing to get wrong anywhere in the text system.
//
//  Baud rate: 25_000_000 / 217 = 115207.4, an error of +0.006% against
//  115200. A UART tolerates about +/-2% per end, so this is not close to the
//  limit. The divisor is a parameter, so changing CLK_FREQ is enough.
// ============================================================================

`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_FREQ = 25_000_000,    // video_clk, Hz
    parameter BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst,              // synchronous to clk, active high

    input  wire       rx,               // asynchronous serial input, idle HIGH

    output reg  [7:0] data,             // last byte received
    output reg        valid,            // 1-clock pulse when data is fresh
    output wire       line_idle         // synchronised copy of rx (for debug)
);

    // --------------------------------------------------------------
    //  bit timing
    // --------------------------------------------------------------
    localparam DIV  = CLK_FREQ / BAUD;  // 217 clocks per bit
    localparam HALF = DIV / 2;          // 108 clocks to the middle of the bit

    // --------------------------------------------------------------
    //  3-stage synchroniser
    //
    //  Reset value 3'b111 because the line is idle HIGH: starting the
    //  synchroniser at 0 would look like a start bit for one bit time on every
    //  reset and inject a spurious 0x00.
    // --------------------------------------------------------------
    reg [2:0] sync;

    always @(posedge clk or posedge rst) begin
        if (rst) sync <= 3'b111;
        else     sync <= {sync[1:0], rx};
    end

    wire rx_s = sync[2];

    assign line_idle = rx_s;

    // --------------------------------------------------------------
    //  receiver state machine
    // --------------------------------------------------------------
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [2:0]  bitn;
    reg [7:0]  sh;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st    <= S_IDLE;
            cnt   <= 16'd0;
            bitn  <= 3'd0;
            sh    <= 8'h00;
            data  <= 8'h00;
            valid <= 1'b0;
        end else begin
            // valid is a one-clock pulse; it is cleared every cycle and only
            // set again at the end of a good frame
            valid <= 1'b0;

            case (st)

                // ----------------------------------------------
                //  wait for the start bit's falling edge
                // ----------------------------------------------
                S_IDLE: begin
                    if (!rx_s) begin
                        // the line just went low: this is the leading edge of
                        // the start bit. Do NOT sample yet -- jump half a bit
                        // forward to reach the middle of it.
                        cnt <= HALF[15:0];
                        st  <= S_START;
                    end
                end

                // ----------------------------------------------
                //  halfway into the start bit: verify it is still low
                //
                //  A momentary glitch on the line (or a noise spike) looks
                //  exactly like a start edge. Checking again at the middle
                //  rejects it, which is why this state exists at all.
                // ----------------------------------------------
                S_START: begin
                    if (cnt == 16'd1) begin
                        if (rx_s) begin
                            st <= S_IDLE;           // false start, drop it
                        end else begin
                            cnt  <= DIV[15:0];
                            bitn <= 3'd0;
                            st   <= S_DATA;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                // ----------------------------------------------
                //  the 8 data bits, LSB FIRST
                // ----------------------------------------------
                S_DATA: begin
                    if (cnt == 16'd1) begin
                        // shift right: the first bit received is bit 0
                        sh <= {rx_s, sh[7:1]};

                        if (bitn == 3'd7) begin
                            st  <= S_STOP;
                            cnt <= DIV[15:0];
                        end else begin
                            bitn <= bitn + 3'd1;
                            cnt  <= DIV[15:0];
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                // ----------------------------------------------
                //  middle of the stop bit: commit only if it is HIGH
                //
                //  A stop bit that reads low means the frame was malformed --
                //  usually a baud mismatch or a glitch. Publishing it anyway
                //  would scatter random characters into the message, so it is
                //  dropped instead. With the divisor above the stop bit is
                //  sampled 217 clocks after the last data bit, comfortably
                //  inside the real stop bit.
                // ----------------------------------------------
                S_STOP: begin
                    if (cnt == 16'd1) begin
                        if (rx_s) begin
                            data  <= sh;
                            valid <= 1'b1;
                        end
                        st <= S_IDLE;           // ready for the next start bit
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                default: st <= S_IDLE;

            endcase
        end
    end

endmodule
