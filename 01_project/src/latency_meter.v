// ============================================================================
//  latency_meter.v -- on-chip latency measurement  (Task A)
//
//  WHAT IT MEASURES
//  ----------------
//  The number of video_clk cycles between the emergency input becoming stable
//  and the layer descriptor registers updating. That is the deterministic part
//  of the response: everything after it is one scan line of pixel pipeline.
//
//  WHY IT EXISTS
//  -------------
//  The original plan used an oscilloscope on two pins. Not every team has one,
//  and the measurement is more convincing when the number comes from the
//  device itself:
//
//      1 clk @ 25 MHz = 40 ns
//      reported_cycles x 40 ns = measured delay
//
//  The counter is frozen afterwards, so the value survives until the next
//  trigger. It can be shown on the 7-segment display, shifted out over UART,
//  or simply held while a photo is taken.
//
//  HONESTY NOTE FOR THE DEFENCE
//  ----------------------------
//  This number does NOT include the remaining pixel pipeline (up to one scan
//  line, about 20 us, plus one frame of scan-out). A bench scope would show
//  the same order of magnitude. State the breakdown rather than claiming the
//  counter covers the whole path.
//
//  INDICATORS
//  ----------
//    latency_measurement  : level, high while the counter is running
//    latency_one_frame    : latched, high if the measured delay exceeded one
//                           frame (16.7 ms). It should never be high.
// ============================================================================

`timescale 1ns / 1ps

module latency_meter #(
    // 25 MHz / 16.7 ms = 417500 clocks in one 640x480@60 frame.
    // The compare is a threshold only, so a round number is fine.
    parameter [31:0] ONE_FRAME_CYCLES = 32'd417_500
)(
    input  wire        clk,
    input  wire        rst,

    // start on a rising edge, stop on a rising edge
    input  wire        start_evt,
    input  wire        stop_evt,

    output reg  [31:0] latency_cycles,       // frozen result, 0 = no measurement yet
    output reg         latency_valid,        // 1 once a measurement has been taken
    output reg         latency_measurement,  // high while counting
    output reg         latency_one_frame     // 1 if the result exceeded one frame
);

    reg  [31:0] cnt;
    reg         running;
    reg         start_d, stop_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            start_d             <= 1'b0;
            stop_d              <= 1'b0;
            cnt                 <= 32'd0;
            running             <= 1'b0;
            latency_cycles      <= 32'd0;
            latency_valid       <= 1'b0;
            latency_measurement <= 1'b0;
            latency_one_frame   <= 1'b0;
        end else begin
            start_d <= start_evt;
            stop_d  <= stop_evt;

            // rising-edge detect on both events
            if (start_evt && !start_d) begin
                // a new measurement always restarts cleanly, even if the
                // previous one never saw its stop event
                cnt                 <= 32'd0;
                running             <= 1'b1;
                latency_measurement <= 1'b1;
                latency_one_frame   <= 1'b0;
            end else if (running) begin
                cnt <= cnt + 32'd1;

                if (stop_evt && !stop_d) begin
                    running             <= 1'b0;
                    latency_measurement <= 1'b0;
                    latency_cycles      <= cnt;
                    latency_valid       <= 1'b1;
                    latency_one_frame   <= (cnt > ONE_FRAME_CYCLES);
                end

                // safety net: if the stop event never comes, do not count
                // forever. 2 frames is already far beyond spec, so the result
                // is capped and still reported.
                if (cnt >= (ONE_FRAME_CYCLES << 1)) begin
                    running             <= 1'b0;
                    latency_measurement <= 1'b0;
                    latency_cycles      <= cnt;
                    latency_valid       <= 1'b1;
                    latency_one_frame   <= 1'b1;
                end
            end
        end
    end

endmodule
