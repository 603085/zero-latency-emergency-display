// =============================================================================
//  audio_hdmi_i2s.v  --  standard 48 kHz I2S transmitter fed by the SD FIFO
// -----------------------------------------------------------------------------
//  The official baseline generator is used as the timing reference:
//      BCLK  = MCLK / 2
//      64 BCLK clocks per channel, 128 per stereo frame => 48 kHz at 12.288 MHz
//      24-bit signed PCM in the top bits of the 64-bit channel word
//
//  Source data is 16 kHz mono.  One output sample is held for three 48 kHz
//  frames, so the playback speed is exactly right (48 kHz / 3 = 16 kHz).
//  A 32-bit FIFO word contains two little-endian PCM samples:
//      rd_data[15:0]  = first sample
//      rd_data[31:16] = second sample
// =============================================================================
`timescale 1ns / 1ps

module audio_hdmi_i2s (
    input  wire        I_mclk,
    input  wire        I_rst,

    // FIFO read side: rd_valid is a level in show-ahead mode; assert rd_req
    // for one clock to pop the current word.
    output reg         rd_req,
    input  wire        rd_valid,
    input  wire [31:0] rd_data,

    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT,
    output wire        data_ok,

    // =====================================================================
    //  [C 2026-10-23] 并行样点旁路输出
    // ---------------------------------------------------------------------
    //  为什么要有这个：
    //    原来 24bit 样点只能靠 I2S 串行发出去，再由 I2S_receiver 在
    //    video_clk(25MHz) 域把 BCLK(6.144MHz) 采回来还原。
    //    25/6.144 = 4.07 个时钟/位，而接收端用的是 3 级同步器 ——
    //    余量只有 1 个时钟，会周期性"滑码"：某一位被多采/少采一次，
    //    整个 24bit 字就整体移位 N 位。
    //    后果就是板上看到的：声音内容对、但小得离谱（移位 = 除以 2^N），
    //    频谱往高频堆（滑码是冲激 = 宽带噪声），只有增益最大的最高段
    //    偶尔跳一下。
    //    现在直接把 24bit 样点用"数据稳定 + 翻转标志"跨到 video_clk，
    //    I2S 串行那一路只留作波形观察，不再参与数据通路。
    // =====================================================================
    output reg  [23:0] O_pcm_data,     // 当前 48 kHz 帧的 24bit 样点（单声道，左右相同）
    output reg         O_pcm_toggle    // 每个 48 kHz 帧翻转一次，给跨时钟域用
);

    reg signed [15:0] cur_lo;
    reg signed [15:0] cur_hi;
    reg               word_valid;
    // Loudness compensation for the quiet SD voice/ring material.
    // Below -12 dBFS: 4x; -12 to -6 dBFS: 2x; louder peaks: 1.5x.
    // Every branch is saturation-limited to signed 16-bit.  The graduated
    // curve reaches test-tone-like loudness without applying a fixed 4x to
    // already near-full-scale peaks.
    function signed [15:0] boost_sat;
        input signed [15:0] x;
        reg signed [16:0] ax;
        reg signed [17:0] y;
        begin
            if (x >= 0) ax = {1'b0, x};
            else        ax = -$signed({1'b0, x});
            if (ax < 17'sd8192)
                y = x * 18'sd4;       // loud boost for quiet voice/ring
            else if (ax < 17'sd16384)
                y = x * 18'sd2;       // medium-level boost
            else
                y = x + (x >>> 1);   // keep peaks nearly intact
            if (y > 18'sd32767)       boost_sat = 16'sh7FFF;
            else if (y < -18'sd32768) boost_sat = 16'sh8000;
            else                      boost_sat = y[15:0];
        end
    endfunction

    reg               sample_phase;    // 0 = low half-word, 1 = high half-word
    reg [1:0]         frame_div;       // 0,1,2; one source sample per 3 frames
    reg signed [15:0] out_sample;

    reg [5:0]         bit_cnt;
    reg [63:0]        shift_reg;

    wire signed [15:0] sample_next =
        (frame_div == 2'd2)
            ? (sample_phase ? (word_valid ? cur_hi : 16'sd0)
                            : (word_valid ? cur_lo : 16'sd0))
            : out_sample;
    wire signed [23:0] next24 = {{8{sample_next[15]}}, sample_next};
    wire signed [23:0] out24  = {{8{out_sample[15]}},   out_sample};

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            rd_req       <= 1'b0;
            cur_lo       <= 16'sd0;
            cur_hi       <= 16'sd0;
            word_valid   <= 1'b0;
            sample_phase <= 1'b0;
            frame_div    <= 2'd0;
            out_sample   <= 16'sd0;
            bit_cnt      <= 6'd0;
            shift_reg    <= 64'd0;
            O_i2s_BCLK   <= 1'b0;
            O_i2s_LRCK   <= 1'b0;
            O_i2s_DOUT   <= 1'b0;
            O_pcm_data   <= 24'd0;
            O_pcm_toggle <= 1'b0;
        end else begin
            O_i2s_BCLK <= ~O_i2s_BCLK;
            rd_req     <= 1'b0;

            // -----------------------------------------------------------------
            //  SHOW_AHEAD FIFO: rd_valid is a level, rd_data is the current
            //  word.  Capture it once and pop it with rd_req.  No request/valid
            //  overlap is possible, so a word cannot be lost at a sample tick.
            // -----------------------------------------------------------------
            if (rd_valid && !word_valid) begin
                cur_lo     <= boost_sat($signed(rd_data[15:0]));
                cur_hi     <= boost_sat($signed(rd_data[31:16]));
                word_valid <= 1'b1;
                rd_req     <= 1'b1;       // pop the word we just captured
            end

            // -----------------------------------------------------------------
            //  I2S shift engine, identical in timing to the official example.
            // -----------------------------------------------------------------
            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT <= shift_reg[63];
                shift_reg  <= {shift_reg[62:0], 1'b0};

                if (bit_cnt == 6'd63) begin
                    bit_cnt <= 6'd0;

                    if (O_i2s_LRCK == 1'b0) begin
                        // left channel finished -> send the same sample on right
                        O_i2s_LRCK <= 1'b1;
                        shift_reg  <= {out24, 40'd0};
                    end else begin
                        // right channel finished -> choose next source sample
                        O_i2s_LRCK <= 1'b0;
                        shift_reg  <= {next24, 40'd0};

                        // ★ 并行样点旁路：与这一帧 I2S 上发的 24bit 完全一致
                        //   （next24 = {{8{s[15]}}, s[15:0]}，左对齐满量程）
                        O_pcm_data   <= next24;
                        O_pcm_toggle <= ~O_pcm_toggle;

                        if (frame_div == 2'd2) begin
                            frame_div  <= 2'd0;
                            out_sample <= sample_next;
                            if (sample_phase) begin
                                sample_phase <= 1'b0;
                                if (word_valid)
                                    word_valid <= 1'b0; // need the next FIFO word
                            end else
                                sample_phase <= 1'b1;
                        end else begin
                            frame_div <= frame_div + 2'd1;
                        end
                    end
                end else
                    bit_cnt <= bit_cnt + 6'd1;
            end
        end
    end

    assign data_ok = word_valid;

endmodule
