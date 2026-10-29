// =============================================================================
//  audio_sd_stream.v  --  SD -> 32bit asynchronous FIFO -> I2S player
// -----------------------------------------------------------------------------
//  The TD-generated rfifo_32_32_512 is used because it maps to block RAM.
//  A byte-to-word packer reconstructs the little-endian PCM pairs:
//
//      byte 0 -> word[7:0]     byte 1 -> word[15:8]
//      byte 2 -> word[23:16]   byte 3 -> word[31:24]
//
//  The SD byte stream is only sampled while this module owns the bus.  The
//  first byte after a grant is handled explicitly, so an ack/data same-cycle
//  case cannot shift the whole stream by one byte.
//
//  Data layout in audio_sd.bin:
//      alarm: 131072, 471 sectors (120576 samples, 7.536 s)
//      ring : 131543, 249 sectors ( 63744 samples, 3.984 s)
// =============================================================================
`timescale 1ns / 1ps

module audio_sd_stream #(
    parameter [31:0] ALARM_BASE_SECTOR = 32'd131072,
    parameter [31:0] ALARM_SECTORS     = 32'd471,
    parameter [31:0] RING_BASE_SECTOR  = 32'd131543,
    parameter [31:0] RING_SECTORS      = 32'd249
)(
    input  wire        clk_sd,
    input  wire        rst_sd,
    output reg         sd_areq,
    output reg  [31:0] sd_aaddr,
    input  wire        sd_aack,       // one-cycle grant pulse
    input  wire [7:0]  sd_adata,
    input  wire        sd_advalid,
    input  wire        sd_aend,
    input  wire        emg,

    output reg  [3:0]  state_code,
    output reg  [15:0] sec_done,

    input  wire        clk_a,
    input  wire        rst_a,
    input  wire        rd_req,
    output wire        rd_valid,
    output wire [31:0] rd_data,
    output wire        buf_low
);

    localparam [3:0] S_IDLE    = 4'd0,
                     S_REQ     = 4'd1,
                     S_COLLECT = 4'd2,
                     S_NEXT    = 4'd3;

    // -------------------------------------------------------------------------
    //  2048 x 32bit asynchronous FIFO = 8 KB = 256 ms of 16 kHz mono.
//  Almost-full is deliberately set to 1900, leaving 148 words > the 128
//  words in one SD sector.  A 2045 threshold would overflow/drop the tail of
//  one sector and corrupt the PCM stream after the first fill.
    // -------------------------------------------------------------------------
    wire        fifo_full, fifo_empty, fifo_afull, fifo_aempty;
    wire        fifo_valid;
    wire [31:0] fifo_dout;
    reg  [31:0] fifo_di;
    reg         fifo_we;
    reg         fifo_re;
    reg         fifo_rst;
    wire [10:0] fifo_wrusedw;
    wire [10:0] fifo_rdusedw;

    rfifo_32_32_512 #(
        .DATA_WIDTH_W (32),
        .DATA_WIDTH_R (32),
        .ADDR_WIDTH_W (11),        // 2048 words
        .ADDR_WIDTH_R (11),
        .AL_FULL_NUM  (1900),   // leave at least 128 words (one sector) of headroom
        .AL_EMPTY_NUM (4),
        .SHOW_AHEAD_EN(1'b1),
        .OUTREG_EN    ("NOREG")
    ) u_audio_fifo (
        .rst        (fifo_rst),
        .clkw       (clk_sd),
        .clkr       (clk_a),
        .we         (fifo_we),
        .di         (fifo_di),
        .re         (fifo_re),
        .dout       (fifo_dout),
        .valid      (fifo_valid),
        .full_flag  (fifo_full),
        .empty_flag (fifo_empty),
        .afull      (fifo_afull),
        .aempty     (fifo_aempty),
        .wrusedw    (fifo_wrusedw),
        .rdusedw    (fifo_rdusedw)
    );

    assign buf_low = (fifo_rdusedw <= 11'd64);   // < 8 ms of audio

    // -------------------------------------------------------------------------
    //  Write side: one SD sector at a time; stop at FIFO almost-full.
    // -------------------------------------------------------------------------
    reg [3:0]  st;
    reg [31:0] cur_sec;
    reg [31:0] file_base;
    reg [31:0] file_sect;
    reg        emg_d;
    reg        emg_s1;                       // [FIX v7] 2-stage sync
    reg        emg_s2;
    reg [20:0] emg_dbcnt;                     // [FIX v7] 20 ms integration
    reg        emg_db;                        // [FIX v7] debounced source select
    reg [9:0]  rst_cnt;                       // [FIX v7] 512 clk = 5.12 us (> 81.4 ns read clk)
    reg [1:0]  bcnt;
    reg [23:0] acc;

    wire [31:0] cur_end = file_base + file_sect;
    wire        emg_chg = (emg_db != emg_d);
    // [FIX v7] ADDRESS GUARD.  cur_sec may only ever address this file.
    // Without it a desynchronised base/section pair can walk past the
    // 720-sector audio window and read whatever an older recording left on
    // the card -- the board played a previous recording exactly that way.
    wire [31:0] cur_sec_safe = ((cur_sec >= file_base) && (cur_sec < cur_end))
                               ? cur_sec : file_base;

    always @(posedge clk_sd or posedge rst_sd) begin
        if (rst_sd) begin
            st         <= S_IDLE;
            sd_areq    <= 1'b0;
            sd_aaddr   <= RING_BASE_SECTOR;
            cur_sec    <= RING_BASE_SECTOR;
            file_base  <= RING_BASE_SECTOR;
            file_sect  <= RING_SECTORS;
            emg_d      <= 1'b0;
            emg_s1     <= 1'b0;
            emg_s2     <= 1'b0;
            emg_dbcnt  <= 21'd0;
            emg_db     <= 1'b0;
            state_code <= 4'd0;
            sec_done   <= 16'd0;
            rst_cnt   <= 10'd511;
            fifo_rst   <= 1'b1;
            fifo_we    <= 1'b0;
            fifo_di    <= 32'd0;
            bcnt       <= 2'd0;
            acc        <= 24'd0;
        end else begin
            emg_s1 <= emg;                        // stage 1
            emg_s2 <= emg_s1;                     // stage 2
            if (emg_s2 == emg_db)
                emg_dbcnt <= 21'd0;
            else if (emg_dbcnt >= 21'd2000000) begin
                emg_db    <= emg_s2;
                emg_dbcnt <= 21'd0;
            end else
                emg_dbcnt <= emg_dbcnt + 21'd1;

            emg_d   <= emg_db;
            fifo_we <= 1'b0;

            if (rst_cnt != 10'd0) begin
                rst_cnt  <= rst_cnt - 10'd1;
                fifo_rst <= 1'b1;
            end else
                fifo_rst <= 1'b0;

            if (emg_chg) begin
                file_base <= emg_db ? ALARM_BASE_SECTOR : RING_BASE_SECTOR;
                file_sect <= emg_db ? ALARM_SECTORS     : RING_SECTORS;
                cur_sec   <= emg_db ? ALARM_BASE_SECTOR : RING_BASE_SECTOR;
                sd_aaddr  <= emg_db ? ALARM_BASE_SECTOR : RING_BASE_SECTOR;
                sec_done  <= 16'd0;
                sd_areq   <= 1'b0;
                st        <= S_IDLE;
                rst_cnt   <= 10'd511;
                fifo_rst  <= 1'b1;
                bcnt      <= 2'd0;
                acc       <= 24'd0;
                state_code<= 4'd0;
            end else begin
                case (st)
                    S_IDLE: begin
                        state_code <= 4'd1;
                        if (!fifo_afull && !fifo_full) begin
                            sd_aaddr <= cur_sec_safe;
                            sd_areq  <= 1'b1;
                            st       <= S_REQ;
                        end
                    end

                    S_REQ: begin
                        state_code <= 4'd2;
                        sd_areq    <= 1'b1;
                        if (sd_aack) begin
                            bcnt <= 2'd0;
                            acc  <= 24'd0;
                            st   <= S_COLLECT;
                            // Defensive: some card/controller combinations
                            // return the grant and first byte in the same clock.
                            if (sd_advalid) begin
                                acc[7:0] <= sd_adata;
                                bcnt     <= 2'd1;
                            end
                        end
                    end

                    S_COLLECT: begin
                        state_code <= 4'd3;
                        if (sd_advalid) begin
                            case (bcnt)
                                2'd0: acc[7:0]   <= sd_adata;
                                2'd1: acc[15:8]  <= sd_adata;
                                2'd2: acc[23:16] <= sd_adata;
                                2'd3: begin
                                    fifo_di <= {sd_adata, acc[23:0]};
                                    fifo_we <= 1'b1;
                                end
                            endcase
                            bcnt <= bcnt + 2'd1;
                        end

                        if (sd_aend) begin
                            sd_areq  <= 1'b0;
                            sec_done <= sec_done + 16'd1;
                            st       <= S_NEXT;
                        end
                    end

                    S_NEXT: begin
                        state_code <= 4'd4;
                        if (cur_sec + 32'd1 >= cur_end)
                            cur_sec <= file_base;
                        else
                            cur_sec <= cur_sec + 32'd1;
                        st <= S_IDLE;
                    end

                    default: st <= S_IDLE;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    //  Read side: one outstanding request, one-cycle valid response.
    // -------------------------------------------------------------------------
    always @(posedge clk_a or posedge rst_a) begin
        if (rst_a)
            fifo_re <= 1'b0;
        else begin
            fifo_re <= 1'b0;
            if (rd_req && !fifo_empty)
                fifo_re <= 1'b1;
        end
    end

    // SHOW_AHEAD mode: valid is a level and dout is the current word.
    // The player captures this word on the cycle it asserts rd_req.
    assign rd_valid = fifo_valid;
    assign rd_data  = fifo_dout;

endmodule
