module bmp_read #(
    // 支持的源图最大宽度 = scaler 行缓冲深度（超过就当"头不匹配"跳过）
    parameter [11:0] MAX_SRC_W = 12'd2048,
    // 缩放器是否在位（由 sd_card_bmp 的 SCALER_ENABLE 传入，两边必须一致）：
    //   1'b1 → 接受任意尺寸，交给 scaler_bilinear 缩放到 640×480；
    //   1'b0 → 只放行 640×480。直通时非 640×480 的图必须跳过，否则像素数不足、
    //          frame_write 写不满一帧，而 load_busy 没有超时 → 永久卡死。
    parameter        SCALER_ACTIVE = 1'b1
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [3:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [3:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    // ⚠️ bmp_width / bmp_height 自"支持任意分辨率"起【不再参与头比对】
    //    （老逻辑硬要求 宽==640 && 高==480，见下面 header_match）。
    //    端口保留只为少动上层；源宽上限改由参数 MAX_SRC_W 控制。
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    // scaler 说"可以继续送源像素" —— 唯一的反压手段，在扇区边界生效
    input                       src_ready,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,
    // 解析出来的原始尺寸 + 图像首像素（给 scaler）
    output wire [11:0]          bmp_w_o,
    output wire [11:0]          bmp_h_o,
    output wire                 bmp_sof_o
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;

//  ---------- 冷却计数器（2026-10-20 加）----------
//  达标之后 scan_done 不立刻置起，先空走 COOL_STEPS 步：
//  让 SD 卡从「连续读」平滑过渡到「空闲」，装载首图前有个喘息。
//  64 步 x 8 扇区 = 512 扇区 ≈ 84 ms @25MHz SPI，相对少扫的 8600 扇区可忽略。
localparam [15:0] COOL_STEPS = 16'd64;
reg [15:0] cool_cnt;
reg        cool_pending;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

// --------------------------------------------------------------------------
//  行/像素计数：支持任意宽高 + 跳过 BMP 每行末尾的 4 字节对齐补位
//  BMP 一行有 w*3 个像素字节，后面补 0 到 4 字节的整数倍（row_stride）。
//  老代码把像素区当成"连续 3 字节一个像素"，只要 w*3 不是 4 的倍数，
//  每行就会把补位字节当像素吃掉 → 相位每行漂 (row_stride - w*3) 字节，
//  三行就漂掉一个像素，整幅画面剪切错位。426x320（w*3=1278, 补 2）必踩。
// --------------------------------------------------------------------------
reg  [13:0] byte_in_row;    // 当前行内第几个字节（含补位）
reg  [11:0] px_in_row;      // 当前行内第几个像素
reg  [23:0] px_cnt;         // 已发出的源像素总数
reg         load_next_pending;  // 扇区读完、等 src_ready 再推进地址

wire [11:0] src_w12 = width[11:0];
// [FIX 2026-10-25b] Pipeline the BMP geometry at the header/data boundary.
// The old version fed width -> MULT18 -> add/compare directly into
// byte_in_row, producing a -2.07 ns path at 100 MHz.  That path can corrupt
// the row counter while loading a new/large BMP, leaving the picture source
// stuck or causing an emergency/normal reload race.
wire [13:0] row_bytes_raw  = {2'd0, src_w12} * 14'd3;
wire [13:0] row_stride_raw = (row_bytes_raw + 14'd3) & ~14'd3;
wire [23:0] px_total_raw   = src_w12 * height[11:0];

reg [11:0] src_w_r;
reg [11:0] src_h_r;
reg [13:0] row_bytes_r;
reg [13:0] row_stride_r;
reg [23:0] px_total_r;

// Only"像素区之内、且在当前行的 w*3 之内"的字节才是像素字节
wire        bmp_byte_is_pix = (bmp_len_cnt >= pixel_offset) && (byte_in_row < row_bytes_r);
wire        px_done    = (px_total_r != 24'd0) && (px_cnt >= px_total_r);

// 首像素标志：进入数据阶段置起，第一个 bmp_data_wr_en 那一拍清掉。
// （不能用 px_cnt==0 判断：px_cnt 和 bmp_data_wr_en 不是同一拍生效，
//   实测这样写 SOF 永远出不来 —— scaler 就永远不会开始。）
reg         first_px;
assign bmp_w_o  = src_w_r;
assign bmp_h_o  = src_h_r;
assign bmp_sof_o= bmp_data_wr_en && first_px;

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

assign ready = (state == ST_IDLE);
// --------------------------------------------------------------------------
//  头比对
//
//  SCALER_ACTIVE（模块参数，由 sd_card_bmp 的 SCALER_ENABLE 传入，两边必须一致）：
//    · 1'b1 → 缩放器在位：接受任意尺寸，交给 scaler_bilinear 缩放到 640×480；
//    · 1'b0 → 缩放器直通：**只放行 640×480**。必须这样 —— 非 640×480 的图
//      像素数不足，frame_write 写不满一帧，而 load_busy 没有超时 → 永久卡死。
//      （320×240 / 426×320 / 800×600 那三张测试图就是这种情况。）
//
//  2026-10-13：缩放器曾因组合路径过长（13.389 ns > 10 ns）被临时旁路，
//  现已做流水改造后重新使能，所以默认值是 1'b1。
// --------------------------------------------------------------------------
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width       != 32'd0) &&
                      (height      != 32'd0) &&
                      (height[31]  == 1'b0) &&
                      (height[15:12] == 4'd0) &&          // 高度上限 4095（px_total 用 24 位）
                      (width[11:0] <= MAX_SRC_W) &&
                      (bit_count   == 16'd24) &&
                      (compression == 32'd0) &&
                      (SCALER_ACTIVE ||                      // 直通时：只收 640×480
                       ((width  == 32'd640) && (height == 32'd480)));
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len) &&
                        bmp_byte_is_pix;
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
// ==========================================================================
//  命中【不再跳整张图】，和未命中一样每 8 个扇区走一格。
//
//  上一版的错：命中后跳 scan_sector + file_sector_count 再向上对齐到 8 的倍数。
//      file_sector_count = (921654 + 511) >> 9 = 1801
//  但文件实际占 225 个簇 = 1800 个扇区 —— 这个跳转本身就多跳了，
//  向上对齐又可能再多跳 7 个。
//  结果：只要下一张图的起始扇区落在跳转位置【后面一点】，就被永久跳过。
//  之前那张卡图片间距恰好是 1808，跳转正好落在下一张头上所以没事；
//  重新拷贝后布局一变就中招 —— 表现是「只找到前几张」。
//
//  为什么去掉跳转是安全的：命中判据极严，必须同时满足
//      byte[0..1]  = "BM"
//      byte[18..21]= 640
//      byte[22..25]= 480
//      byte[28..29]= 24
//  四个字段全中。BMP 像素数据里凑齐这四个的概率可以忽略，
//  所以一张图只会被认出一次（它的文件头所在位置）。
//
//  代价：扫描会走过每张图的内部（每张多 225 次读），
//  但总读数 131071/8 = 16384 次，仍然约 2.7 秒。
//  好处：不管图片怎么排布、间距多少、卡碎不碎，都能找全。
// ==========================================================================
assign next_scan_sector_if_match = scan_sector + 32'd8;
// 未命中时一次前进 8 个扇区，不是一个。
//
// BMP 文件在 FAT32 上必然从【簇边界】开始；现代 SD 卡的簇不小于 4 KB
// （= 8 个扇区），而且簇大小一定是 2 的幂，所以所有簇边界都是 8 的倍数。
// 因此跨 8 个扇区取样【不会漏掉任何文件头】，但扫描快 8 倍：
// 逐扇区走扫 67 MB 要 20 秒（板上表现为数码管卡在 2、一直黑屏），
// 步进 8 之后只要 2.5 秒，于是可以把范围恢复成全盘，不会漏图。
//
// 唯一的例外是 <=256 MB 的老卡（FAT32 簇只有 512 字节 = 1 个扇区），
// 那种卡步进 8 会漏文件。真遇到就把这里的 8 改回 1。
assign next_scan_sector_if_miss  = scan_sector + 32'd8;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

// Latched geometry for the whole data phase.  Captured when a valid load
// header ends, i.e. long before the first pixel byte, so it cannot lag behind.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        src_w_r      <= 12'd0;
        src_h_r      <= 12'd0;
        row_bytes_r  <= 14'd0;
        row_stride_r <= 14'd0;
        px_total_r   <= 24'd0;
    end else if ((state == ST_LOAD_HDR) && sd_sec_read_end && header_match) begin
        src_w_r      <= src_w12;
        src_h_r      <= height[11:0];
        row_bytes_r  <= row_bytes_raw;
        row_stride_r <= row_stride_raw;
        px_total_r   <= px_total_raw;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

// 行内字节计数（含补位）/ 行内像素 / 总像素计数
always @(posedge clk or posedge rst) begin
    if (rst) begin
        byte_in_row <= 14'd0;
        px_in_row   <= 12'd0;
        px_cnt      <= 24'd0;
    end else if (state != ST_LOAD_DATA) begin
        byte_in_row <= 14'd0;
        px_in_row   <= 12'd0;
        px_cnt      <= 24'd0;
    end else if (sd_sec_read_data_valid) begin
        if (bmp_len_cnt >= pixel_offset) begin
            // 像素区内：行内按字节推进；走满 row_stride 个字节（含补位）就换行
            if (byte_in_row == (row_stride_r - 14'd1)) begin
                byte_in_row <= 14'd0;
                px_in_row   <= 12'd0;
            end else begin
                byte_in_row <= byte_in_row + 14'd1;
            end
            // 一个像素 = 3 个像素字节，收满第 3 个才计数
            if (bmp_byte_is_pix && (bmp_byte_idx == 2'd2))
                px_in_row <= px_in_row + 12'd1;
        end else begin
            byte_in_row <= 14'd0;               // 文件头阶段
        end
        if (bmp_byte_is_pix && (bmp_byte_idx == 2'd2))
            px_cnt <= px_cnt + 24'd1;
    end
end

// 首像素标志（SOF）：进入数据阶段置起，第一个 bmp_data_wr_en 那一拍清掉
always @(posedge clk or posedge rst) begin
    if (rst) begin
        first_px <= 1'b0;
    end else if ((state == ST_LOAD_WAIT) && write_req_ack) begin
        first_px <= 1'b1;
    end else if (bmp_data_wr_en) begin
        first_px <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 4'd0;
        scan_sector       <= 32'd0;
        cool_cnt          <= 16'd0;
        cool_pending      <= 1'b0;
        load_sector_latched <= 32'd0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 4'd0;
        scan_sector       <= 32'd0;
        cool_cnt          <= 16'd0;
        cool_pending      <= 1'b0;
        load_sector_latched <= 32'd0;
    end else begin
        scan_found_valid <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 4'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 4'd1;

                        if (scan_found_total >= scan_target_count - 4'd1) begin
                            //  达到目标张数：不立刻 scan_done，先进入冷却
                            //  （继续按 +8 空走，走完 COOL_STEPS 步才真正停）
                            cool_pending     <= 1'b1;
                            cool_cnt         <= COOL_STEPS;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else if (next_scan_sector_if_match > scan_max_sector) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        if (cool_pending) begin
                            //  冷却中：继续空走，走完 COOL_STEPS 步才 scan_done
                            if (cool_cnt == 16'd0) begin
                                scan_done <= 1'b1;
                                state     <= ST_IDLE;
                            end else begin
                                cool_cnt         <= cool_cnt - 16'd1;
                                sd_sec_read_addr <= next_scan_sector_if_miss;
                                scan_sector      <= next_scan_sector_if_miss;
                            end
                        end else if (scan_sector >= scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code        <= 4'd3;
                load_next_pending <= 1'b0;      // 进入数据阶段：第一枪用锁存地址
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            // ==============================================================
            //  数据阶段：发源像素 + 唯一的反压点
            //
            //  结束条件用【像素数】而不是文件长度：收满 w*h 个源像素就够了
            //  （BMP 尾部可能还有多余字节，不必读）。
            //
            //  sd_sec_read 是电平，拉低就停在扇区边界 —— 这是唯一的反压手段。
            //  没有它，极端长宽比（如 640x10 放大到 640x480）时 scaler 要发
            //  480 行 ≈ 6.7 ms，而源 6.4 ms 就灌完了，输入 FIFO 必然溢出。
            // ==============================================================
            ST_LOAD_DATA: begin
                state_code <= 4'd4;

                if (px_done) begin
                    sd_sec_read       <= 1'b0;
                    load_next_pending <= 1'b0;
                    state             <= ST_IDLE;
                end else if (sd_sec_read) begin
                    if (sd_sec_read_end) begin
                        sd_sec_read       <= 1'b0;
                        load_next_pending <= 1'b1;   // 等 scaler 放行再推进地址
                    end
                end else if (src_ready) begin
                    // 地址在这里递增（而不是在 read_end 那拍），
                    // 这样"等待 src_ready"期间不会重复读同一个扇区
                    if (load_next_pending) begin
                        sd_sec_read_addr  <= sd_sec_read_addr + 32'd1;
                        load_next_pending <= 1'b0;
                    end
                    sd_sec_read <= 1'b1;
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
