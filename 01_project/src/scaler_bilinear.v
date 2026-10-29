// ============================================================================
//  scaler_bilinear.v -- 任意分辨率 BMP -> 640x480 双线性缩放（写侧）
//
//  插入位置：bmp_read（源像素流）与 frame_read_write 的写侧之间。
//  进 SDRAM 的永远是 640x480x24，所以 SDRAM / 读侧 / 显示 / 转场 / OSD 都不用改。
//
//  算法（角点对齐、定点双线性）：
//      sx = ox*(src_w-1)/(OUT_W-1)      sy = oy*(src_h-1)/(OUT_H-1)
//      out = lerp( lerp(prev[x0],prev[x1],fx), lerp(cur[x0],cur[x1],fx), fy )
//      lerp(a,b,f) = a + ((b-a)*f >> F)                 全无符号，F = 16
//      （单乘法写法：与 (a*(2^F-f) + b*f)>>F 数学等价，但乘法器减半 18→9；
//        取整方式略有不同，所以 Python 参考实现用同一式子，比对仍是逐位一致）
//  角点对齐的好处：两端严格对齐（半像素偏移会让整幅图看起来"位移"），
//  坐标非负，只需要 x1 的右边界钳位；y 方向由"行对"结构天然处理边界。
//
//  流水与缓冲（速度上真正的约束是 SD SPI ≈1 像素/us，缩放器只要不当瓶颈）：
//    · 收行：FIFO -> 行缓冲；收满一行切到另一块缓冲（ping-pong）
//    · 发射：每个输出像素 10 拍计算（E_RD0..E_VOUT，★ 2026-10-13 流水改造后）
//            + E_GAP（1 + EMIT_DIV 默认16 = 17）拍间隔 → 27 拍/像素
//            ≈ 3.7 Mpixel/s，远高于写侧需要（SPI 一帧 ~0.5 s）-> 不需要加反压
//    · 发射一行约 640*27 = 17 k 拍 ≈ 173 us，期间源会送来约 170 像素，
//      所以带 512 深输入 FIFO；同时给出 src_ready 让 bmp_read 在扇区边界节流
//      （一个扇区最多 171 像素，FIFO 留了 192 的余量）
//
//  ★ 2026-10-13 流水改造（修复 100 MHz 时序）：
//      原实现把【行缓冲 RAM 输出 → 减法 → 18×18 乘法 → 加法】全部塞在
//      E_HLERP 一个 100 MHz 周期里 —— 实测数据路径 13.389 ns（预算 10 ns），
//      造成 sd_card_clk WNS −3.669 ns / 146 个失效端点、该域 Fmax 掉到 73 MHz。
//      现在水平插值（6 次）与垂直插值（3 次）各拆成三级：求差 → 乘法 → 加法，
//      并且 x1 的读数据先打进寄存器（e_p1/e_c1），乘法器输入端全是寄存器。
//      数学与位宽和原 lerp8() 逐位等价（切片取的就是 prod[FW +: 9]）。
//    · 一帧发完（OUT_H 行）后，源还有剩余像素（缩小的情况）→ 进入 S_DRAIN
//      把 FIFO 排空、把 w*h 个源像素等完，避免把 bmp_read 卡死
//
//  规模：2 块行缓冲 MAX_SRC_W*24bit（2048 -> 98 Kb ≈ 12 个 9Kb BRAM）
//        + 输入 FIFO 512*24bit。器件 64 个 9Kb BRAM，当前设计用 20 个。
//
//  ⚠️ 源宽 > MAX_SRC_W 不支持：bmp_read 应按"头不匹配"跳过；这里拉高 too_wide 提示。
// ============================================================================

`timescale 1ns / 1ps

module scaler_bilinear #(
    parameter integer OUT_W      = 640,
    parameter integer OUT_H      = 480,
    parameter integer MAX_SRC_W  = 2048,
    parameter integer FRAC_BITS  = 16,
    parameter integer FIFO_DEPTH = 512,
    parameter integer EMIT_DIV   = 16
)(
    input  wire        clk,             // sd_card_clk 100 MHz
    input  wire        rst,             // 高有效

    // ---------------- 源像素流（来自 bmp_read） ----------------
    input  wire        src_valid,
    input  wire [23:0] src_rgb,         // {R,G,B}
    input  wire        src_sof,         // 一帧第一个像素：锁存宽高、清累加器
    input  wire [11:0] src_w,
    input  wire [11:0] src_h,
    output wire        src_ready,       // 允许上游继续（扇区级节流）

    // ---------------- 目标像素流（送 frame_read_write 写侧） ----------------
    output reg         dst_wr_en,
    output reg  [23:0] dst_rgb,
    output reg         dst_frame_done,
    output reg         too_wide
);

    localparam integer FW   = FRAC_BITS;
    localparam integer FULL = (1 << FW);
    localparam integer FPTR = $clog2(FIFO_DEPTH);

    // ==================================================================
    //  输入 FIFO
    // ==================================================================
    // 输入 FIFO：读是【异步】的（组合读 head），所以只能落在 LUT 分布式 RAM 上
    // （512x24 ≈ 768 个 LUT4 当 16x1 RAM 用）。这块不大，可以接受；
    // 真正贵的是行缓冲（98 kbit），必须显式要求 BRAM —— 见下面 line_a/line_b。
    reg [23:0]    fifo [0:FIFO_DEPTH-1];
    reg [FPTR:0]  fifo_wp, fifo_rp;
    wire [FPTR:0] fifo_cnt   = fifo_wp - fifo_rp;
    wire          fifo_empty = (fifo_wp == fifo_rp);
    wire          fifo_room  = (fifo_cnt < (FIFO_DEPTH - 192));
    assign        src_ready  = fifo_room;

    // 行缓冲：必须映射到 BRAM。
    //  · TD 认的是 Verilog 属性 (* ram_style="bram" *)（见 TD 自带 ip\atmr\memory_spram.v），
    //    IP 文件里那种 /* fehdl force_ram=1 */ 注释 pragma 在**模块自己的代码里**
    //    会被理解成"寄存器堆" —— 实测 seq 从 465 暴涨到 111057（≈ 全部存储位数的 FF），
    //    所以这里用属性形式。
    //  · 写、读各放一个独立的 always 块（TD 模板就是这种最保守的写法）；
    //    读是同步的（rd_addr -> rda_q/rdb_q 一拍延迟，发射流水已按这个延迟对齐）。
    localparam integer AW = $clog2(MAX_SRC_W);   // 行缓冲地址位宽（必须精确等于深度）
    (* ram_style="bram" *) reg [23:0] line_a [0:MAX_SRC_W-1];
    (* ram_style="bram" *) reg [23:0] line_b [0:MAX_SRC_W-1];
    reg [23:0] rda_q, rdb_q;
    reg [AW-1:0] rd_addr;

    always @(posedge clk) rda_q <= line_a[rd_addr];
    always @(posedge clk) rdb_q <= line_b[rd_addr];

    // 上电清零（TD 自带 RAM IP 也是这么写的；有些推断流程需要它才肯认成存储器）
    integer li;
    initial begin
        for (li = 0; li < MAX_SRC_W; li = li + 1) begin
            line_a[li] = 24'd0;
            line_b[li] = 24'd0;
        end
    end

    reg [AW-1:0] wr_addr;
    reg [11:0] row_cnt;         // 已收完的源行数（= 当前正在收的行号）
    reg        cur_is_b;        // 已收完的"当前行"是否在 line_b（收完一行时更新）

    // ---- 主状态机（声明必须放在下面 line_we 之前：Verilog 要求先声明后用）----
    localparam [2:0] S_IDLE = 3'd0, S_DIV = 3'd1, S_FILL = 3'd2,
                     S_EMIT = 3'd3, S_DRAIN = 3'd4, S_DONE = 3'd5;
    reg [2:0]  state;

    // 行缓冲写：与 S_FILL 的状态/地址推进同拍、但放在独立的简单 always 块里
    wire        line_we   = (state == S_FILL) && !fifo_empty;
    wire        fill_to_b = row_cnt[0];    // 第 j 行（j 为奇数）写 line_b
    always @(posedge clk) if (line_we && !fill_to_b) line_a[wr_addr] <= fifo[fifo_rp[FPTR-1:0]];
    always @(posedge clk) if (line_we &&  fill_to_b) line_b[wr_addr] <= fifo[fifo_rp[FPTR-1:0]];

    wire [23:0] rd_prev = cur_is_b ? rda_q : rdb_q;
    wire [23:0] rd_cur  = cur_is_b ? rdb_q : rda_q;

    // ==================================================================
    //  定点步进：step = ((n-1) << F) / (OUT-1)，逐位恢复除法（每帧两次，各 32 拍）
    // ==================================================================
    reg [31:0] dv;              // 被除数（左移）
    reg [16:0] rm;              // 部分余数
    reg [31:0] q;
    reg [5:0]  dcnt;
    reg        div_y;
    reg [31:0] step_x, step_y;
    wire [16:0] den_cur = div_y ? (OUT_H[16:0] - 17'd1) : (OUT_W[16:0] - 17'd1);

    reg [31:0] x_acc, y_acc;
    reg [11:0] out_x, out_y;
    reg [11:0] src_w_lat, src_h_lat;
    reg [23:0] src_cnt;                     // 已收到的源像素数（用于排空）
    wire [23:0] src_total = src_w_lat * src_h_lat;
    wire [11:0] need_y0 = y_acc[FW+11:FW];

    // ==================================================================
    //  发射子状态机（主状态机的声明已提到上面 line_we 之前）
    // ==================================================================
    localparam [3:0] E_DEC   = 4'd0,  E_RD0   = 4'd1,  E_RD1   = 4'd2,
                     E_RD2   = 4'd3,
                     // ★ 2026-10-13 流水改造新增：把插值拆成 求差/乘法/加法 三级
                     E_LAT   = 4'd4,   // x1 的 prev/cur 打进寄存器
                     E_DIFF  = 4'd5,   // 水平插值 1/3：d = x1 − x0
                     E_HPROD = 4'd6,   // 水平插值 2/3：p = (d*fx) 的 prod[FW +: 9]
                     E_HADD  = 4'd7,   // 水平插值 3/3：out = x0 + p
                     E_VDIFF = 4'd8,   // 垂直插值 1/3：d = bot − top
                     E_VPROD = 4'd9,   // 垂直插值 2/3：p = (d*fy) 的 prod[FW +: 9]
                     E_VOUT  = 4'd10,  // 垂直插值 3/3：加回并输出（dst_wr_en=1）
                     E_GAP   = 4'd11;
    reg [3:0]  e_state;
    reg [4:0]  emit_div;
    reg        e_same_row;
    reg [11:0] e_x1;
    reg [FW-1:0] e_fx, e_fy;
    reg [23:0] e_p0, e_c0, e_p1, e_c1;
    reg [7:0]  e_topr, e_topg, e_topb, e_botr, e_botg, e_botb;

    // ---- 流水改造新增的中间级寄存器 ----
    reg signed [8:0] h_dtr, h_dtg, h_dtb;   // prev 行：x1 − x0
    reg signed [8:0] h_dcr, h_dcg, h_dcb;   // cur  行：x1 − x0
    reg [8:0]        h_ptr, h_ptg, h_ptb;   // ((x1−x0)*fx) 的 prod[FW +: 9]
    reg [8:0]        h_pcr, h_pcg, h_pcb;
    reg signed [8:0] v_dr, v_dg, v_db;      // 垂直：bot − top
    reg [8:0]        v_pr, v_pg, v_pb;      // ((bot−top)*fy) 的 prod[FW +: 9]

    // 与 lerp8 里的乘法结果切片【逐位一致】：25 位有符号乘积，取 [FW +: 9]
    function [8:0] hmul;
        input signed [8:0]    dd;
        input        [FW-1:0] ff;
        reg   signed [24:0]   pp;
        begin
            pp   = dd * $signed({1'b0, ff});
            hmul = pp[FW +: 9];
        end
    endfunction

    // 单乘法插值：out = a + ((b-a)*f >> F)
    //   与 (a*(FULL-f) + b*f)>>F 数学等价，但**乘法器减半（18 → 9）** ——
    //   实测两乘法版本把器件 29 个 DSP 全占了（#dsp 29/29 100%），必须降下来。
    //   取整方式略有不同（先算差再乘），所以 Python 参考实现同步改成同一式子，
    //   仿真比对仍然是"逐位一致"，不是"看起来差不多"。
    //
    //   ⚠️ 2026-10-13 流水改造后，插值实际由 hmul() + E_DIFF/E_HPROD/E_HADD
    //      （以及垂直的 E_VDIFF/E_VPROD/E_VOUT）三级完成，本函数已【不再被调用】，
    //      保留在这里作为"位宽与取整等价"的参考实现（pp[FW +: 9] 就是 hmul 取的位）。
    function [7:0] lerp8;
        input [7:0]    aa;
        input [7:0]    bb;
        input [FW-1:0] ff;
        reg signed [8:0]  diff;
        reg signed [24:0] prod;
        begin
            diff  = $signed({1'b0, bb}) - $signed({1'b0, aa});
            prod  = diff * $signed({1'b0, ff});
            lerp8 = aa + prod[FW +: 9];      // 等价于 aa + (prod >>> FW)，再截到 8 位
        end
    endfunction

    // 当前正在收的行应该写哪个数组：第 j 行 j 为奇数 -> b
    // （放在这里是因为上面的 line_we 写块要用它，Verilog 要求先声明）
    // wire fill_to_b 见上方行缓冲区

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            fifo_wp <= 0; fifo_rp <= 0;
            wr_addr <= {AW{1'b0}};
            row_cnt <= 12'd0; cur_is_b <= 1'b0;
            rd_addr <= {AW{1'b0}};
            dv <= 32'd0; rm <= 17'd0; q <= 32'd0; dcnt <= 6'd0; div_y <= 1'b0;
            step_x <= 32'd0; step_y <= 32'd0;
            x_acc <= 32'd0; y_acc <= 32'd0; out_x <= 12'd0; out_y <= 12'd0;
            src_cnt <= 24'd0;
            e_state <= E_DEC; emit_div <= 5'd0; e_same_row <= 1'b0;
            e_x1 <= 12'd0; e_fx <= 0; e_fy <= 0;
            e_p0 <= 24'd0; e_c0 <= 24'd0; e_p1 <= 24'd0; e_c1 <= 24'd0;
            e_topr <= 8'd0; e_topg <= 8'd0; e_topb <= 8'd0;
            e_botr <= 8'd0; e_botg <= 8'd0; e_botb <= 8'd0;
            h_dtr <= 9'sd0; h_dtg <= 9'sd0; h_dtb <= 9'sd0;
            h_dcr <= 9'sd0; h_dcg <= 9'sd0; h_dcb <= 9'sd0;
            h_ptr <= 9'd0;  h_ptg <= 9'd0;  h_ptb <= 9'd0;
            h_pcr <= 9'd0;  h_pcg <= 9'd0;  h_pcb <= 9'd0;
            v_dr  <= 9'sd0; v_dg  <= 9'sd0; v_db  <= 9'sd0;
            v_pr  <= 9'd0;  v_pg  <= 9'd0;  v_pb  <= 9'd0;
            src_w_lat <= 12'd0; src_h_lat <= 12'd0;
            dst_wr_en <= 1'b0; dst_rgb <= 24'd0;
            dst_frame_done <= 1'b0; too_wide <= 1'b0;
        end else begin
            dst_wr_en      <= 1'b0;
            dst_frame_done <= 1'b0;

            // ---------------- 源像素入 FIFO + 计数 ----------------
            if (src_valid) begin
                fifo[fifo_wp[FPTR-1:0]] <= src_rgb;
                fifo_wp  <= fifo_wp + 1'b1;
                src_cnt  <= src_cnt + 24'd1;
            end

            if (src_sof) begin
                // ---------------- 帧起点 ----------------
                src_w_lat <= src_w;
                src_h_lat <= src_h;
                too_wide  <= (src_w > MAX_SRC_W[11:0]);
                wr_addr   <= {AW{1'b0}};
                row_cnt   <= 12'd0;
                cur_is_b  <= 1'b0;
                x_acc     <= 32'd0;
                y_acc     <= 32'd0;
                out_x     <= 12'd0;
                out_y     <= 12'd0;
                src_cnt   <= src_valid ? 24'd1 : 24'd0;   // 本拍这个像素也算
                e_state   <= E_DEC;
                dv        <= ({20'd0, src_w} - 32'd1) << FW;
                rm        <= 17'd0;
                q         <= 32'd0;
                dcnt      <= 6'd32;
                div_y     <= 1'b0;
                state     <= S_DIV;
            end else begin
                case (state)
                    // ============== 逐位恢复除法（32 拍 / 次） ==============
                    S_DIV: begin
                        if (dcnt == 6'd0) begin
                            if (!div_y) begin
                                step_x <= q;
                                dv     <= ({20'd0, src_h_lat} - 32'd1) << FW;
                                rm     <= 17'd0;
                                q      <= 32'd0;
                                dcnt   <= 6'd32;
                                div_y  <= 1'b1;
                            end else begin
                                step_y <= q;
                                state  <= S_FILL;
                            end
                        end else begin
                            if ({rm[15:0], dv[31]} >= den_cur) begin
                                rm <= {rm[15:0], dv[31]} - den_cur;
                                q  <= {q[30:0], 1'b1};
                            end else begin
                                rm <= {rm[15:0], dv[31]};
                                q  <= {q[30:0], 1'b0};
                            end
                            dv   <= {dv[30:0], 1'b0};
                            dcnt <= dcnt - 6'd1;
                        end
                    end

                    // ============== 收一行源像素 ==============
                    // 注意：行缓冲的实际写入已经在上面 line_we 那两个简单 always 块里做了，
                    // 这里只负责推进 fifo_rp / wr_addr / 行计数（与写入同拍同地址）。
                    S_FILL: begin
                        if (!fifo_empty) begin
                            fifo_rp <= fifo_rp + 1'b1;

                            if (wr_addr == (src_w_lat - 12'd1)) begin
                                wr_addr  <= {AW{1'b0}};
                                row_cnt  <= row_cnt + 12'd1;
                                cur_is_b <= fill_to_b;      // 刚收完的这一行在哪
                                e_state  <= E_DEC;
                                state    <= S_EMIT;
                            end else begin
                                wr_addr <= wr_addr + 1'b1;
                            end
                        end
                    end

                    // ============== 发射输出行 ==============
                    S_EMIT: begin
                        case (e_state)
                            // ---- 决定现在能不能发、用哪两行 ----
                            E_DEC: begin
                                if (out_y == OUT_H[11:0]) begin
                                    state   <= (src_cnt >= src_total) ? S_DONE : S_DRAIN;
                                end else if (src_h_lat == 12'd1) begin
                                    // 只有一行：上下都用它
                                    e_same_row <= 1'b1;
                                    x_acc      <= 32'd0;
                                    out_x      <= 12'd0;
                                    e_state    <= E_RD0;
                                end else if (row_cnt <= 12'd1) begin
                                    // 还没有"上一行"，收下一行
                                    state <= S_FILL;
                                end else if (need_y0 == (row_cnt - 12'd2)) begin
                                    e_same_row <= 1'b0;
                                    x_acc      <= 32'd0;
                                    out_x      <= 12'd0;
                                    e_state    <= E_RD0;
                                end else if ((row_cnt == src_h_lat) && (need_y0 == (row_cnt - 12'd1))) begin
                                    // 最后一行：y1 钳位成同一行
                                    e_same_row <= 1'b1;
                                    x_acc      <= 32'd0;
                                    out_x      <= 12'd0;
                                    e_state    <= E_RD0;
                                end else if (row_cnt >= src_h_lat) begin
                                    // 源行已收完但没有匹配的输出行（理论上不该发生）
                                    state <= (src_cnt >= src_total) ? S_DONE : S_DRAIN;
                                end else begin
                                    state <= S_FILL;
                                end
                            end

                            // ---- 取 x0 地址 + 记小数 ----
                            E_RD0: begin
                                e_fx    <= x_acc[FW-1:0];
                                e_fy    <= y_acc[FW-1:0];
                                e_x1    <= (x_acc[FW+11:FW] >= (src_w_lat - 12'd1))
                                           ? x_acc[FW+11:FW] : (x_acc[FW+11:FW] + 12'd1);
                                rd_addr <= x_acc[FW+AW-1:FW];
                                e_state <= E_RD1;
                            end

                            // ---- 取 x1 地址 ----
                            // 注意流水深度：rd_addr 是寄存器，line[] 的读数据又比地址晚一拍，
                            // 所以"E_RD0 写的地址"要到 E_RD2 才变成 rda_q/rdb_q 上的有效数据。
                            // 早先在这里就采集 p0/c0，拿到的是【上一个地址】的数据（实测
                            // c0 取到了 x=1 的红点），一定要留意这个两拍延迟。
                            E_RD1: begin
                                rd_addr <= e_x1[AW-1:0];
                                e_state <= E_RD2;
                            end

                            // ---- 收 x0 处的 prev/cur ----
                            E_RD2: begin
                                e_p0 <= rd_prev;
                                e_c0 <= rd_cur;
                                e_state <= E_LAT;   // ★ 原来指向 E_HLERP（已拆成三级）
                            end

                            // ---- ★ 流水改造：把 x1 的 prev/cur 先打进寄存器 ----
                            // 关键一步：从这一拍起乘法器的输入全来自寄存器，
                            // 行缓冲 RAM 的输出（ramread.clkb）被切断在乘法器之外 ——
                            // 那正是原来 13.389 ns 那条路径的起点。
                            E_LAT: begin
                                e_p1 <= rd_prev;
                                e_c1 <= rd_cur;
                                e_state <= E_DIFF;
                            end

                            // ---- 水平插值 1/3：d = x1 − x0（1 个减法） ----
                            E_DIFF: begin
                                h_dtr <= $signed({1'b0,e_p1[23:16]}) - $signed({1'b0,e_p0[23:16]});
                                h_dtg <= $signed({1'b0,e_p1[15:8]})  - $signed({1'b0,e_p0[15:8]});
                                h_dtb <= $signed({1'b0,e_p1[7:0]})   - $signed({1'b0,e_p0[7:0]});
                                h_dcr <= $signed({1'b0,e_c1[23:16]}) - $signed({1'b0,e_c0[23:16]});
                                h_dcg <= $signed({1'b0,e_c1[15:8]})  - $signed({1'b0,e_c0[15:8]});
                                h_dcb <= $signed({1'b0,e_c1[7:0]})   - $signed({1'b0,e_c0[7:0]});
                                e_state <= E_HPROD;
                            end

                            // ---- 水平插值 2/3：p = (d*fx) 的 prod[FW +: 9]（1 个乘法） ----
                            E_HPROD: begin
                                h_ptr <= hmul(h_dtr, e_fx);
                                h_ptg <= hmul(h_dtg, e_fx);
                                h_ptb <= hmul(h_dtb, e_fx);
                                h_pcr <= hmul(h_dcr, e_fx);
                                h_pcg <= hmul(h_dcg, e_fx);
                                h_pcb <= hmul(h_dcb, e_fx);
                                e_state <= E_HADD;
                            end

                            // ---- 水平插值 3/3：out = x0 + p（1 个加法） ----
                            E_HADD: begin
                                e_topr <= e_p0[23:16] + h_ptr;
                                e_topg <= e_p0[15:8]  + h_ptg;
                                e_topb <= e_p0[7:0]   + h_ptb;
                                e_botr <= e_c0[23:16] + h_pcr;
                                e_botg <= e_c0[15:8]  + h_pcg;
                                e_botb <= e_c0[7:0]   + h_pcb;
                                e_state <= E_VDIFF;
                            end

                            // ---- 垂直插值 1/3：d = bot − top ----
                            E_VDIFF: begin
                                v_dr <= $signed({1'b0,e_botr}) - $signed({1'b0,e_topr});
                                v_dg <= $signed({1'b0,e_botg}) - $signed({1'b0,e_topg});
                                v_db <= $signed({1'b0,e_botb}) - $signed({1'b0,e_topb});
                                e_state <= E_VPROD;
                            end

                            // ---- 垂直插值 2/3：p = (d*fy) 的 prod[FW +: 9] ----
                            E_VPROD: begin
                                v_pr <= hmul(v_dr, e_fy);
                                v_pg <= hmul(v_dg, e_fy);
                                v_pb <= hmul(v_db, e_fy);
                                e_state <= E_VOUT;
                            end

                            // ---- 垂直插值 3/3：加回并输出 ----
                            // ⚠️ 必须逐通道写 8 位 part-select：`{a+v, b+v, c+v}` 这种拼接里
                            //    每个操作数是【自定宽】（8 位 + 9 位 → 9 位），拼接会变成 27 位，
                            //    赋给 24 位的 dst_rgb 会丢掉高 3 位 → R/G/B 全体错位。
                            //    写成 part-select 后，每个表达式在 8 位上下文里求值再截断，
                            //    与 lerp8() 的 8 位返回严格同义（mod 256）。
                            E_VOUT: begin
                                if (e_same_row) begin
                                    dst_rgb <= {e_botr, e_botg, e_botb};
                                end else begin
                                    dst_rgb[23:16] <= e_topr + v_pr;
                                    dst_rgb[15:8]  <= e_topg + v_pg;
                                    dst_rgb[7:0]   <= e_topb + v_pb;
                                end
                                dst_wr_en <= 1'b1;
                                x_acc     <= x_acc + step_x;
                                emit_div  <= EMIT_DIV[4:0];
                                e_state   <= E_GAP;
                            end

                            // ---- 限速 + 行内推进 ----
                            E_GAP: begin
                                if (emit_div != 5'd0) begin
                                    emit_div <= emit_div - 5'd1;
                                end else if (out_x == (OUT_W[11:0] - 12'd1)) begin
                                    out_x   <= 12'd0;
                                    out_y   <= out_y + 12'd1;
                                    y_acc   <= y_acc + step_y;
                                    e_state <= E_DEC;
                                end else begin
                                    out_x   <= out_x + 12'd1;
                                    e_state <= E_RD0;
                                end
                            end

                            default: e_state <= E_DEC;
                        endcase
                    end

                    // ============== 排空剩下的源像素 ==============
                    // 缩小的情况下，输出行早就发完了，但源还有行没读完：
                    // 必须继续把 FIFO 排空并把 w*h 个像素等完，否则 bmp_read
                    // 会因为 src_ready=0 停在扇区边界上，整条链卡死。
                    S_DRAIN: begin
                        if (!fifo_empty) fifo_rp <= fifo_rp + 1'b1;
                        if (src_cnt >= src_total) state <= S_DONE;
                    end

                    S_DONE: begin
                        dst_frame_done <= 1'b1;
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
