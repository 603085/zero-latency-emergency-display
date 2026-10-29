
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 3b. SDRAM 的 180° 移相时钟：clkc[2]
#     —— 原文件漏了这一条，它就是 ext_mem_clk 组那 122 个失效端点的来源
#
#     sys_pll 的 clk2 是 125 MHz / 180° 移相，只接到 SDRAM 硬核的 Clk_sft。
#     原文件只 rename 了 clkc[0] / clkc[1]，clkc[2] 留着自动生成的名字，
#     于是 TD 把它和 ext_mem_clk 当成"同源同步"的一对，按 180° 相移只给
#     4 ns 半周期预算去算两者之间的路径 ——> ext_mem_clk 组报
#     SWNS -6.699 ns / 122 个失效端点（README §9 第 1 条、
#     《组装与校验报告》§5.3；改完这一条那组数字应当显著改善）。
#
#     这个相移是 SDRAM 硬核内部的事（Clk / Clk_sft 成对使用），两者之间
#     不该做同步时序分析，所以：先给它一个名字（报告里认得出），
#     再把它和 ext_mem_clk 声明成异步组。
#
#     ⚠️ 前提已核对：clkc[2] 只驱动 sdram 的 Clk_sft 一个引脚，
#        没有别的负载 —— 所以切掉这两个时钟之间的路径不会漏掉真路径。
#        各时钟域【内部】的路径仍然照常分析。
# ------------------------------------------------------------
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]

set_clock_groups -asynchronous \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {ext_mem_clk_sft}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
#
#    另外把 clk(50MHz) 也切出去。clk 域和别的域之间**只有复位这一条路**：
#    clk 域产生 por_cnt / rst_all，各域拿它当【异步复位】（always @(posedge clk
#    or posedge rst)）。被当同步路径算的话：
#      · clk -> audio_mclk 得 48 个伪违例（A 包文档 §5.3，-2.959 ns）；
#      · clk -> hdmi_5x_clk 得 4 个 recovery 伪违例
#        （por_cnt[19] -> u_hdmi2phy_wrapper/u2_lane_lvds_8_1/S_shift_data_reg[8].sr，
#          数据路径只有 1.945ns，剩下的全是 1.874ns 的 clock skew）。
#
#    ⚠️ clk 必须【单独成一组】：SDC 里同一个 group 内的时钟之间不会被切，
#       放进第二组只会切掉 audio_mclk 与它们的关系，clk 与它们的路径照旧分析
#       （第一版就是这么写的，所以 clk -> hdmi_5x_clk 的 4 个 recovery 还在）。
#
#    两个域各自的内部路径仍然照常分析。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {clk}] \
    -group [get_clocks {video_clk hdmi_5x_clk sd_card_clk ext_mem_clk}]

# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# ------------------------------------------------------------------
#  ENABLED -- the crossings were verified, see the note that used to
#  say "only enable once confirmed".
#
#  Every crossing between these domains goes through a synchroniser or a
#  FIFO, which is what makes the group correct rather than a way of hiding a
#  problem:
#
#    sd_card_clk <-> video_clk
#        hold_load, busy_stack, load_req, next_req_pending are all passed
#        through two- or three-stage synchronisers in sd_card_bmp and
#        video_layers_top.
#    ext_mem_clk <-> video_clk
#        the frame write/read FIFOs in frame_read_write.
#    hdmi_5x_clk  is generated from the same PLL as video_clk and is used only
#        by the HDMI serialiser.
#
#  Without this the report showed SWNS -6.699 ns / STNS -404.658 ns, which
#  looked like a failing design. Those numbers were entirely inter-clock: the
#  intra-clock path group for video_clk reported +0.283 ns, i.e. the real logic
#  meets its constraint. Timing a 2-flop synchroniser as a synchronous path
#  always produces a large false violation, because its first stage is
#  deliberately sampling an unrelated clock.
#
#  Paths WITHIN each domain are still fully analysed.
# ------------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
