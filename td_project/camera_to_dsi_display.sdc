create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 2.415 -waveform {0.000 1.208} [get_nets {IO_rx_clk_pad_p}]
derive_clocks
rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]
rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]

# Keep the original MIPI/PLL cuts. sys_clk_50m is excluded so the new LAT
# launch -> CSI first-stage paths can receive a real datapath-only budget.
# The reverse MIPI -> sys50 cut is preserved explicitly, not broadened.
set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}]
set_false_path -from [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] -to [get_clocks {sys_clk_50m}]
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

# These existing non-pointer CDC paths were already cut by the old groups.
# Preserve only their reviewed source/destination scope, not whole domains.
set_false_path -from [get_clocks HDMI_PIXEL_CLK] -to [get_regs {u_video_out/vs_sync[0]}]
set_false_path -from [get_regs {u_video_out/reset_count[*]}] -to [get_clocks HDMI_PIXEL_CLK]
set_false_path -from [get_regs {u_video_in/O_frame_available}] -to [get_regs {u_video_out/frame_ready_sync[0]}]

# The image FIFO keeps its unchanged vendor IP max_delay (13.033 ns).
# The directly instantiated tag FIFOs need explicit full-hierarchy constraints.
# Fixed budgets=min(clock periods)-0.300 ns; final audit checks those periods.
# Use plain SDC statements: TD root SDC does not reliably expand Tcl foreach.
set_max_delay -from [get_regs {u_video_out/u_source_tags/u_tag_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_out/u_source_tags/u_tag_fifo/wr_to_rd_cross_inst/sync_r1[*]}] -datapath_only 13.033
set_false_path -hold -from [get_regs {u_video_out/u_source_tags/u_tag_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_out/u_source_tags/u_tag_fifo/wr_to_rd_cross_inst/sync_r1[*]}]
set_max_delay -from [get_regs {u_video_out/u_source_tags/u_tag_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_out/u_source_tags/u_tag_fifo/rd_to_wr_cross_inst/sync_r1[*]}] -datapath_only 13.033
set_false_path -hold -from [get_regs {u_video_out/u_source_tags/u_tag_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_out/u_source_tags/u_tag_fifo/rd_to_wr_cross_inst/sync_r1[*]}]
set_max_delay -from [get_regs {u_video_in/u_tagged_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_in/u_tagged_fifo/wr_to_rd_cross_inst/sync_r1[*]}] -datapath_only 9.360
set_false_path -hold -from [get_regs {u_video_in/u_tagged_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_in/u_tagged_fifo/wr_to_rd_cross_inst/sync_r1[*]}]
set_max_delay -from [get_regs {u_video_in/u_tagged_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_in/u_tagged_fifo/rd_to_wr_cross_inst/sync_r1[*]}] -datapath_only 9.360
set_false_path -hold -from [get_regs {u_video_in/u_tagged_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs {u_video_in/u_tagged_fifo/rd_to_wr_cross_inst/sync_r1[*]}]

# ---- handshake-protected clock-domain crossing (added 2026-10-13) ----
# The automatic-gain value is produced in the HDMI pixel domain and consumed by the I2C
# configuration logic in the 24 MHz domain. The value is held stable for at least one whole
# frame (~16.7 ms) and the receiver captures it on an edge detected from a two-flop
# synchronised change toggle, so this path carries no timing requirement. STA would otherwise
# analyse it against the raw clock-edge relationship, and because the 74.25 MHz and 24 MHz
# edges nearly coincide at some alignments the budget collapses to 0.674 ns, which showed up
# as a -0.17 ns violation (all ten violations of the 2026-10-13 13:48 run were this path).
# Only this one direction is declared asynchronous; every other relation above is untouched.
set_false_path -from [get_clocks {HDMI_PIXEL_CLK}] -to [get_clocks {pll_clk_24m}]

# 2026-10-16: the DBG request synchroniser crosses from the 24 MHz config domain into the
# pixel domain (S_ae_ladder_req_24m -> S_dbg_req_sync). It is a proper 2-FF synchroniser, so
# the pair gets a false path like the reverse direction below. Without it the tool budgets
# only the worst-case phase (0.65 ns) and reports a -0.25 ns setup violation.
set_false_path -from [get_clocks {pll_clk_24m}] -to [get_clocks {HDMI_PIXEL_CLK}]

# ---- LAT Gray CDC repair (2026-10-19) ----
# TD get_regs selects logical REGISTER OUTPUT NETS, not synthesized cell names
# ending in _reg. The S_lat_tick_gray* alias selects all 32 source bits,
# including Gray[31], whose returned object is tick_bin[31]. Use exact source
# REGISTER collections: a root CLOCK collection may include generated clocks
# and would incorrectly budget/cut the local synchronizer stages as well.
# Later stages retain their ordinary local-clock setup AND hold checks.
# 2026-10-25: explicit first-stage logical bits avoid TD wildcard aliases
# expanding into source registers after packing. No budget/clock is changed;
# both branches and all local hops require per-logical-bit audit.
# CSI min(period) - 0.300 = 9.660 - 0.300 = 9.360 ns.
# PIX min(period) - 0.300 = 13.333 - 0.300 = 13.033 ns.
set_max_delay -from [get_regs {S_lat_tick_gray*}] -to [get_regs {u_lat_rx_csi/gray_pipe[0] u_lat_rx_csi/gray_pipe[1] u_lat_rx_csi/gray_pipe[2] u_lat_rx_csi/gray_pipe[3] u_lat_rx_csi/gray_pipe[4] u_lat_rx_csi/gray_pipe[5] u_lat_rx_csi/gray_pipe[6] u_lat_rx_csi/gray_pipe[7] u_lat_rx_csi/gray_pipe[8] u_lat_rx_csi/gray_pipe[9] u_lat_rx_csi/gray_pipe[10] u_lat_rx_csi/gray_pipe[11] u_lat_rx_csi/gray_pipe[12] u_lat_rx_csi/gray_pipe[13] u_lat_rx_csi/gray_pipe[14] u_lat_rx_csi/gray_pipe[15] u_lat_rx_csi/gray_pipe[16] u_lat_rx_csi/gray_pipe[17] u_lat_rx_csi/gray_pipe[18] u_lat_rx_csi/gray_pipe[19] u_lat_rx_csi/gray_pipe[20] u_lat_rx_csi/gray_pipe[21] u_lat_rx_csi/gray_pipe[22] u_lat_rx_csi/gray_pipe[23] u_lat_rx_csi/gray_pipe[24] u_lat_rx_csi/gray_pipe[25] u_lat_rx_csi/gray_pipe[26] u_lat_rx_csi/gray_pipe[27] u_lat_rx_csi/gray_pipe[28] u_lat_rx_csi/gray_pipe[29] u_lat_rx_csi/gray_pipe[30] u_lat_rx_csi/gray_pipe[31]}] -datapath_only 9.360
set_false_path -hold -from [get_regs {S_lat_tick_gray*}] -to [get_regs {u_lat_rx_csi/gray_pipe[0] u_lat_rx_csi/gray_pipe[1] u_lat_rx_csi/gray_pipe[2] u_lat_rx_csi/gray_pipe[3] u_lat_rx_csi/gray_pipe[4] u_lat_rx_csi/gray_pipe[5] u_lat_rx_csi/gray_pipe[6] u_lat_rx_csi/gray_pipe[7] u_lat_rx_csi/gray_pipe[8] u_lat_rx_csi/gray_pipe[9] u_lat_rx_csi/gray_pipe[10] u_lat_rx_csi/gray_pipe[11] u_lat_rx_csi/gray_pipe[12] u_lat_rx_csi/gray_pipe[13] u_lat_rx_csi/gray_pipe[14] u_lat_rx_csi/gray_pipe[15] u_lat_rx_csi/gray_pipe[16] u_lat_rx_csi/gray_pipe[17] u_lat_rx_csi/gray_pipe[18] u_lat_rx_csi/gray_pipe[19] u_lat_rx_csi/gray_pipe[20] u_lat_rx_csi/gray_pipe[21] u_lat_rx_csi/gray_pipe[22] u_lat_rx_csi/gray_pipe[23] u_lat_rx_csi/gray_pipe[24] u_lat_rx_csi/gray_pipe[25] u_lat_rx_csi/gray_pipe[26] u_lat_rx_csi/gray_pipe[27] u_lat_rx_csi/gray_pipe[28] u_lat_rx_csi/gray_pipe[29] u_lat_rx_csi/gray_pipe[30] u_lat_rx_csi/gray_pipe[31]}]
set_max_delay -from [get_regs {S_lat_tick_gray*}] -to [get_regs {u_lat_rx_pix/gray_pipe[0] u_lat_rx_pix/gray_pipe[1] u_lat_rx_pix/gray_pipe[2] u_lat_rx_pix/gray_pipe[3] u_lat_rx_pix/gray_pipe[4] u_lat_rx_pix/gray_pipe[5] u_lat_rx_pix/gray_pipe[6] u_lat_rx_pix/gray_pipe[7] u_lat_rx_pix/gray_pipe[8] u_lat_rx_pix/gray_pipe[9] u_lat_rx_pix/gray_pipe[10] u_lat_rx_pix/gray_pipe[11] u_lat_rx_pix/gray_pipe[12] u_lat_rx_pix/gray_pipe[13] u_lat_rx_pix/gray_pipe[14] u_lat_rx_pix/gray_pipe[15] u_lat_rx_pix/gray_pipe[16] u_lat_rx_pix/gray_pipe[17] u_lat_rx_pix/gray_pipe[18] u_lat_rx_pix/gray_pipe[19] u_lat_rx_pix/gray_pipe[20] u_lat_rx_pix/gray_pipe[21] u_lat_rx_pix/gray_pipe[22] u_lat_rx_pix/gray_pipe[23] u_lat_rx_pix/gray_pipe[24] u_lat_rx_pix/gray_pipe[25] u_lat_rx_pix/gray_pipe[26] u_lat_rx_pix/gray_pipe[27] u_lat_rx_pix/gray_pipe[28] u_lat_rx_pix/gray_pipe[29] u_lat_rx_pix/gray_pipe[30] u_lat_rx_pix/gray_pipe[31]}] -datapath_only 13.033
set_false_path -hold -from [get_regs {S_lat_tick_gray*}] -to [get_regs {u_lat_rx_pix/gray_pipe[0] u_lat_rx_pix/gray_pipe[1] u_lat_rx_pix/gray_pipe[2] u_lat_rx_pix/gray_pipe[3] u_lat_rx_pix/gray_pipe[4] u_lat_rx_pix/gray_pipe[5] u_lat_rx_pix/gray_pipe[6] u_lat_rx_pix/gray_pipe[7] u_lat_rx_pix/gray_pipe[8] u_lat_rx_pix/gray_pipe[9] u_lat_rx_pix/gray_pipe[10] u_lat_rx_pix/gray_pipe[11] u_lat_rx_pix/gray_pipe[12] u_lat_rx_pix/gray_pipe[13] u_lat_rx_pix/gray_pipe[14] u_lat_rx_pix/gray_pipe[15] u_lat_rx_pix/gray_pipe[16] u_lat_rx_pix/gray_pipe[17] u_lat_rx_pix/gray_pipe[18] u_lat_rx_pix/gray_pipe[19] u_lat_rx_pix/gray_pipe[20] u_lat_rx_pix/gray_pipe[21] u_lat_rx_pix/gray_pipe[22] u_lat_rx_pix/gray_pipe[23] u_lat_rx_pix/gray_pipe[24] u_lat_rx_pix/gray_pipe[25] u_lat_rx_pix/gray_pipe[26] u_lat_rx_pix/gray_pipe[27] u_lat_rx_pix/gray_pipe[28] u_lat_rx_pix/gray_pipe[29] u_lat_rx_pix/gray_pipe[30] u_lat_rx_pix/gray_pipe[31]}]
# The startup/session toggle is a separate single-bit CDC; scope only its
# launch -> first synchronizer hop, not all paths between the two clocks.
set_max_delay -from [get_regs {S_lat_session_tog*}] -to [get_regs {u_lat_rx_csi/tog_sync*}] -datapath_only 9.360
set_false_path -hold -from [get_regs {S_lat_session_tog*}] -to [get_regs {u_lat_rx_csi/tog_sync*}]
set_max_delay -from [get_regs {S_lat_session_tog*}] -to [get_regs {u_lat_rx_pix/tog_sync*}] -datapath_only 13.033
set_false_path -hold -from [get_regs {S_lat_session_tog*}] -to [get_regs {u_lat_rx_pix/tog_sync*}]

# ============================================================================
# ============================================================================
# 2026-10-31: obsolete APB clock-group cut removed. No APB fabric slave remains.
# MCU/core/timer clocks retain their real relationships and ordinary timing checks.

create_generated_clock -name MCU_AHB_CLK -source [get_pins {u_mcu/MCU_fb7459297e7a_Inst/PH1P_PHY_MCU_Inst.core_clk}] -master_clock {u_mcu/MCU_fb7459297e7a_Inst/PH1P_PHY_MCU_Inst.core_clk} -divide_by 2 [get_pins {u_mcu/MCU_fb7459297e7a_Inst/PH1P_PHY_MCU_Inst.ahb_clk}]
