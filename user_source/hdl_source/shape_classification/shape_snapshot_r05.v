// Candidate production observer: real 720p dual captures plus paired backend.
// No RGB/control output, CDC, PLL or pin changes. I_frame_tick must be a safe
// blanking edge BEFORE active pixels, not an active first-pixel bypass.
module shape_snapshot_r05 #(parameter integer PAPER_REQUIRED=1,MULTI_TARGETS=1)(
 input wire I_clk,I_rst_n,I_enable,I_cancel,I_frame_tick,
 input wire I_de,I_sof,I_eol,I_pixel_good,
 input wire [11:0] I_x,I_y,input wire [23:0] I_rgb,
 input wire [7:0] I_threshold,I_paper_threshold,
 input wire [1:0] I_profile,input wire [31:0] I_source_frame_id,
 input wire [4:0] I_area_min,I_area_max,
 output wire [248:0] O_multi_packets,
 output wire O_valid,output wire [2:0] O_status,output wire [1:0] O_class,
 output wire [11:0] O_xmin,O_ymin,O_xmax,O_ymax,O_center_x,O_center_y,
 output wire [31:0] O_seq,O_source,O_epoch,
 output wire O_fault,O_busy,O_capture_drop,O_result_valid,O_result_paper,
 output wire [2:0] O_result_status,output wire [1:0] O_result_class,
 output wire [31:0] O_result_source,
 output wire [20:0] O_job_cycles,
 output wire [19:0] O_diag_code,
 output wire O_diag_pvalid,O_diag_kvalid,
 output wire [33:0] O_diag_pbox,O_diag_kbox,
 output wire [31:0] O_diag_source,O_diag_epoch
);
 wire capture_enable,iw,ib,ie,ix,pw,pb,pe,px;
 wire [15:0] ia;
 wire [13:0] pa;
 shape_capture ink(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(capture_enable),.I_cancel(I_cancel),
  .I_de(I_de),.I_sof(I_sof),.I_eol(I_eol),.I_pixel_good(I_pixel_good),.I_x(I_x),.I_y(I_y),
  .I_rgb(I_rgb),.I_threshold(I_threshold),.O_write(iw),.O_bit(ib),.O_sof(),.O_eof(ie),
  .O_addr(ia),.O_bad_event(ix),.O_active());
 generate if(PAPER_REQUIRED!=0)begin: paper_capture
 shape_paper_capture paper(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(capture_enable),.I_cancel(I_cancel),
  .I_de(I_de),.I_sof(I_sof),.I_eol(I_eol),.I_pixel_good(I_pixel_good),.I_x(I_x),.I_y(I_y),
  .I_rgb(I_rgb),.I_threshold(I_paper_threshold),.O_write(pw),.O_bit(pb),.O_sof(),.O_eof(pe),
  .O_addr(pa),.O_bad_event(px),.O_active());
 end else begin: no_paper_capture
  assign pw=1'b0;assign pb=1'b0;assign pe=1'b0;assign px=1'b0;assign pa=14'd0;
 end endgenerate
 shape_pair_backend_r05 #(.PAPER_REQUIRED(PAPER_REQUIRED),.MULTI_TARGETS(MULTI_TARGETS)) backend(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),.I_cancel(I_cancel),
  .I_sof(I_de&&I_sof),.I_frame_tick(I_frame_tick),.I_source(I_source_frame_id),
  .I_gray(I_threshold),.I_paper_threshold(I_paper_threshold),.I_profile(I_profile),
  .I_area_min(I_area_min),.I_area_max(I_area_max),
  .I_ink_write(iw),.I_ink_bit(ib),.I_ink_eof(ie),.I_ink_bad(ix),.I_ink_addr(ia),
  .I_paper_write(pw),.I_paper_bit(pb),.I_paper_eof(pe),.I_paper_bad(px),.I_paper_addr(pa),
  .O_capture_enable(capture_enable),.O_fault(O_fault),.O_busy(O_busy),.O_capture_drop(O_capture_drop),
  .O_result_valid(O_result_valid),.O_result_paper(O_result_paper),.O_result_status(O_result_status),
  .O_result_target(),.O_result_class(O_result_class),.O_result_profile(),.O_result_reason(),.O_result_retry(),
  .O_result_l(),.O_result_r(),.O_result_t(),.O_result_b(),.O_result_area(),.O_result_seq(),
  .O_result_source(O_result_source),.O_result_epoch(),
  .O_multi_packets(O_multi_packets),.O_valid(O_valid),.O_status(O_status),.O_class(O_class),.O_xmin(O_xmin),.O_ymin(O_ymin),.O_xmax(O_xmax),.O_ymax(O_ymax),
  .O_center_x(O_center_x),.O_center_y(O_center_y),.O_seq(O_seq),.O_source(O_source),.O_epoch(O_epoch),
  .O_context_epoch(),.O_job_cycles(O_job_cycles),
  .O_diag_code(O_diag_code),.O_diag_pvalid(O_diag_pvalid),.O_diag_kvalid(O_diag_kvalid),
  .O_diag_pbox(O_diag_pbox),.O_diag_kbox(O_diag_kbox),.O_diag_source(O_diag_source),.O_diag_epoch(O_diag_epoch));
endmodule
