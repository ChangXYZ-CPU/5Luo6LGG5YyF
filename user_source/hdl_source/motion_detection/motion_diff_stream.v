// M03 front-end. Takes reader RGB and aligned control BEFORE display effects.
// Outputs form a provisional low-resolution stream; M04 must wait for O_done
// and cancel on O_reference_clear. No original video data/control is retimed.
module motion_diff_stream #(
 parameter integer WIDTH=1280,HEIGHT=720,DIFF_THRESHOLD=16,MAX_REPEAT_FRAMES=120
)(
 input wire I_clk,I_rst_n,I_enable,I_boundary,I_de,I_user,I_last,I_pixel_good,
 input wire I_source_valid,
 input wire [31:0] I_source_id,I_config,
 input wire [23:0] I_rgb,
 input wire [11:0] I_x,I_y,
 output wire O_valid,O_first,O_final,O_compare_valid,O_mask,O_done,
 output wire O_reference_valid,O_reference_clear,O_abort,O_busy,
 output wire [7:0] O_gray,O_reference_gray,O_diff,O_x,
 output wire [6:0] O_y,
 output wire [31:0] O_source_id,O_reference_id,O_epoch
);
 localparam PARAM_OK=WIDTH>=8 && WIDTH<=1280 && HEIGHT>=8 && HEIGHT<=720 &&
                      WIDTH%8==0 && HEIGHT%8==0 && DIFF_THRESHOLD>=0 && DIFF_THRESHOLD<=255 && MAX_REPEAT_FRAMES>=1;
 reg pending_frame;
 wire overlap=I_enable && I_de && I_user && pending_frame && !O_done;
 wire cs,ca,ce,cc,cabort,clear,cnew;
 wire [31:0] source,epoch;
 motion_frame_control #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MAX_REPEAT_FRAMES(MAX_REPEAT_FRAMES)) u_control(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable && PARAM_OK && !overlap),
  .I_boundary(I_boundary),.I_de(I_de),.I_user(I_user),.I_last(I_last),
  .I_pixel_good(I_pixel_good),.I_source_valid(I_source_valid),.I_source_id(I_source_id),
  .I_config(I_config),.I_x(I_x),.I_y(I_y),
  .O_start(cs),.O_accept(ca),.O_end(ce),.O_commit(cc),.O_abort(cabort),
  .O_reference_clear(clear),.O_new_source(cnew),.O_frame_source_id(source),.O_config_epoch(epoch)
 );
 assign O_reference_clear=clear || overlap || (I_boundary && pending_frame && !O_done);
 assign O_abort=cabort || overlap || (I_boundary && pending_frame && !O_done);
 assign O_busy=pending_frame && !O_done;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)pending_frame<=0;
  else if(O_reference_clear)pending_frame<=0;
  else if(cs && ca)pending_frame<=1;
  else if(O_done)pending_frame<=0;
 end
 wire bv,bfirst,bfinal;
 wire [7:0] bg,bx;wire [6:0] by;
 motion_gray_blocks #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_blocks(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(O_reference_clear),
  .I_accept(ca),.I_first(cs),.I_final(cc),.I_rgb(I_rgb),.I_x(I_x),.I_y(I_y),
  .O_valid(bv),.O_first(bfirst),.O_final(bfinal),.O_gray(bg),.O_x(bx),.O_y(by)
 );
 motion_history_diff #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DIFF_THRESHOLD(DIFF_THRESHOLD)) u_history(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(O_reference_clear),
  .I_frame_start(cs && ca),.I_source_id(source),.I_epoch(epoch),
  .I_valid(bv),.I_first(bfirst),.I_final(bfinal),.I_gray(bg),.I_x(bx),.I_y(by),
  .O_valid(O_valid),.O_first(O_first),.O_final(O_final),.O_compare_valid(O_compare_valid),
  .O_mask(O_mask),.O_done(O_done),.O_reference_valid(O_reference_valid),
  .O_gray(O_gray),.O_reference_gray(O_reference_gray),.O_diff(O_diff),.O_x(O_x),.O_y(O_y),
  .O_source_id(O_source_id),.O_reference_id(O_reference_id),.O_epoch(O_epoch)
 );
endmodule
