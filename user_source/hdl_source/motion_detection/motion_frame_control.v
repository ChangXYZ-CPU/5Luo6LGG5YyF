// M02 control-plane skeleton, NOT a detector or a history RAM substitute.
// M03 owns the actual reference RAM and may commit it only at O_commit.
// Outputs are sampled at the same edge as reader RGB (no image latency).
module motion_frame_control #(
 parameter integer WIDTH=1280,HEIGHT=720,MAX_REPEAT_FRAMES=120
)(
 input wire I_clk,I_rst_n,I_enable,I_boundary,I_de,I_user,I_last,I_pixel_good,
 input wire I_source_valid,
 input wire [31:0] I_source_id,I_config,
 input wire [11:0] I_x,I_y,
 output wire O_start,O_accept,O_end,O_commit,O_abort,O_reference_clear,
 output wire O_new_source,
 output wire [31:0] O_frame_source_id,O_config_epoch
);
 wire ga,gs,ge,gb,gfb,gcb;
 localparam CONTROL_PARAM_OK=(MAX_REPEAT_FRAMES>=1);
 red_frame_guard #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_geometry(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable && !I_boundary),
  .I_de(I_de),.I_user(I_user),.I_last(I_last),.I_pixel_good(I_pixel_good),
  .I_x(I_x),.I_y(I_y),.O_accept(ga),.O_frame_start(gs),.O_frame_end(ge),
  .O_bad_event(gb),.O_frame_bad(gfb),.O_config_bad(gcb)
 );
 reg live,bad,capture,last_valid,config_seen;
 reg [31:0] frame_id,last_id,config_value,epoch,repeats;
 wire sof=I_enable && I_de && I_user && !I_boundary;
 wire config_changed=config_seen && I_config!=config_value;
 wire expired=sof && last_valid && I_source_id==last_id && repeats>=MAX_REPEAT_FRAMES;
 wire meta_bad=I_de && (!I_source_valid || (!sof && (!live || I_source_id!=frame_id)));
 wire fault=!CONTROL_PARAM_OK || gb || gcb || meta_bad || config_changed || (I_boundary && live);
 wire start_good=CONTROL_PARAM_OK && sof && ga && I_source_valid && !config_changed;
 wire new_source=start_good && (!last_valid || I_source_id!=last_id);
 assign O_start=sof;
 assign O_new_source=new_source;
 assign O_accept=CONTROL_PARAM_OK && I_rst_n && I_enable && !I_boundary && ga && I_source_valid && !config_changed &&
                 (sof ? new_source : (live && !bad && capture && I_source_id==frame_id));
 assign O_end=O_accept && ge;
 assign O_commit=O_end;
 assign O_abort=I_rst_n && I_enable && fault;
 assign O_reference_clear=!I_rst_n || !I_enable || O_abort || expired;
 assign O_frame_source_id=sof ? I_source_id : frame_id;
 assign O_config_epoch=epoch;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   live<=0;bad<=0;capture<=0;last_valid<=0;config_seen<=0;
   frame_id<=0;last_id<=0;config_value<=0;epoch<=0;repeats<=0;
  end else if(!I_enable) begin
   live<=0;bad<=0;capture<=0;last_valid<=0;config_seen<=0;repeats<=0;
  end else begin
   config_seen<=1;config_value<=I_config;
   if(config_changed) begin epoch<=epoch+32'd1;last_valid<=0;repeats<=0;end
   if(I_boundary) begin
    live<=0;bad<=0;capture<=0;
    if(live) begin last_valid<=0;repeats<=0;end
   end else if(sof) begin
    frame_id<=I_source_id;live<=!ge;bad<=!start_good;capture<=new_source;
    if(start_good) begin
     if(!new_source && repeats<MAX_REPEAT_FRAMES) repeats<=repeats+32'd1;
    end else begin last_valid<=0;repeats<=0;end
   end else if(fault) begin
    bad<=1;capture<=0;last_valid<=0;repeats<=0;
   end
   if(ga && ge) live<=0;
   if(O_commit) begin last_id<=I_source_id;last_valid<=1;repeats<=0;end
  end
 end
endmodule
