// Actual MC-to-pAXI wiring, factored out so the live integration is testable.
// pAXI WSTRB is a HIGH-active MASK, unlike AXI byte enables (IPUG035 table).
module ddr_paxi_adapter(
 input calibrated,input [24:0] cmd_addr,input [2:0] cmd,input cmd_valid,
 input [127:0] write_data,input [15:0] write_mask,input write_valid,write_last,
 output cmd_ready,data_ready,
 output [24:0] awaddr,araddr,output awvalid,arvalid,
 output [127:0] wdata,output [15:0] wstrb,output wvalid,wlast,
 input awready,arready,wready
);
 assign cmd_ready=calibrated&&(cmd[0]?arready:awready);
 assign data_ready=calibrated&&wready;
 assign awaddr=cmd_addr;
 assign araddr=cmd_addr;
 assign awvalid=calibrated&&cmd_valid&&!cmd[0];
 assign arvalid=calibrated&&cmd_valid&&cmd[0];
 assign wdata=write_data;
 assign wstrb=write_mask;
 assign wvalid=calibrated&&write_valid;
 assign wlast=write_last;
endmodule
