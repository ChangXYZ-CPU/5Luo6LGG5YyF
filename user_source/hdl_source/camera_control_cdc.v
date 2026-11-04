`timescale 1ns/1ps
// Latest-value mailbox for the joint AE/AG control packet. Source payload is
// held until a round-trip ACK; rapid changes coalesce and the latest pair is
// sent after ACK. No minimum event spacing or coincident clocks is assumed.
// Both domains share I_rst_n (the production PLL lock reset).
module camera_control_cdc(
    input wire I_src_clk, I_dst_clk, I_rst_n,
    input wire [15:0] I_ae, I_ag,
    input wire I_refresh,
    output reg [15:0] O_ae, O_ag,
    output reg O_req
);
    reg [31:0] payload;
    reg req_toggle, ack_toggle;
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_sync;
    reg seeded, refresh_pending;
    wire idle = (ack_sync[2] == req_toggle);
    always @(posedge I_src_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            payload <= {16'd1920,16'd160};
            req_toggle <= 1'b0; ack_sync <= 3'd0;
            seeded <= 1'b0; refresh_pending <= 1'b0;
        end else begin
            ack_sync <= {ack_sync[1:0],ack_toggle};
            if (I_refresh) refresh_pending <= 1'b1;
            if (idle && (!seeded || refresh_pending || I_refresh || payload != {I_ae,I_ag})) begin
                payload <= {I_ae,I_ag};
                req_toggle <= ~req_toggle;
                seeded <= 1'b1; refresh_pending <= 1'b0;
            end
        end
    end
    always @(posedge I_dst_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            req_sync <= 3'd0; ack_toggle <= 1'b0;
            O_ae <= 16'd1920; O_ag <= 16'd160; O_req <= 1'b0;
        end else begin
            req_sync <= {req_sync[1:0],req_toggle};
            O_req <= 1'b0;
            if (req_sync[2] != ack_toggle) begin
                {O_ae,O_ag} <= payload;
                O_req <= 1'b1;
                ack_toggle <= req_sync[2];
            end
        end
    end
endmodule
