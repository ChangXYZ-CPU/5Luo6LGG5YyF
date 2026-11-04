// key_debounce.v - active-low button debounce, 1-cycle pulse on RELEASE.
// Preserve the established release-triggered project interaction.
module key_debounce #(
    parameter CLK_FREQ         = 75_000_000,
    parameter DEBOUNCE_TIME_MS = 20
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_button,    // active-low, external pullup
    output reg  O_pulse      // 1-cycle high pulse on press
);

    localparam integer COUNT_MAX = CLK_FREQ / 1000 * DEBOUNCE_TIME_MS;  // 1,500,000 (21 bits)

    reg [$clog2(COUNT_MAX)-1:0] count;
    reg btn_sync1, btn_sync2;
    reg btn_state;   // settled level: 1 = released, 0 = pressed

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            btn_sync1 <= 1'b1;
            btn_sync2 <= 1'b1;
            btn_state <= 1'b1;
            count     <= 0;
            O_pulse   <= 1'b0;
        end else begin
            btn_sync1 <= I_button;
            btn_sync2 <= btn_sync1;
            O_pulse   <= 1'b0;
            if(btn_sync2 != btn_state) begin
                if(count == COUNT_MAX - 1) begin
                    count     <= 0;
                    btn_state <= btn_sync2;
                    if(btn_state == 1'b0)
                        O_pulse <= 1'b1;   // old state was pressed; now settled to released
                end else begin
                    count <= count + 1'b1;
                end
            end else begin
                count <= 0;
            end
        end
    end

endmodule
