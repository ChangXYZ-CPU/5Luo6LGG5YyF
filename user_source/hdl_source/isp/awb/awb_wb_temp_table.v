// White-balance colour-temperature table for SC520CS.
// K -> R/B channel gain, Q0.16 (0x10000 = 1.0), G channel is the reference (1.0).
// Range 2000..8000 K, step 200 K -> 31 entries. Generated from the standard
// blackbody/daylight approximation (Tanner Helland fit), gains clamped to 0.25x..4.0x.
// NOTE: the conversion depends on the sensor spectral response, so these values are a
// STARTING APPROXIMATION TO BE CALIBRATED on the real sensor with a grey card.
module awb_wb_temp_table(
    input  wire [12:0] I_temp,      // 2000..8000 (K)
    output reg  [19:0] O_gain_r,    // Q0.16, 0x10000 = 1.0
    output reg  [19:0] O_gain_b
);
    // The v4 AWB that consumes this table clamps its gains to 0.5x..2.0x, so only the
    // 3000..8000 K span can be applied in full; anything outside is clamped here.
    wire [12:0] t = (I_temp < 13'd3000) ? 13'd3000 : ((I_temp > 13'd8000) ? 13'd8000 : I_temp);

    always @(*) begin
        case (t)
            13'd2000 : begin O_gain_r=20'd35176  ; O_gain_b=20'd262144 ; end
            13'd2200 : begin O_gain_r=20'd37612  ; O_gain_b=20'd244929 ; end
            13'd2400 : begin O_gain_r=20'd39837  ; O_gain_b=20'd167875 ; end
            13'd2600 : begin O_gain_r=20'd41883  ; O_gain_b=20'd135178 ; end
            13'd2800 : begin O_gain_r=20'd43777  ; O_gain_b=20'd117110 ; end
            13'd3000 : begin O_gain_r=20'd45541  ; O_gain_b=20'd105652 ; end
            13'd3200 : begin O_gain_r=20'd47191  ; O_gain_b=20'd97740  ; end
            13'd3400 : begin O_gain_r=20'd48741  ; O_gain_b=20'd91949  ; end
            13'd3600 : begin O_gain_r=20'd50202  ; O_gain_b=20'd87526  ; end
            13'd3800 : begin O_gain_r=20'd51584  ; O_gain_b=20'd84038  ; end
            13'd4000 : begin O_gain_r=20'd52896  ; O_gain_b=20'd81215  ; end
            13'd4200 : begin O_gain_r=20'd54143  ; O_gain_b=20'd78884  ; end
            13'd4400 : begin O_gain_r=20'd55332  ; O_gain_b=20'd76926  ; end
            13'd4600 : begin O_gain_r=20'd56469  ; O_gain_b=20'd75257  ; end
            13'd4800 : begin O_gain_r=20'd57557  ; O_gain_b=20'd73818  ; end
            13'd5000 : begin O_gain_r=20'd58600  ; O_gain_b=20'd72563  ; end
            13'd5200 : begin O_gain_r=20'd59603  ; O_gain_b=20'd71460  ; end
            13'd5400 : begin O_gain_r=20'd60568  ; O_gain_b=20'd70481  ; end
            13'd5600 : begin O_gain_r=20'd61497  ; O_gain_b=20'd69607  ; end
            13'd5800 : begin O_gain_r=20'd62394  ; O_gain_b=20'd68822  ; end
            13'd6000 : begin O_gain_r=20'd63261  ; O_gain_b=20'd68112  ; end
            13'd6200 : begin O_gain_r=20'd64099  ; O_gain_b=20'd67467  ; end
            13'd6400 : begin O_gain_r=20'd64911  ; O_gain_b=20'd66878  ; end
            13'd6600 : begin O_gain_r=20'd65536  ; O_gain_b=20'd65536  ; end
            13'd6800 : begin O_gain_r=20'd64571  ; O_gain_b=20'd63288  ; end
            13'd7000 : begin O_gain_r=20'd65408  ; O_gain_b=20'd62230  ; end
            13'd7200 : begin O_gain_r=20'd66099  ; O_gain_b=20'd61379  ; end
            13'd7400 : begin O_gain_r=20'd66690  ; O_gain_b=20'd60669  ; end
            13'd7600 : begin O_gain_r=20'd67206  ; O_gain_b=20'd60060  ; end
            13'd7800 : begin O_gain_r=20'd67664  ; O_gain_b=20'd59528  ; end
            13'd8000 : begin O_gain_r=20'd68076  ; O_gain_b=20'd59057  ; end
            default: begin O_gain_r=20'd65536; O_gain_b=20'd65536; end
        endcase
    end
endmodule
