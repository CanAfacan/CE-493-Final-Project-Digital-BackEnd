module adc_axis_router #(
  parameter ADC_BITS = 10,
  parameter W = 24,
  parameter FRAC = 12
)(
  input                       clk,
  input                       rst_n,
  input      [ADC_BITS-1:0]   adc_code,   // offset-binary
  input                       adc_valid,
  input      [1:0]            mux_sel,    // 0 X, 1 Y, 2 Z
  output reg signed [W-1:0]   x_sample, y_sample, z_sample,
  output reg                  x_valid,  y_valid,  z_valid
);
  wire [ADC_BITS:0] u = {1'b0, adc_code};
  wire signed [ADC_BITS:0] s = $signed(u) - (1 << (ADC_BITS-1));
  wire signed [W-1:0] q = {{(W-(ADC_BITS+1+FRAC)){s[ADC_BITS]}}, s, {FRAC{1'b0}}};

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      x_sample <= 0; y_sample <= 0; z_sample <= 0;
      x_valid <= 1'b0; y_valid <= 1'b0; z_valid <= 1'b0;
    end else begin
      x_valid <= 1'b0; y_valid <= 1'b0; z_valid <= 1'b0;
      if (adc_valid) begin
        case (mux_sel)
          2'd0: begin x_sample <= q; x_valid <= 1'b1; end
          2'd1: begin y_sample <= q; y_valid <= 1'b1; end
          2'd2: begin z_sample <= q; z_valid <= 1'b1; end
          default: ;
        endcase
      end
    end
  end
endmodule
