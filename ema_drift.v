// baseline tracker y = y + (x - y) >> alpha_shift when |x - y| <= freeze_th
module ema_drift #(
  parameter W=24
)(
  input                      clk,
  input                      rst_n,
  input  signed [W-1:0]      din,
  input                      din_valid,
  input  [7:0]               alpha_shift,
  input  signed [W-1:0]      freeze_th,
  output reg signed [W-1:0]  drift_out
);
  wire signed [W-1:0] err = din - drift_out;
  wire [W-1:0] abs_err = err[W-1] ? (~err + 1'b1) : err;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      drift_out <= 0;
    end else if(din_valid) begin
      if(abs_err <= freeze_th) begin
        drift_out <= drift_out + (err >>> alpha_shift);
      end
    end
  end
endmodule
