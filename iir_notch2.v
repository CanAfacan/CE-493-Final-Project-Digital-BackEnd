// standard two-pole notch section in Q12
module iir_notch2 #(
  parameter W=24,
  parameter FRAC=12
)(
  input                       clk,
  input                       rst_n,
  input  signed [W-1:0]       din,
  input                       din_valid,
  input  signed [W-1:0]       b1,   // -2*cos(w0)
  input  signed [W-1:0]       a1,   // -2*r*cos(w0)
  input  signed [W-1:0]       a2,   // r^2
  output reg signed [W-1:0]   dout,
  output reg                  dout_valid
);
  reg signed [W-1:0] x1, x2, y1, y2;

  function [W-1:0] mult_q;
    input signed [W-1:0] a, b;
    reg signed [2*W-1:0] p;
    begin p = a*b + (1<<(FRAC-1)); mult_q = p >>> FRAC; end
  endfunction

  wire signed [W-1:0] t1 = mult_q(b1, x1);
  wire signed [W-1:0] t2 = mult_q(a1, y1);
  wire signed [W-1:0] t3 = mult_q(a2, y2);
  wire signed [W:0] ypre = $signed(din) + $signed(t1) + $signed(x2) - $signed(t2) - $signed(t3);
  wire signed [W-1:0] ys = (ypre[W]^ypre[W-1]) ? (ypre[W] ? {1'b1,{(W-1){1'b0}}} : {1'b0,{(W-1){1'b1}}}) : ypre[W-1:0];

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin x1<=0; x2<=0; y1<=0; y2<=0; dout<=0; dout_valid<=1'b0; end
    else begin
      dout_valid <= 1'b0;
      if(din_valid) begin
        dout <= ys; dout_valid <= 1'b1;
        x2 <= x1; x1 <= din;
        y2 <= y1; y1 <= ys;
      end
    end
  end
endmodule
