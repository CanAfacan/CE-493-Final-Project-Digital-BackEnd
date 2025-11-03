module mat3x3_q12 #(
  parameter W=24,
  parameter FRAC=12
)(
  input  signed [W-1:0] x_in, y_in, z_in,
  input  signed [W-1:0] m00,m01,m02,m10,m11,m12,m20,m21,m22,
  output signed [W-1:0] x_out, y_out, z_out
);
  function [W-1:0] mult_q;
    input signed [W-1:0] a,b;
    reg signed [2*W-1:0] p;
    begin p = a*b + (1<<(FRAC-1)); mult_q = p >>> FRAC; end
  endfunction

  wire signed [W-1:0] x0 = mult_q(m00, x_in);
  wire signed [W-1:0] x1 = mult_q(m01, y_in);
  wire signed [W-1:0] x2 = mult_q(m02, z_in);
  wire signed [W-1:0] y0 = mult_q(m10, x_in);
  wire signed [W-1:0] y1 = mult_q(m11, y_in);
  wire signed [W-1:0] y2 = mult_q(m12, z_in);
  wire signed [W-1:0] z0 = mult_q(m20, x_in);
  wire signed [W-1:0] z1 = mult_q(m21, y_in);
  wire signed [W-1:0] z2 = mult_q(m22, z_in);

  wire signed [W-1:0] t01, sx, t23, sy, t45, sz;
  sat_add #(W) ax0 (.a(x0), .b(x1), .y(t01));
  sat_add #(W) ax1 (.a(t01), .b(x2), .y(sx));
  sat_add #(W) ay0 (.a(y0), .b(y1), .y(t23));
  sat_add #(W) ay1 (.a(t23), .b(y2), .y(sy));
  sat_add #(W) az0 (.a(z0), .b(z1), .y(t45));
  sat_add #(W) az1 (.a(t45), .b(z2), .y(sz));

  assign x_out = sx; assign y_out = sy; assign z_out = sz;
endmodule
