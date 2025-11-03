// Q12 gain after offset removal with saturation
module gain_offset_apply #(
  parameter W=24,
  parameter FRAC=12
)(
  input  signed [W-1:0] din,
  input  signed [W-1:0] offset,
  input  signed [W-1:0] gain,
  output signed [W-1:0] dout
);
  wire signed [W-1:0] din_z = din - offset;
  wire signed [2*W-1:0] p = din_z * gain;
  wire signed [2*W-1:0] pr = p + (1<<(FRAC-1));
  wire signed [W-1:0] s = pr >>> FRAC;
  wire signed [W:0] ext = {s[W-1], s};
  assign dout = (ext[W]^ext[W-1]) ? (ext[W] ? {1'b1,{(W-1){1'b0}}} : {1'b0,{(W-1){1'b1}}}) : s;
endmodule
