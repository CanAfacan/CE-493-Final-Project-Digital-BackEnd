// sat_add.v - saturating adder for equal widths
module sat_add #(parameter W=24) (
  input  signed [W-1:0] a,
  input  signed [W-1:0] b,
  output signed [W-1:0] y
);
  wire signed [W:0] s = a + b;
  assign y = (s[W] ^ s[W-1]) ? (s[W] ? {1'b1,{(W-1){1'b0}}} : {1'b0,{(W-1){1'b1}}}) : s[W-1:0];
endmodule

// sat_clip.v - signed clip from WIN to WOUT with saturation
module sat_clip #(parameter WIN=48, parameter WOUT=24) (
  input  signed [WIN-1:0] din,
  output signed [WOUT-1:0] dout
);
  localparam signed [WIN-1:0] MAXP = {{(WIN-WOUT){1'b0}}, 1'b0, {(WOUT-1){1'b1}}};
  localparam signed [WIN-1:0] MAXN = {{(WIN-WOUT){1'b1}}, 1'b1, {(WOUT-1){1'b0}}};
  assign dout = (din > MAXP) ? MAXP[WOUT-1:0] :
                (din < MAXN) ? MAXN[WOUT-1:0] :
                din[WOUT-1:0];
endmodule
