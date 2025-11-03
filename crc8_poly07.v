// bitwise CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07)
// this file is not synthesizable as is
module crc8_poly07 (
  input        clk,
  input        rst_n,
  input        clr,       // clear CRC to 0
  input        en,        // consume one input bit on this cycle
  input        din,       // MSB-first bit
  output [7:0] crc
);
  reg [7:0] r;
  assign crc = r;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) r <= 8'h00;
    else if (clr) r <= 8'h00;
    else if (en) begin
      // MSB-first LFSR for poly 0x07
      // feedback is din XOR r[7]
      wire fb = din ^ r[7];
      r[7] <= r[6];
      r[6] <= r[5];
      r[5] <= r[4];
      r[4] <= r[3];
      r[3] <= r[2];
      r[2] <= r[1] ^ fb; // tap at x^2
      r[1] <= r[0] ^ fb; // tap at x^1
      r[0] <= fb;        // x^0
    end
  end
endmodule
