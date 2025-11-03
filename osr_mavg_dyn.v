// average by 2^log2_osr with runtime control
module osr_mavg_dyn #(
  parameter W = 24,
  parameter LOG2_OSR_MAX = 8  // OSR up to 256
)(
  input                        clk,
  input                        rst_n,
  input  signed [W-1:0]        din,
  input                        din_valid,
  input        [7:0]           log2_osr,   // only up to LOG2_OSR_MAX
  output reg signed [W-1:0]    dout,
  output reg                   dout_valid
);
  reg [LOG2_OSR_MAX-1:0] cnt;
  reg signed [W+LOG2_OSR_MAX:0] acc;

  wire [LOG2_OSR_MAX-1:0] osr_mask = (1'b1 << log2_osr[LOG2_OSR_MAX-1:0]) - 1'b1;
  wire [LOG2_OSR_MAX-1:0] osr_val  = (1'b1 << log2_osr[LOG2_OSR_MAX-1:0]);

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      cnt <= 0; acc <= 0; dout <= 0; dout_valid <= 1'b0;
    end else begin
      dout_valid <= 1'b0;
      if (din_valid) begin
        acc <= acc + din;
        cnt <= cnt + 1'b1;
        if (cnt == osr_val - 1) begin
          // arithmetic shift by log2_osr
          dout <= acc >>> log2_osr[LOG2_OSR_MAX-1:0];
          dout_valid <= 1'b1;
          cnt <= 0;
          acc <= 0;
        end
      end
    end
  end
endmodule
