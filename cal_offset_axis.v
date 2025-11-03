// average 2^LOG2_NSAMPLES decimated samples
module cal_offset_axis #(
  parameter W=24,
  parameter LOG2_NSAMPLES=8
)(
  input                      clk,
  input                      rst_n,
  input                      start,       // init pulse to begin
  input  signed [W-1:0]      din,
  input                      din_valid,
  output reg                 busy,
  output reg                 done,
  output reg signed [W-1:0]  offset_out
);
  localparam [1:0] IDLE=2'd0, RUN=2'd1, DONE=2'd2;
  reg [1:0] st;
  reg [LOG2_NSAMPLES:0] cnt;
  reg signed [W+LOG2_NSAMPLES:0] acc;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      st<=IDLE; busy<=1'b0; done<=1'b0; cnt<=0; acc<=0; offset_out<=0;
    end else begin
      done<=1'b0;
      case(st)
        IDLE: begin
          busy<=1'b0; cnt<=0; acc<=0;
          if(start) begin st<=RUN; busy<=1'b1; end
        end
        RUN: begin
          if(din_valid) begin
            acc <= acc + din;
            cnt <= cnt + 1'b1;
            if(cnt == ((1<<LOG2_NSAMPLES)-1)) begin
              offset_out <= acc >>> LOG2_NSAMPLES;
              st<=DONE;
            end
          end
        end
        DONE: begin busy<=1'b0; done<=1'b1; st<=IDLE; end
      endcase
    end
  end
endmodule
