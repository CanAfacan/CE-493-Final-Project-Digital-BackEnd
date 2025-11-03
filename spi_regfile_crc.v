// SPI mode-0 with CRC-8 on each 40-bit command and 8-bit CRC
module spi_regfile_crc #(
  parameter W=24,
  parameter FRAC=12
)(
  input  clk,
  input  rst_n,
  // SPI pins
  input  sclk,
  input  cs_n,
  input  mosi,
  output miso,

  // config outputs
  output reg        notch50_en,
  output reg        notch60_en,
  output reg        selftest_en,
  output reg [7:0]  osr_log2,
  output reg [7:0]  drift_alpha_shift,
  output reg signed [W-1:0] drift_freeze_th,
  output reg signed [W-1:0] offset_x, offset_y, offset_z,
  output reg signed [W-1:0] gain_x, gain_y, gain_z,
  output reg signed [W-1:0] b1_50, a1_50, a2_50,
  output reg signed [W-1:0] b1_60, a1_60, a2_60,
  output reg signed [W-1:0] m00, m01, m02, m10, m11, m12, m20, m21, m22,
  output reg        cal_start,

  // status and outputs
  input             cal_busy,
  input             cal_done,
  input  signed [W-1:0] out_x, out_y, out_z,
  input  [31:0]     out_mag2,
  input  [31:0]     version_id,
  input  [31:0]     drift_slope_x, drift_slope_y, drift_slope_z,

  output reg        crc_error  // high if last write CRC failed
);
  reg miso_r; assign miso = miso_r;

  // resync SCLK and CS to clk
  reg sclk_d, sclk_dd, cs_d;
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin sclk_d<=0; sclk_dd<=0; cs_d<=1; end
    else begin sclk_d<=sclk; sclk_dd<=sclk_d; cs_d<=cs_n; end
  end
  wire sclk_rise = (sclk_d & ~sclk_dd) & ~cs_d;
  wire sclk_fall = (~sclk_d & sclk_dd) & ~cs_d;

  // frame shift registers
  reg [39:0] sh_in;   // [rw|addr(7)|data(32)]
  reg [31:0] sh_out;
  reg [5:0]  bitcnt;
  wire       rw   = sh_in[39];
  wire [6:0] addr = sh_in[38:32];

  // CRC instances for RX and TX
  wire [7:0] crc_rx, crc_tx;
  reg        crc_clr, crc_en, crc_in_bit;
  crc8_poly07 UCRCRX (.clk(clk), .rst_n(rst_n), .clr(crc_clr), .en(crc_en), .din(crc_in_bit), .crc(crc_rx));
  crc8_poly07 UCRCTX (.clk(clk), .rst_n(rst_n), .clr(crc_clr), .en(crc_en), .din(crc_in_bit), .crc(crc_tx)); // same driving, used differently

  // simple state: 0 shifting command+data, then 8 CRC bits
  reg [7:0] crc_recvd;
  reg       in_crc_phase;

  // reset defaults
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      notch50_en<=1'b0; notch60_en<=1'b0; selftest_en<=1'b0;
      osr_log2<=8'd3; drift_alpha_shift<=8'd18; drift_freeze_th<=0;
      offset_x<=0; offset_y<=0; offset_z<=0;
      gain_x<=(1<<FRAC); gain_y<=(1<<FRAC); gain_z<=(1<<FRAC);
      {b1_50,a1_50,a2_50,b1_60,a1_60,a2_60} <= 0;
      {m00,m01,m02,m10,m11,m12,m20,m21,m22} <= 0;
      cal_start<=1'b0; crc_error<=1'b0;
      bitcnt<=0; sh_in<=0; sh_out<=32'h0; miso_r<=1'b0;
      crc_clr<=1'b1; crc_en<=1'b0; crc_in_bit<=1'b0; in_crc_phase<=1'b0; crc_recvd<=8'h00;
    end else begin
      cal_start <= 1'b0;

      if(cs_n) begin
        bitcnt <= 0; in_crc_phase<=1'b0; crc_clr<=1'b1; crc_en<=1'b0;
      end else begin
        // start CRC at beginning of frame
        if(bitcnt==0 && sclk_rise) begin crc_clr<=1'b0; end

        // receive bits
        if(sclk_rise) begin
          if(!in_crc_phase) begin
            sh_in <= {sh_in[38:0], mosi};
            bitcnt <= bitcnt + 1'b1;
            // feed RX CRC with incoming bit
            crc_en <= 1'b1; crc_in_bit <= mosi;
            if(bitcnt==7) begin
              // after rw+addr, prepare read data
              case (addr)
                7'h00: sh_out <= 32'h4D41474D;  // "MAGM"
                7'h01: sh_out <= version_id;
                7'h02: sh_out <= {28'h0, selftest_en, notch60_en, notch50_en, 1'b0};
                7'h03: sh_out <= {30'h0, cal_done, cal_busy};
                7'h04: sh_out <= {crc_error, 31'h0};
                7'h10: sh_out <= {{(32-W){offset_x[W-1]}}, offset_x};
                7'h11: sh_out <= {{(32-W){offset_y[W-1]}}, offset_y};
                7'h12: sh_out <= {{(32-W){offset_z[W-1]}}, offset_z};
                7'h14: sh_out <= {{(32-W){gain_x[W-1]}}, gain_x};
                7'h15: sh_out <= {{(32-W){gain_y[W-1]}}, gain_y};
                7'h16: sh_out <= {{(32-W){gain_z[W-1]}}, gain_z};
                7'h18: sh_out <= {{(32-W){drift_freeze_th[W-1]}}, drift_freeze_th};
                7'h19: sh_out <= {24'h0, drift_alpha_shift};
                7'h1A: sh_out <= {24'h0, osr_log2};
                7'h1B: sh_out <= {{(32-W){b1_50[W-1]}}, b1_50};
                7'h1C: sh_out <= {{(32-W){a1_50[W-1]}}, a1_50};
                7'h1D: sh_out <= {{(32-W){a2_50[W-1]}}, a2_50};
                7'h1E: sh_out <= {{(32-W){b1_60[W-1]}}, b1_60};
                7'h1F: sh_out <= {{(32-W){a1_60[W-1]}}, a1_60};
                7'h20: sh_out <= {{(32-W){a2_60[W-1]}}, a2_60};
                7'h30: sh_out <= {{(32-W){m00[W-1]}}, m00};
                7'h31: sh_out <= {{(32-W){m01[W-1]}}, m01};
                7'h32: sh_out <= {{(32-W){m02[W-1]}}, m02};
                7'h33: sh_out <= {{(32-W){m10[W-1]}}, m10};
                7'h34: sh_out <= {{(32-W){m11[W-1]}}, m11};
                7'h35: sh_out <= {{(32-W){m12[W-1]}}, m12};
                7'h36: sh_out <= {{(32-W){m20[W-1]}}, m20};
                7'h37: sh_out <= {{(32-W){m21[W-1]}}, m21};
                7'h38: sh_out <= {{(32-W){m22[W-1]}}, m22};
                7'h40: sh_out <= drift_slope_x;
                7'h41: sh_out <= drift_slope_y;
                7'h42: sh_out <= drift_slope_z;
                7'h50: sh_out <= {{(32-W){out_x[W-1]}}, out_x};
                7'h51: sh_out <= {{(32-W){out_y[W-1]}}, out_y};
                7'h52: sh_out <= {{(32-W){out_z[W-1]}}, out_z};
                7'h53: sh_out <= out_mag2;
                default: sh_out <= 32'h0;
              endcase
            end
            if(bitcnt==39) begin
              in_crc_phase <= 1'b1;
              bitcnt <= 0;
            end
          end else begin
            // receive CRC byte
            crc_recvd <= {crc_recvd[6:0], mosi};
            bitcnt <= bitcnt + 1'b1;
            if(bitcnt==7) begin
              // end of CRC
              if(!rw) begin
                crc_error <= (crc_recvd != crc_rx);
                if(crc_recvd == crc_rx) begin
                  // commit write
                  case (addr)
                    7'h02: begin notch50_en<=sh_in[2]; notch60_en<=sh_in[3]; selftest_en<=sh_in[4]; cal_start<=sh_in[0]; end
                    7'h10: offset_x <= sh_in[23:0];
                    7'h11: offset_y <= sh_in[23:0];
                    7'h12: offset_z <= sh_in[23:0];
                    7'h14: gain_x   <= sh_in[23:0];
                    7'h15: gain_y   <= sh_in[23:0];
                    7'h16: gain_z   <= sh_in[23:0];
                    7'h18: drift_freeze_th <= sh_in[23:0];
                    7'h19: drift_alpha_shift <= sh_in[7:0];
                    7'h1A: osr_log2 <= sh_in[7:0];
                    7'h1B: b1_50 <= sh_in[23:0];
                    7'h1C: a1_50 <= sh_in[23:0];
                    7'h1D: a2_50 <= sh_in[23:0];
                    7'h1E: b1_60 <= sh_in[23:0];
                    7'h1F: a1_60 <= sh_in[23:0];
                    7'h20: a2_60 <= sh_in[23:0];
                    7'h30: m00 <= sh_in[23:0];
                    7'h31: m01 <= sh_in[23:0];
                    7'h32: m02 <= sh_in[23:0];
                    7'h33: m10 <= sh_in[23:0];
                    7'h34: m11 <= sh_in[23:0];
                    7'h35: m12 <= sh_in[23:0];
                    7'h36: m20 <= sh_in[23:0];
                    7'h37: m21 <= sh_in[23:0];
                    7'h38: m22 <= sh_in[23:0];
                    default: ;
                  endcase
                end
              end
            end
          end
        end

        // shift out data and then CRC for reads
        if(sclk_fall) begin
          if(!in_crc_phase) begin
            miso_r   <= sh_out[31];
            // feed TX CRC with the same bit stream the master will check: rw+addr+data
            // we begin feeding during command and data period. We already fed RX CRC with MOSI.
            sh_out   <= {sh_out[30:0], 1'b0};
          end else begin
            // output CRC of [rw|addr|data]
            miso_r <= crc_tx[7 - bitcnt];
          end
        end
      end
    end
  end
endmodule
