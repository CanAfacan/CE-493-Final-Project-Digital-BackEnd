// DIGITAL CHAIN (! INCOMPLETE !)

module mag_dig_top #(
  parameter ADC_BITS=10,
  parameter W=24,
  parameter FRAC=12,
  parameter LOG2_OSR_MAX=8
)(
  input                 clk,
  input                 rst_n,

  // ADC interface from analog SAR and MUX per Midterm
  input  [ADC_BITS-1:0] adc_code,
  input                 adc_valid,
  input  [1:0]          mux_sel,

  // SPI
  input  sclk,
  input  cs_n,
  input  mosi,
  output miso,

  input  [31:0]         version_id
);
  // ADC demux
  wire signed [W-1:0] xs, ys, zs; wire xv, yv, zv;
  adc_axis_router #(.ADC_BITS(ADC_BITS),.W(W),.FRAC(FRAC)) UROUT (
    .clk(clk), .rst_n(rst_n),
    .adc_code(adc_code), .adc_valid(adc_valid), .mux_sel(mux_sel),
    .x_sample(xs), .y_sample(ys), .z_sample(zs),
    .x_valid(xv), .y_valid(yv), .z_valid(zv)
  );

  // Config via SPI
  wire notch50_en, notch60_en, selftest_en;
  wire [7:0] osr_log2, drift_alpha_shift;
  wire signed [W-1:0] drift_freeze_th;
  wire signed [W-1:0] offset_x,offset_y,offset_z;
  wire signed [W-1:0] gain_x,gain_y,gain_z;
  wire signed [W-1:0] b1_50,a1_50,a2_50,b1_60,a1_60,a2_60;
  wire signed [W-1:0] m00,m01,m02,m10,m11,m12,m20,m21,m22;
  wire cal_start;
  wire crc_error;

  // Outputs to SPI
  wire signed [W-1:0] out_x, out_y, out_z;
  wire [31:0] out_mag2;
  wire [31:0] drift_slope_x, drift_slope_y, drift_slope_z;
  wire cal_busy, cal_done;
  assign cal_busy = 1'b0; // simple pulse model below
  assign cal_done = 1'b0;

  spi_regfile_crc #(.W(W),.FRAC(FRAC)) USPI (
    .clk(clk), .rst_n(rst_n),
    .sclk(sclk), .cs_n(cs_n), .mosi(mosi), .miso(miso),
    .notch50_en(notch50_en), .notch60_en(notch60_en), .selftest_en(selftest_en),
    .osr_log2(osr_log2), .drift_alpha_shift(drift_alpha_shift), .drift_freeze_th(drift_freeze_th),
    .offset_x(offset_x), .offset_y(offset_y), .offset_z(offset_z),
    .gain_x(gain_x), .gain_y(gain_y), .gain_z(gain_z),
    .b1_50(b1_50), .a1_50(a1_50), .a2_50(a2_50),
    .b1_60(b1_60), .a1_60(a1_60), .a2_60(a2_60),
    .m00(m00), .m01(m01), .m02(m02), .m10(m10), .m11(m11), .m12(m12), .m20(m20), .m21(m21), .m22(m22),
    .cal_start(cal_start),
    .cal_busy(cal_busy), .cal_done(cal_done),
    .out_x(out_x), .out_y(out_y), .out_z(out_z), .out_mag2(out_mag2),
    .version_id(version_id),
    .drift_slope_x(drift_slope_x), .drift_slope_y(drift_slope_y), .drift_slope_z(drift_slope_z),
    .crc_error(crc_error)
  );

  // Oversampling decimators with runtime OSR
  wire signed [W-1:0] x_dec, y_dec, z_dec; wire xv_dec, yv_dec, zv_dec;
  osr_mavg_dyn #(.W(W), .LOG2_OSR_MAX(LOG2_OSR_MAX)) UDX (.clk(clk), .rst_n(rst_n), .din(xs), .din_valid(xv), .log2_osr(osr_log2), .dout(x_dec), .dout_valid(xv_dec));
  osr_mavg_dyn #(.W(W), .LOG2_OSR_MAX(LOG2_OSR_MAX)) UDY (.clk(clk), .rst_n(rst_n), .din(ys), .din_valid(yv), .log2_osr(osr_log2), .dout(y_dec), .dout_valid(yv_dec));
  osr_mavg_dyn #(.W(W), .LOG2_OSR_MAX(LOG2_OSR_MAX)) UDZ (.clk(clk), .rst_n(rst_n), .din(zs), .din_valid(zv), .log2_osr(osr_log2), .dout(z_dec), .dout_valid(zv_dec));

  // Offset and gain
  wire signed [W-1:0] x_cg,y_cg,z_cg;
  gain_offset_apply #(.W(W),.FRAC(FRAC)) CGX (.din(x_dec), .offset(offset_x), .gain(gain_x), .dout(x_cg));
  gain_offset_apply #(.W(W),.FRAC(FRAC)) CGY (.din(y_dec), .offset(offset_y), .gain(gain_y), .dout(y_cg));
  gain_offset_apply #(.W(W),.FRAC(FRAC)) CGZ (.din(z_dec), .offset(offset_z), .gain(gain_z), .dout(z_cg));

  // Drift tracking
  wire signed [W-1:0] x_drift, y_drift, z_drift;
  ema_drift #(.W(W)) DFX (.clk(clk), .rst_n(rst_n), .din(x_cg), .din_valid(xv_dec), .alpha_shift(drift_alpha_shift), .freeze_th(drift_freeze_th), .drift_out(x_drift));
  ema_drift #(.W(W)) DFY (.clk(clk), .rst_n(rst_n), .din(y_cg), .din_valid(yv_dec), .alpha_shift(drift_alpha_shift), .freeze_th(drift_freeze_th), .drift_out(y_drift));
  ema_drift #(.W(W)) DFZ (.clk(clk), .rst_n(rst_n), .din(z_cg), .din_valid(zv_dec), .alpha_shift(drift_alpha_shift), .freeze_th(drift_freeze_th), .drift_out(z_drift));

  wire signed [W-1:0] x_hp = x_cg - x_drift;
  wire signed [W-1:0] y_hp = y_cg - y_drift;
  wire signed [W-1:0] z_hp = z_cg - z_drift;

  // Notches
  wire signed [W-1:0] x_n50,y_n50,z_n50, x_n60,y_n60,z_n60;
  iir_notch2 #(.W(W),.FRAC(FRAC)) N50X (.clk(clk), .rst_n(rst_n), .din(x_hp), .din_valid(xv_dec), .b1(b1_50), .a1(a1_50), .a2(a2_50), .dout(x_n50), .dout_valid());
  iir_notch2 #(.W(W),.FRAC(FRAC)) N50Y (.clk(clk), .rst_n(rst_n), .din(y_hp), .din_valid(yv_dec), .b1(b1_50), .a1(a1_50), .a2(a2_50), .dout(y_n50), .dout_valid());
  iir_notch2 #(.W(W),.FRAC(FRAC)) N50Z (.clk(clk), .rst_n(rst_n), .din(z_hp), .din_valid(zv_dec), .b1(b1_50), .a1(a1_50), .a2(a2_50), .dout(z_n50), .dout_valid());

  wire signed [W-1:0] x_after50 = notch50_en ? x_n50 : x_hp;
  wire signed [W-1:0] y_after50 = notch50_en ? y_n50 : y_hp;
  wire signed [W-1:0] z_after50 = notch50_en ? z_n50 : z_hp;

  iir_notch2 #(.W(W),.FRAC(FRAC)) N60X (.clk(clk), .rst_n(rst_n), .din(x_after50), .din_valid(xv_dec), .b1(b1_60), .a1(a1_60), .a2(a2_60), .dout(x_n60), .dout_valid());
  iir_notch2 #(.W(W),.FRAC(FRAC)) N60Y (.clk(clk), .rst_n(rst_n), .din(y_after50), .din_valid(yv_dec), .b1(b1_60), .a1(a1_60), .a2(a2_60), .dout(y_n60), .dout_valid());
  iir_notch2 #(.W(W),.FRAC(FRAC)) N60Z (.clk(clk), .rst_n(rst_n), .din(z_after50), .din_valid(zv_dec), .b1(b1_60), .a1(a1_60), .a2(a2_60), .dout(z_n60), .dout_valid());

  wire signed [W-1:0] x_lin_in = notch60_en ? x_n60 : x_after50;
  wire signed [W-1:0] y_lin_in = notch60_en ? y_n60 : y_after50;
  wire signed [W-1:0] z_lin_in = notch60_en ? z_n60 : z_after50;

  // Self test hook left as a simple gate to zero
  wire signed [W-1:0] x_pre = selftest_en ? 0 : x_lin_in;
  wire signed [W-1:0] y_pre = selftest_en ? 0 : y_lin_in;
  wire signed [W-1:0] z_pre = selftest_en ? 0 : z_lin_in;

  // Linearization
  mat3x3_q12 #(.W(W),.FRAC(FRAC)) MAT (
    .x_in(x_pre), .y_in(y_pre), .z_in(z_pre),
    .m00(m00), .m01(m01), .m02(m02),
    .m10(m10), .m11(m11), .m12(m12),
    .m20(m20), .m21(m21), .m22(m22),
    .x_out(out_x), .y_out(out_y), .z_out(out_z)
  );

  // Magnitude squared for monitoring
  wire signed [2*W-1:0] xx = out_x * out_x;
  wire signed [2*W-1:0] yy = out_y * out_y;
  wire signed [2*W-1:0] zz = out_z * out_z;
  wire signed [2*W:0] sum = xx + yy + zz;
  assign out_mag2 = sum[31:0];

  // Simple drift slope metrics, update every 1024 decimated samples on each axis
  reg [9:0] cntx, cnty, cntz;
  reg signed [W-1:0] x_drift_prev, y_drift_prev, z_drift_prev;
  reg [31:0] slope_x, slope_y, slope_z;

  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      cntx<=0; x_drift_prev<=0; slope_x<=0;
      cnty<=0; y_drift_prev<=0; slope_y<=0;
      cntz<=0; z_drift_prev<=0; slope_z<=0;
    end else begin
      if(xv_dec) begin
        cntx <= cntx + 1'b1;
        if(cntx==10'd1023) begin slope_x <= x_drift - x_drift_prev; x_drift_prev <= x_drift; cntx<=0; end
      end
      if(yv_dec) begin
        cnty <= cnty + 1'b1;
        if(cnty==10'd1023) begin slope_y <= y_drift - y_drift_prev; y_drift_prev <= y_drift; cnty<=0; end
      end
      if(zv_dec) begin
        cntz <= cntz + 1'b1;
        if(cntz==10'd1023) begin slope_z <= z_drift - z_drift_prev; z_drift_prev <= z_drift; cntz<=0; end
      end
    end
  end

  assign drift_slope_x = slope_x;
  assign drift_slope_y = slope_y;
  assign drift_slope_z = slope_z;

endmodule
