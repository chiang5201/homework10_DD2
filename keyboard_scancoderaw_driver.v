module keyboard_scancoderaw_driver(
  input  CLOCK_50, 
  input [3:0] KEY,
  output scan_ready, // 1 when a scan_code arrives from the inner driver
  output [7:0] scan_code, // most recent byte scan_code
  input    PS2_DAT, // PS2 data line
  input    PS2_CLK, // PS2 clock line
  input reset,
  output [9:0] LEDR
);
	
wire read;

assign LEDR[9:2] = scan_code;
reg [24:0] count, nextcount;
always@(posedge CLOCK_50) begin

	count <= nextcount;
end

always@(*) begin
	nextcount = count + 1;
end
assign read = (count ==0 ) ? 1 : 0;
assign LEDR[0] = (count == 0) ? ~LEDR[0] : LEDR[0];

// generates the read signal for the keyboard inner driver
//oneshot pulser(
//   .pulse_out(read),
//   .trigger_in(scan_ready),
//   .clk(CLOCK_50)
//);

// inner driver that handles the PS2 keyboard protocol
// outputs a scan_ready signal accompanied with a new scan_code
keyboard_inner_driver kbd(
  .keyboard_clk(PS2_CLK),
  .keyboard_data(PS2_DAT),
  .clock50(CLOCK_50),
  .reset(reset),
  .read(read),
  .scan_ready(scan_ready),
  .scan_code(scan_code)
);

endmodule