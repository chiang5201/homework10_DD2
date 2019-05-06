module show(
			input wire CLOCK_50,
			input  wire clk, // 75 mhz for VGA
			input  wire reset,
			input  wire [10:0] px,
		   input  wire [10:0] py,
			output wire out,
			 input  wire [9:0]sw,
			output  wire cursor,
			input wire PS2_DAT,
			input wire PS2_CLK,
			output wire [9:0] LEDR); 
				
wire unkonw;
wire [6:0] B;	 
wire [7:0] bits;
reg [12:0] counter,Ncounter;
reg [10:0] val,Nval; 
reg [10:0] npx, cpx, npx2;
wire outpix,cpix;
assign out=outpix;
assign cursor=cpix;

//===========Logic to take keyboard codes and pass them to font rom=========
wire scan_ready;
wire [7:0] scan_code;
wire valid, makeBreak;
assign LEDR[0] = makeBreak;
assign LEDR[1] = valid;
assign LEDR[9:2] = scan_code;

always@(posedge clk) begin // syncronous block on 75 mHz clock
	npx<=cpx;
	npx2<=npx;
	
end

always @(*)begin	
cpx=px;
end




// this counter is used to add delay between a keypress, to allow single character press
wire move_right;
keyboard_press_driver keyboard_driver(clk, valid, makeBreak, scan_code, PS2_DAT, PS2_CLK, reset);

// shift key logic instantiation
wire uppercase;
shift_key_FSM shiftKeyGovernor(clk, reset, makeBreak, scan_code, uppercase);
//==============================================
wire [6:0] charts;
wire cu,cd,cl,cr,enter,back,shift;	
pressed A(  .clk(clk),//input wire clk,
				.reset(reset), //input wire reset,
				.ready(makeBreak), //input wire ready,
				.done(scan_code[7]),//input wire done,
				.out(move_right));//output reg out);




multiplexer out_display(.sel(npx2[3:0]),//input [7:0] sel, 
								.data(bits),		//input [7:0]data,
								.display(outpix));// output reg display);

							
chat	look_up_chart( .clk(clk),//input  wire clk,
							.reset(reset),
							.in(scan_code),//input  wire [7:0]in,
							.out(charts),
							.up(cu),
							.donw(cd),
							.right(cr),
							.left(cl),     //output reg  [6:0]out)	
							.enter(enter),
							.back(back),
							.shift(shift),
							.uppercase(uppercase)
							);
							
wire [12:0] addrROM;//<======================================================================change to addr of rom
cursor_position display_cursor(.clk(clk),		//input wire clk,
										.reset(reset),	//input wire reset,
										.action(move_right),
										.left(cl),	//input wire left,
										.right(cr),	//input wire right,
										.up(cu),			//input wire up,
										.down(cd),	//input wire down,
										.enter(enter),
										.back(back),
										.shift(shift),
										.px(px),			//input wire [10:0] px,
										.py(py),			//input wire [10:0] py,
										.place(cpix),	//output wire place
										.addr(addrROM));//output wire addr);							

dual_port_ram_sync ROM
   (.clk(clk),
	 .we( ~(cu | cd | cl | cr | scan_code[7] | enter )     ),
    .addr_a(addrROM),//<======change ROM addr
	 .addr_b({py[9:4],px[9:3]}),//<==========char
    .din_a(charts),
	 .dout_a(unkonw), 
	 .dout_b(B)) ;
 
font_rom   look_UP_table(.clk(clk),      //input wire clk,
								 .addr({B,py[3:0]} ),   //input wire [10:0] addr,
								 .data(bits)  );//output reg [7:0] data

endmodule
