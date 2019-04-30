module XGA_Text(

	input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    output wire [7:0] VGA_R,
    output wire [7:0] VGA_G,
    output wire [7:0] VGA_B,
    output wire       VGA_CLK,
    output wire       VGA_BLANK_N,
    output wire       VGA_SYNC_N,
    output wire       VGA_HS,
    output wire       VGA_VS,
	 input wire [9:0] SW,
	 input wire PS2_DAT,
	 input wire PS2_CLK,
	 output [9:0] LEDR);

	//---------- YOUR DESIGN STARTS HERE -----------------
	// You can add code to this file, or add new verilog files.
	// If you add new files, put all of them in the same directory
	// as this file.
wire unknow, clk75, sysnc, reset;
	
 clk75M  clk00(
		.refclk(CLOCK_50),   //  refclk.clk
		.rst(1'd0),      //   reset.reset
		.outclk_0(clk75), // outclk0.clk
		.locked(unkonw)    //  locked.export
	);
assign reset=SW[9];	

wire key00, key01, key02, key03;
	keypressed button00(.clock(clk75), .reset(SW[9]), .enable_in(KEY[0]), .enable_out(key00));	
	keypressed button01(.clock(clk75), .reset(SW[9]), .enable_in(KEY[1]), .enable_out(key01));
	keypressed button02(.clock(clk75), .reset(SW[9]), .enable_in(KEY[2]), .enable_out(key02));
	keypressed button03(.clock(clk75), .reset(SW[9]), .enable_in(KEY[3]), .enable_out(key03));	
/*
reg Bg,ball;	
	assign reset=~KEY[0];	
always @ (posedge clk75)begin
if (reset)
 begin 
	Bg<=0;
	ball<=0;
 end
else if(key01)
	Bg<=1;
else if(key02)
	Bg<=0;
else if(key03) 
	ball<=1;
	
end	
*/
	
assign VGA_CLK=clk75;	
assign VGA_SYNC_N = ~sysnc;
wire[10:0] py,px;

vesasync pix00(
    .clk(clk75),             // 75 MHz clock
    .reset(reset),            // positive reset
    .hsync(VGA_HS),           // hsync signal
    .vsync(VGA_VS),           // vsync signal
    .csync(sysnc),           // composite sync signal
    .video_on(VGA_BLANK_N),        // beam on
    .pixel_x(px),   // pixel X ccordinate
    .pixel_y(py)  // pixel Y coordinate
);	
		
wire print, back,at;
mover_ball coloer( .clk(clk75), 
						.reset(reset),
						.px(px),
						.py(py), 
						.color(print));

						
show back_ground( 
			.CLOCK_50(CLOCK_50),
			.clk(clk75),
			.reset(reset),
			.px(px),
		   .py(py),
			.out(back),
			.sw(SW),//<== testing input
			.up(~KEY[1]),
			.down(~KEY[2]),
			.right(~KEY[0]),
			.left(~KEY[3]),
			.cursor(at),
			.PS2_DAT(PS2_DAT),
			.PS2_CLK(PS2_CLK),
			.LEDR(LEDR)
			);
						
						
	assign VGA_B =	 (print) ? 8'hff:8'h0;//ball 
   assign VGA_R = (back)?8'hff:8'h0;//char
   assign VGA_G = (at)?8'hff:8'h0;//( (~VGA_R)&1'b1&back) ? 8'hff:8'h0;
	
	
	
endmodule
//===============================================
module show(
			input wire CLOCK_50,
			input  wire clk,
			input  wire reset,
			input  wire [10:0] px,
		   input  wire [10:0] py,
			output wire out,
			 input  wire [9:0]sw,
 			 input  wire up,
			 input  wire down,
			 input  wire right,
			 input  wire left,
			output  wire cursor,
			input wire PS2_DAT,
			input wire PS2_CLK,
			output wire [9:0] LEDR); 
				
wire unkonw;
wire [6:0] B;	 
wire [7:0] bits;
reg [12:0] counter,Ncounter;
reg [10:0] val,Nval; 
wire outpix,cpix;
assign out=outpix;
assign cursor=cpix;

//===========Logic to take keyboard codes and pass them to font rom=========
wire scan_ready;
wire [7:0] scan_code;
keyboard_scancoderaw_driver keyboard_input(
  CLOCK_50, 
  KEY,
  scan_ready, // 1 when a scan_code arrives from the inner driver
  scan_code, // most recent byte scan_code
  PS2_DAT, // PS2 data line
  PS2_CLK, // PS2 clock line
  sw[9], // this is the reset signal
  LEDR
);
//==============================================


wire [12:0] addrROM;//<======================================================================change to addr of rom
cursor_position display_cursor(.clk(clk),		//input wire clk,
										.reset(reset),	//input wire reset,
										.left(left),	//input wire left,
										.right(right),	//input wire right,
										.up(up),			//input wire up,
										.down(down),	//input wire down,
										.px(px),			//input wire [10:0] px,
										.py(py),			//input wire [10:0] py,
										.place(cpix),	//output wire place
										.addr(addrROM));//output wire addr);


multiplexer out_display(.sel(px%8),			//input [7:0] sel, 
								.data(bits),		//input [7:0]data,
								.display(outpix));// output reg display);

dual_port_ram_sync ROM
   (.clk(clk),
	 .we(sw[0]),
    .addr_a(addrROM),//<======change ROM addr
	 .addr_b({py[9:4],px[9:3]}),
    .din_a(sw[6:1]),//(val[8:2]),//(6'h33),//<==========char
    .dout_a(unkonw), 
	 .dout_b(B)) ;
 
 
font_rom   look_UP_table(.clk(clk),      //input wire clk,
								 //.addr({B,py[3:0]} ),   //input wire [10:0] addr,
								 .addr({scan_code[6:0],py[3:0]}),
								 .data(bits)  );//output reg [7:0] data

		
always @ (posedge clk)begin
if(reset)
 begin	
	counter<=0;
	val<=97;
 end
else
 begin
	counter<=Ncounter;
	val<=Nval;
 end
 
end
always @(*)begin	
Nval=val;
Ncounter=counter;		

if(counter<6144)
 begin	
		Nval=(71*val+11)%2047;
	Ncounter=counter+1;		
 end
 
end

endmodule
//===================================================
module multiplexer(input wire [7:0] sel, 
						 input wire [7:0]data,
						 output reg display);
 
always @ (*)
begin
	case(sel)
		8'd0:display= data[7];
		8'd1:display= data[6];
		8'd2:display= data[5];
		8'd3:display= data[4];
		8'd4:display= data[3];
		8'd5:display= data[2];
		8'd6:display= data[1];
		8'd7:display= data[0];
	endcase
end

endmodule
//=====================================================
module cursor_position(input wire clk,
							  input wire reset,
							  input wire left,
							  input wire right,
							  input wire up,
							  input wire down,
							  input wire [10:0] px,
							  input wire [10:0] py,
							  output wire place,
							  output wire [12:0] addr);
reg[10:0] x,y,nx,ny;							  
reg[12:0] counter, ncounter;
reg display;

assign place=display;
assign addr=counter;
							  
always @(posedge clk)begin
if(reset)
 begin
 x<=0;//512;
 y<=0;//384;
 counter<=0;
 end
else if(px==1328 && py==805)
 begin
 x<=nx;
 y<=ny;
 counter<=ncounter;
 end 
							end
always @(*) begin
ny=y;
nx=x;
ncounter=counter;
display=0;
if(left)
 begin
 if(x>0)
	begin
		nx=x-8;
		ncounter=counter-1;
	end
 end
if(right)
 begin
 if(x<1015)
	begin
		nx=x+8;
		ncounter=counter+1;
	end
 end
if(up) 
 begin
 if(y>0)
	begin
		ny=y-16;
		ncounter=counter-128;
	end
 end
if(down) 
 begin
 if(y<751)
	begin
		ny=y+16;
		ncounter=counter+128;
	end
 end
 
 
if(px-nx<8 && py-ny <16) 
 display=1;
				end

endmodule
//====================================================
module mover_ball(input  wire clk, 
						input  wire reset,
						input  wire [10:0] px,
						input  wire [10:0] py, 
						output wire color);

localparam initi=0, upr=1, upl=2, downr=3, downl=4;
reg[10:0] bx, by, nbx, nby;
reg[ 2:0] stat,nstat;
						
always @(posedge clk)
begin
	if(reset)
	begin
		bx<=512;
		by<=384;
		stat<=0;
	end
	else if(px==1328 && py==805)
	begin
		stat<=nstat;
		bx<=nbx;
		by<=nby;
	end
end	

reg print;

always @(*)
begin
nbx=bx;
nby=by;
nstat=stat;
 
case(stat)	
initi:begin
nstat=downr;
end
upr:begin
nby=by-1;
nbx=bx+1;
	if(nby==16)
		nstat=downr;
   else if(nbx==1007)
		nstat=upl;
end
upl:begin
nby=by-1;
nbx=bx-1;
	if(nby==16)
		nstat=downl;
	else if(nbx==16)
		nstat=upr;
end
downr:begin
nby=by+1;
nbx=bx+1;
	if(nby==751)
		nstat=upr;
	else if(nbx==1007)
		nstat=downl;
end
downl:begin
nby=by+1;
nbx=bx-1;
	if(nby==751)
		nstat=upl;
	else if(nbx== 16)
		nstat=downr;
end
default stat= initi;
endcase	

print=0;
//( ( (px-nbx)*(px-nbx)+(py-nby)*(py-nby)  )< (32*32)   )
if( ( (px-nbx)*(px-nbx)+(py-nby)*(py-nby)  )< (16*16)   )
	print=1;

end	
	
assign color= print;
	
endmodule


