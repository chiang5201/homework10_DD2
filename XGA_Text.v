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
	keypressed button00(.clock(clk75), .reset(~SW[9]), .enable_in(KEY[0]), .enable_out(key00));//reset should be to 1 to work	
	keypressed button01(.clock(clk75), .reset(~SW[9]), .enable_in(KEY[1]), .enable_out(key01));
	keypressed button02(.clock(clk75), .reset(~SW[9]), .enable_in(KEY[2]), .enable_out(key02));
	keypressed button03(.clock(clk75), .reset(~SW[9]), .enable_in(KEY[3]), .enable_out(key03));	
	
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
			input  wire clk, // 75 mhz for VGA
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
reg [10:0] npx, cpx, npx2;
wire outpix,cpix;
assign out=outpix;
assign cursor=cpix;

//===========Logic to take keyboard codes and pass them to font rom=========
wire scan_ready;
wire [7:0] scan_code;
/*keyboard_scancoderaw_driver keyboard_input(
  CLOCK_50, 
  KEY,
  scan_ready, // 1 when a scan_code arrives from the inner driver
  scan_code, // most recent byte scan_code
  PS2_DAT, // PS2 data line
  PS2_CLK, // PS2 clock line
  sw[9], // this is the reset signal
  LEDR
);
*/
wire valid, makeBreak;
assign LEDR[0] = makeBreak;
assign LEDR[1] = valid;
assign LEDR[9:2] = scan_code;

always@(posedge clk) begin // syncronous block on 75 mHz clock
	npx<=cpx;
	npx2<=npx;
	if(reset) begin
		delay_state <= idle;
		press_delay <= 0;
		done_delay <= 0;
	end else begin
		press_delay <= npress_delay;
		delay_state <= ndelay_state;
		done_delay <= ndone_delay;
	end
end

always @(*)begin	
//npx=cpx; 
cpx=px;

end

// this counter is used to add delay between a keypress, to allow single character press
reg [24:0] press_delay, npress_delay; // without, a single press results in stream of around 4-6 characters!
reg [19:0] done_delay, ndone_delay;
reg [1:0] delay_state, ndelay_state;
localparam idle = 0, counting = 1, done = 2;
wire move_right;
assign move_right = (delay_state == done) ? 1: 0;

// press delay assignment logic
always@(*) begin
	ndone_delay = done_delay;
	npress_delay = press_delay;
	if(delay_state == idle) ndone_delay = 0;
	else if(delay_state == counting) npress_delay = press_delay + 1'b1; // waiting state when delay_state == 1
	else if(delay_state == done) ndone_delay = done_delay + 1'b1;
end

//next press delay state assignment logic
always@(*) begin
ndelay_state = delay_state;
	case(delay_state)
		idle: if(makeBreak) ndelay_state = counting;
		counting: if(press_delay == 0) ndelay_state = done;
		done: if(done_delay == {sw[8:1], 4'hf, 8'hff}) ndelay_state = idle; //if(done_delay == 16'hffff) ndelay_state = idle;
		default: ndelay_state = delay_state;
	endcase
end



keyboard_press_driver keyboard_driver(clk, valid, makeBreak, scan_code, PS2_DAT, PS2_CLK, reset);
//==============================================


wire [12:0] addrROM;//<======================================================================change to addr of rom
cursor_position display_cursor(.clk(clk),		//input wire clk,
										.reset(reset),	//input wire reset,
										.left(left),	//input wire left,
										.right(move_right),	//input wire right,
										.up(up),			//input wire up,
										.down(down),	//input wire down,
										.px(px),			//input wire [10:0] px,
										.py(py),			//input wire [10:0] py,
										.place(cpix),	//output wire place
										.addr(addrROM));//output wire addr);


multiplexer out_display(.sel(npx2[3:0]),//(px%8),			//input [7:0] sel, 
								.data(bits),		//input [7:0]data,
								.display(outpix));// output reg display);

wire [6:0] charts;								
chat	look_up_chart( .clk(clk),//input  wire clk,
							.reset(reset),
							.in(scan_code),//input  wire [7:0]in,
							.out(charts) );//output reg  [6:0]out)							
								
								
dual_port_ram_sync ROM
   (.clk(clk),
	 //.we(sw[0]),
	 .we(sw[0]),
    .addr_a(addrROM),//<======change ROM addr
	 .addr_b({py[9:4],px[9:3]}),
    //.din_a(6'h33),//<==========char
    .din_a(charts),
	 .dout_a(unkonw), 
	 .dout_b(B)) ;
 
 
 
 
font_rom   look_UP_table(.clk(clk),      //input wire clk,
								 .addr({B,py[3:0]} ),   //input wire [10:0] addr,
								 //.addr({scan_code[6:0],py[3:0]}),
								 .data(bits)  );//output reg [7:0] data



endmodule
//===================================================
module multiplexer(input wire [2:0] sel, 
						 input wire [7:0]data,
						 output reg display);
 
always @ (*)
begin
	case(sel)
		3'd0:display= data[7];
		3'd1:display= data[6];
		3'd2:display= data[5];
		3'd3:display= data[4];
		3'd4:display= data[3];
		3'd5:display= data[2];
		3'd6:display= data[1];
		3'd7:display= data[0];
	default:display=0;	
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
	else if(right)
	 begin
	 if(x<1015)
		begin
			nx=x+8;
			ncounter=counter+1;
		end else begin
			ny = y + 16; // at end of line; go to next line
			nx = 0;
			ncounter=counter+1;
		end
	 end
	else if(up) 
	 begin
	 if(y>0)
		begin
			ny=y-16;
			ncounter=counter-128;
		end
	 end
	else if(down) 
	 begin
	 if(y<751)
		begin
			ny=y+16;
			ncounter=counter+128;
		end
	 end
	 
	 
	if(px-nx<8 && py-ny <16) display=1;
end

endmodule
//======================================================================================
module chat(input  wire clk,
				input  wire reset,
				input  wire [7:0]in,
				output reg [6:0]out);
reg [6:0] display;




reg[2:0] lshift;
//initial lshift=1;


always @(posedge clk)          begin
if(reset)
	begin
	out<=7'b0;
	lshift<=0;
	end

else 
begin
out<=display;
if(in==8'b0001_0010)
	begin
		lshift=lshift+1;
		
		
	end
end
	
											end

always @(*)
begin
if(~lshift)
	case (in)
	8'b0100_0101:display=7'h30;//"0"
	8'b0001_0110:display=7'h31;//"1"
	8'b0001_1110:display=7'h32;//"2"
	8'b0010_0110:display=7'h33;//"3"
	8'b0010_0101:display=7'h34;//"4"
	8'b0010_1110:display=7'h35;//"5"
	8'b0011_0110:display=7'h36;//"6"
	8'b0011_1101:display=7'h37;//"7"
	8'b0011_1110:display=7'h38;//"8"
	8'b0100_0110:display=7'h39;//"9"
   8'h52: display = 8'h27; //'
   8'h41: display = 8'h2C; //,
   8'h4E: display = 8'h2D; //-
   8'h49: display = 8'h2E; //.
   8'h4A: display = 8'h2F; ///
   8'h4C: display = 8'h3B; //;
   8'h55: display = 8'h3D; //=
   8'h54: display = 8'h5B; //[
   8'h5D: display = 8'h5C; //\
   8'h5B: display = 8'h5D; //]
   8'h0E: display = 8'h60; //`  	
	8'h1C: display = 8'h61; //a
   8'h32: display = 8'h62; //b
   8'h21: display = 8'h63; //c
   8'h23: display = 8'h64; //d
   8'h24: display = 8'h65; //e
   8'h2B: display = 8'h66; //f
   8'h34: display = 8'h67; //g
   8'h33: display = 8'h68; //h
   8'h43: display = 8'h69; //i
   8'h3B: display = 8'h6A; //j
   8'h42: display = 8'h6B; //k
   8'h4B: display = 8'h6C; //l
   8'h3A: display = 8'h6D; //m
   8'h31: display = 8'h6E; //n
   8'h44: display = 8'h6F; //o
   8'h4D: display = 8'h70; //p
   8'h15: display = 8'h71; //q
   8'h2D: display = 8'h72; //r
   8'h1B: display = 8'h73; //s
   8'h2C: display = 8'h74; //t
   8'h3C: display = 8'h75; //u
   8'h2A: display = 8'h76; //v
   8'h1D: display = 8'h77; //w
   8'h22: display = 8'h78; //x
   8'h35: display = 8'h79; //y
   8'h1A: display = 8'h7A; //z	
	default:display=7'h00;
	endcase

else
	case(in)
   8'b0100_0101:display=7'h20;//")"
	8'b0001_0110:display=7'h21;//"!"
	8'b0001_1110:display=7'h40;//"@"
	8'b0010_0110:display=7'h23;//"#"
	8'b0010_0101:display=7'h24;//"$"
	8'b0010_1110:display=7'h25;//"%"
	8'b0011_0110:display=7'h26;//"^"
	8'b0011_1101:display=7'h27;//"&"
	8'b0011_1110:display=7'h28;//"*"
	8'b0100_0110:display=7'h29;//"("
	default:display=7'h00;
	endcase

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
