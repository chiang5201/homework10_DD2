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
		
wire chat,cursor;

						
show back_ground( 
			.CLOCK_50(CLOCK_50),
			.clk(clk75),
			.reset(reset),
			.px(px),
		   .py(py),
			.out(chat),
			.sw(SW),//<== testing input
			.cursor(cursor),
			.PS2_DAT(PS2_DAT),
			.PS2_CLK(PS2_CLK),
			.LEDR(LEDR)
			);
						
						
	assign VGA_B =	(chat|cursor) ? 8'hff:8'h0;//ball 
   assign VGA_R = (chat|cursor)?8'hff:8'h0;//char
   assign VGA_G = (chat|cursor)?8'hff:8'h0;//( (~VGA_R)&1'b1&back) ? 8'hff:8'h0;
	
	
	
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

pressed A(  .clk(clk),//input wire clk,
				.reset(reset), //input wire reset,
				.ready(makeBreak), //input wire ready,
				.done(scan_code[7]),//input wire done,
				.out(move_right));//output reg out);



// this counter is used to add delay between a keypress, to allow single character press
wire move_right;
keyboard_press_driver keyboard_driver(clk, valid, makeBreak, scan_code, PS2_DAT, PS2_CLK, reset);

// shift key logic instantiation
wire uppercase;
shift_key_FSM shiftKeyGovernor(clk, reset, makeBreak, scan_code, uppercase);
//==============================================


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
										.addr(addrROM),
										.moved(input_registered));//output wire addr);


multiplexer out_display(.sel(npx2[3:0]),//input [7:0] sel, 
								.data(bits),		//input [7:0]data,
								.display(outpix));// output reg display);

wire [6:0] charts;
wire cu,cd,cl,cr,enter,back,shift;								
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
//==================================================================================
module pressed(input wire clk,
					input wire reset,
					input wire ready,
					input wire done,
					output reg out);

reg [9:0] step,nstep;

always  @(posedge clk)
begin 
 if(reset) step<=0;
 else  step<=nstep;
end

//initial step=0;
always @(*)
begin
out=0;
nstep=step; 
	case(step)
	10'd0: if(ready) nstep=1;
	10'd1: if(ready) nstep=2;
	10'd2: if(ready) nstep=3;
	10'd3: if(ready) nstep=4;
	10'd4: if(ready) nstep=5;
	10'd5: if(ready) nstep=6;
	10'd6:	begin  
					nstep=0;
					out=1;
			end
	default: nstep=0;		
	endcase 

end

endmodule 
//=====================================================
module cursor_position(input wire clk,
							  input wire reset,
							  input wire action,
							  input wire left,
							  input wire right,
							  input wire up,
							  input wire down,
							  input wire enter,
							  input wire back,
							  input wire shift,
							  input wire [10:0] px,
							  input wire [10:0] py,
							  output wire place,
							  output wire [12:0] addr,
							  output wire moved // this goes high for a clock cycle whenever the cursor changes position
							  );
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
reg bs;

always @(*) begin
	ny=y;
	nx=x;
	ncounter=counter;
	display=0;
	if((left||back) && action)
	 begin
	 if(y==0 && x==0) bs=0; 

	 else if(x>0)
		begin
			nx=x-8;
			ncounter=counter-1;
		end
	 else 
		begin
			nx=1016;
			ny=y-16;
		   ncounter=counter-1;
		end
	 end
	else if( right && action)
	 begin
	 if(x<1015)
		begin
			nx=x+8;
			ncounter=counter+1;
		end
	 end
	else if(up&& action) 
	 begin
	 if(y>0)
		begin
			ny=y-16;
			ncounter=counter-128;
		end
	 end
	else if(down && action)  
	 begin
	 if(y<751)
		begin
			ny=y+16;
			ncounter=counter+128;
		end
	 end
	 else if(enter && action)
	 begin
	  	 if(y<751)
		 begin
			ny=y+16;
			nx = 0;
			ncounter=128*(y/16)+128;
		end
	 end
	 else if(shift) bs=0;

	 else if(action)
	  begin
			if(x<1015)
			begin
				nx=x+8;
				ncounter=counter+1;
			end
			else
			begin
				ny = y + 16; // at end of line; go to next line
				nx = 0;
				ncounter=counter+1;
			end
		end
	 
	if(px-nx<8 && py-ny <16) display=1;
end

assign moved = counter != ncounter;
endmodule
//======================================================================================
module chat(input  wire clk,
				input  wire reset,
				input  wire [7:0]in,
				output reg [6:0]out,
				output reg up,
				output reg donw,
				output reg right,
				output reg left,
				output reg enter,
				output reg back,
				output reg shift,
				input wire uppercase
				);

reg [6:0] display;
reg bs;


always @(posedge clk)          begin
if(reset)	out<=7'b0;

else 			out<=display;

	
											end

always @(*)
begin
up=0;
donw=0;
right=0;
left=0;
enter=0;
back=0;
shift=0;
display =0;

	  if(in[7]==1) bs=1;
else if(	8'h59==in || 8'h12==in) shift=1;	 
else if(8'h75==in) up=1;
else if(8'h72==in) donw=1;
else if(8'h6b==in) left=1;
else if(8'h74==in) right=1;
else if(8'h5a==in) enter=1;
else if(8'h66==in) back=1;
	else if(uppercase) begin
		case(in)
			8'h1C: display = 8'h41; //A
          8'h32: display = 8'h42; //B
          8'h21: display = 8'h43; //C
          8'h23: display = 8'h44; //D
          8'h24: display = 8'h45; //E
          8'h2B: display = 8'h46; //F
          8'h34: display = 8'h47; //G
          8'h33: display = 8'h48; //H
          8'h43: display = 8'h49; //I
          8'h3B: display = 8'h4A; //J
          8'h42: display = 8'h4B; //K
          8'h4B: display = 8'h4C; //L
          8'h3A: display = 8'h4D; //M
          8'h31: display = 8'h4E; //N
          8'h44: display = 8'h4F; //O
          8'h4D: display = 8'h50; //P
          8'h15: display = 8'h51; //Q
          8'h2D: display = 8'h52; //R
          8'h1B: display = 8'h53; //S
          8'h2C: display = 8'h54; //T
          8'h3C: display = 8'h55; //U
          8'h2A: display = 8'h56; //V
          8'h1D: display = 8'h57; //W
          8'h22: display = 8'h58; //X
          8'h35: display = 8'h59; //Y
          8'h1A: display = 8'h5A; //Z
			 //number key row
			 8'h16: display = 8'h21; //!
          8'h52: display = 8'h22; //"
          8'h26: display = 8'h23; //#
          8'h25: display = 8'h24; //$
          8'h2E: display = 8'h25; //%
          8'h3D: display = 8'h26; //&              
          8'h46: display = 8'h28; //(
          8'h45: display = 8'h29; //)
          8'h3E: display = 8'h2A; //*
          8'h55: display = 8'h2B; //+
          8'h4C: display = 8'h3A; //:
          8'h41: display = 8'h3C; //<
          8'h49: display = 8'h3E; //>
          8'h4A: display = 8'h3F; //?
          8'h1E: display = 8'h40; //@
          8'h36: display = 8'h5E; //^
          8'h4E: display = 8'h5F; //_
          8'h54: display = 8'h7B; //{
          8'h5D: display = 8'h7C; //|
          8'h5B: display = 8'h7D; //}
          8'h0E: display = 8'h7E; //~
			 default: display = 8'h00;
		endcase
	end else begin // not uppercase
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
	end // end else (not uppercase)

end 
				
				

endmodule 

//______________________SHIFT KEY LOGIC____________________
// This module watches the scan code and uses an FSM to decide whether 
// or not output should be uppercase/special characters
module shift_key_FSM(input wire clk,
							input wire reset,
							input wire ready,
							input wire [7:0] scan_code, 
							output wire uppercase // goes high when we are in upper case state. Used by chat module
							);
	localparam lower = 0, upper_right = 1, upper_left = 2;
	reg [1:0] state, nstate;
	reg BS;
	
	always@(posedge clk) begin
		if(reset) state <= lower; // for initial state
		else begin
			state <= nstate;
		end
	end
	
	always@(*) begin
		nstate = state;
		if(scan_code[7]==1) BS=0;
		else
		begin	
			case({scan_code})			
				8'h59: nstate = upper_right;
				8'h12: nstate = upper_left ;
				default: nstate = state;
			endcase			
			if(~ready) nstate = lower;			
		end
	end
	assign uppercase = (state != lower) ? 1 : 0;					
endmodule
							
							