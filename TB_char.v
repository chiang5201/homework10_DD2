`timescale 1 ns/100 ps
module clkgen(input reset, output reg clk);
	always
		begin
		
		if(reset==1'b0)
		#1	clk=1'b0;
		else
		#5	clk = !clk;
		end
endmodule

module TB_char();


wire clk,up,donw,right,left,enter,back,shift;
wire [6:0] out;
reg [7:0]in;
reg reset,uppercase,reseting;
 

clkgen inputs( reset, clk);
chat testing( 	.clk(clk), 		//input  wire clk,
				.reset(reseting),  //input  wire reset,
				.in(in),	 	//input  wire [7:0]in,
				.out(out), 		//output reg [6:0]out,
				.up(up), 		//output reg up,
				.donw(donw),	//output reg donw,
				.right(right), 	//output reg right,
				.left(left),	//output reg left,
				.enter(enter),	//output reg enter,
				.back(back), 	//output reg back,
				.shift(shift), 	//output reg shift,
				.uppercase(uppercase)	//input wire uppercase
);

	initial begin
reset=1'd0;
uppercase=0; 
reseting=1;
#10;
reset=1'd1;
reseting=0;
#10;
in=8'h12;//left shift key
#10;
in=8'h59;//right shift key
#10
in=8'h75; //up
#10
in=8'h72;//down
#10
in=8'h6b;//left
#10
in=8'h74;//right
#10
in=8'h5a;//enter
#10
in=8'h66;//backspace
#10;
in=8'h5A;//'z'
#10
in=8'h1c; //'a'
#10;
uppercase=1; 
in=8'h1c; //'A'
#10;
in=8'h5A; //'Z'
#10;
	end
endmodule