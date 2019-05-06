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



module  TB_shift_key_FSM();

wire clk, uppercase;
reg reset_clk,reset,ready;
reg [7:0] scan_code; 

clkgen inputs( reset_clk, clk);

shift_key_FSM test(	.clk(clk),		//input wire clk,
					.reset(reset),	//input wire reset,
					.ready(ready),	//input wire ready,
					.scan_code(scan_code),	//input wire [7:0] scan_code, 
					.uppercase(uppercase)	//output wire uppercase // goes high when we are in upper case state. Used by chat module
							);

initial begin
ready=1;
reset_clk=1'd0;
#10;
reset_clk=1'd1;
#10
reset=1'h1;
#10;
reset=1'h0;
#10;
scan_code=8'h12;
#10
ready=0;
#10
ready=1;
scan_code=8'h19;
#10
ready=0;
#10;
ready=1;
scan_code=8'h59;
#10
ready=0;
#10;

end
							
endmodule