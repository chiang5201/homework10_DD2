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
`timescale 1 ns/100 ps
module TB_position();
reg reset_clk, reset,action, left, right, up, down, enter, back, shift;
reg [10:0]px, py;
wire place;
wire [12:0]addr;

clkgen inputs( reset_clk, clk);

cursor_position test(.clk(clk),			//input wire clk,
					 .reset(reset),		//input wire reset,
					 .action(action),			//input wire action,
					 .left(left),			//input wire left,
					 .right(right),			//input wire right,
					 .up(up),				//input wire up,
					 .down(down),			//input wire down,
					 .enter(enter),			//input wire enter,
					 .back(back),		  	//input wire back,
					 .shift(shift),			//input wire shift,
					 .px(px),				//input wire [10:0] px,
					 .py(py),		  		//input wire [10:0] py,
					 .place(place),			//output wire place,
					 .addr(addr)		  	//output wire [12:0] addr,
							  );
initial begin
px=12'd1328;
py=12'd805;
action=0;
left=0;
right=0;
up=0;
down=0;
enter=0; 
back=0;
shift=0;
reset_clk=1'd0;
#10;
reset_clk=1'd1;
#10
reset=1'h1;
#10;
reset=1'h0;
#10;
//============
action=1;
left=1;
#10
left=0;
right=1;
#10
right=0;
down=1;
#10
down=0;
up=1;
#10
up=0;
enter=1;
#10
enter=0;
back=1;
#10
back=0;
shift=1;
#10;
shift=0;
#10;

end

endmodule