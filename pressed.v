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
