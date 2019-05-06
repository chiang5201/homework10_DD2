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