
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