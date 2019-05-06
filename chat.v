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
		8'h5D: display = 8'h5C; // '\'
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
