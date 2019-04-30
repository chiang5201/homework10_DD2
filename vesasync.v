module vesasync( 
    input wire clk,              // 75 MHz clock
    input wire reset,            // positive reset
    output wire hsync,           // hsync signal
    output wire vsync,           // vsync signal
    output wire csync,           // composite sync signal
    output wire video_on,        // beam on
    output wire [10:0] pixel_x,   // pixel X ccordinate
    output wire [10:0] pixel_y ); // pixel Y coordinate

   reg [10:0] 	      h_count_reg, v_count_reg;
   wire [10:0] 	      h_count_next, v_count_next;

   reg 		      h_sync_reg, v_sync_reg;
   wire 	      h_sync_next, v_sync_next;
   
   wire 	      h_end, v_end;

   localparam HS = 136;  // H sync width in pixels
   localparam HB = 144;  // H back porch in pixels
   localparam HD = 1024; // H data area in pixels
   localparam HF = 24;   // H front porch in pixels

   localparam VS = 6;    // V sync width in lines
   localparam VB = 29;   // V back porch in lines
   localparam VD = 768;  // V data in lines
   localparam VF = 3;    // V front porch in lones
     
   always @(posedge clk or posedge reset)
     if (reset)
       begin
	  h_count_reg <= 10'b0;
	  v_count_reg <= 10'b0;
	  h_sync_reg  <= 1'b0;
	  v_sync_reg  <= 1'b0;
       end
     else
       begin
	  h_count_reg <= h_count_next;
	  v_count_reg <= v_count_next;
	  h_sync_reg  <= h_sync_next;
	  v_sync_reg  <= v_sync_next;
       end

   assign h_end = (h_count_reg == (HS + HB + HD + HF));
   assign v_end = (v_count_reg == (VS + VB + VD + VF));
   
   assign h_count_next = h_end ? 10'b0 : (h_count_reg + 1);
   assign v_count_next = v_end ? 10'b0 : 
			 h_end ? (v_count_reg + 1) : v_count_reg;
   
   assign h_sync_next  = ((h_count_reg >= (HD + HF)) &&
			  (h_count_reg <= (HD + HF + HS - 1)));
   assign v_sync_next  = ((v_count_reg >= (VD + VF)) &&
			  (v_count_reg <= (VD + VF + VS - 1)));
   
   assign video_on = (h_count_reg < HD) && (v_count_reg < VD);
   
   assign hsync = h_sync_reg;
   assign vsync = v_sync_reg;

   assign pixel_x = h_count_reg;
   assign pixel_y = v_count_reg;

   assign csync = h_sync_reg ^ v_sync_reg;
		   
endmodule // vesasync
