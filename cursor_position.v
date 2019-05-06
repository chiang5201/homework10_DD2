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
							  output wire [12:0] addr
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
