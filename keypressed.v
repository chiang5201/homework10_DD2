module keypressed(clock, reset, enable_in, enable_out);
	input  clock;			// The system clock. (Connect to an FPGA clock signal in the top-level module.)
	input	 reset;			// Active-low reset. (Connect to a pushbutton in the top-level module.)
	input  enable_in;		// Enable input signal. (Connect to a pushbutton in the top-level module.)
	output enable_out;	// The output is high for one FPGA clock cycle each time enable_in is pressed and released.
	reg    enable_out;
	
// Variables for keeping track of the state.
	reg [1:0] key_state, next_key_state;

// Set up parameters for the "state" of the pushbutton.
// Since there are three states, we are using 2 bits to represent the state in a so-called "dense" assignment.
	parameter [1:0] KEY_FREE = 2'b00, KEY_PRESSED = 2'b01, KEY_RELEASED = 2'b10;
	
// STATE MACHINE: REGISTER BLOCK	
// This always block represents sequential logic, so it uses non-blocking assignments.
// It is sensitized to the clock input and the reset input.
// You should picture this block as a 2-bit register with an active-low asynchronous clear.

	always @(posedge clock or negedge reset) begin
	
// If reset = 0, there must have been a negative edge on the reset.
// Since the effect of the reset occurs in the absence of a clock pulse, the reset is ASYNCHRONOUS.

		if(reset == 1'b0)
			key_state <= KEY_FREE;
		
// If reset is not zero but this always block is executing, there must have been a positive clock edge.
// On each positive clock edge, the next state becomes the present state.

		else
			key_state <= next_key_state;

	end

// STATE MACHINE: REGISTER INPUT LOGIC
// This always block represents combinational logic, so it uses blocking assignments.
// It is sensitized to changes in the present key state and the enable input.
// You should picture this block as a combinational circuit that feeds the register inputs.
// It determines the next state based on the current state and the enable input.

	always@(key_state or enable_in) begin
	
// To be safe, assign a default value to next_key_state.
// That way, if none of the paths in the case statement apply, the variable will have a known value.
// This should be overridden by assignments below.

		next_key_state = key_state;
		
// Use the present state to determine the next state.

		case(key_state)
		
// This represents the state where the key is currently unpressed and was not just released:
// In this state, if the enable input button is down, make the next state KEY_PRESSED.

			KEY_FREE: begin
				if(enable_in == 1'b0)
					next_key_state = KEY_PRESSED;
			end
		
// This represents the state where the key is currently pressed:
// In this state, if the enable button is up, make the next state KEY_RELEASED.

			KEY_PRESSED: begin
				if(enable_in == 1'b1)
					next_key_state = KEY_RELEASED;
				end
			
// This represents the state where the key has just gone from being pressed to being released:
// In this state, make the next state KEY_FREE.
// This state transition is independent of the input value.
// This state lasts for exactly one clock cycle - the cycle after the button was released.

			KEY_RELEASED: next_key_state = KEY_FREE;
			
// If none of the above - something that should never happen - make the next state unknown.

			default: next_key_state = 2'bxx;
			
		endcase
		
	end

// OUTPUT MACHINE
// This always block represents combinational logic, so it uses blocking assignments.
// It is sensitized to changes in the present state and enable input.
// You should picture this block as a combinational circuit that operates on the state to determine the output.

// Enable_out is a Moore output, so the block is sensitized only to the key_state.
// In a Mealy machine, the output would be sensitized to a state and to an input.

// I have structured the output machine as an always block to provide an example of how you should do it in general. 
// The behavior of this output is simple enough that a continuous assignment could have been used more effectively.

	always@(key_state) begin
	
// To be safe, assign a default value to enable_out.
// That way, if none of the paths in the case statement apply, the variables will have a known value.
// This should be overridden by assignments below.

		enable_out = 1'b0;
		
// Use the present state to determine the output.

		case (key_state)
		
// If the key is currently unpressed and was not just released, enable_out should be 0.

			KEY_FREE: enable_out = 1'b0;
			
// If the key is currently being pressed, enable_out should be 0.

			KEY_PRESSED: enable_out = 1'b0;
			
// If the key has has just gone from being pressed to being released, enable out should be 1.
// Since this state only lasts for one clock cycle, enable_out is 1 for only one clock cycle.

			KEY_RELEASED: enable_out = 1'b1;
			
// If none of the above - something that should never happen - make the output unknown.

			default: enable_out = 1'bx;
			
		endcase
		
	end
	
endmodule
