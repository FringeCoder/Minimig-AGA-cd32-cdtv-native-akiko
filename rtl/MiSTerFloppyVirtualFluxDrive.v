//////////////////////////////////////////////////////////////////////////////
// MiSTer Floppy Virtual Floppy Drive                                       //
// Copyright (C) RobSmithDev 2022-2025                                      //
// https://mister.robsmithdev.co.uk                                         //
//////////////////////////////////////////////////////////////////////////////

/*

This simulates a virtual Amiga DD floppy drive, like the main part of the core, except this works at a flux level
It preteneds to be the read-data line of a floppy drive and provides no synchronisation at all, which should be perfect.
The output should be fed into MiSTerFloppyPLL 

Data format is WORDS, each BYTE in the word is: 
	0=INDEX
	1=Simple delay of 1/7mhz
	2=Disable actual flux transition on next byte
	             (DISABLED and NOT IN USE) 3=Enable weak-bit generator for duration specified in next byte
	>2 Time until flux transition at 7mhz clock (ie: 7=1us). 
	


*/

module MiSTerFloppyVirtualFluxDrive (
	input clk,   
	input clk7_en,
	input reset,
	input enabled,

	input [3:0] drivesSelect,   				  // Selected when set to 0, hence the 'n'	
	input [1:0] driveSelected,					  // drive NUMBER selected
	
	input nMotorEnabled,							  // If the motor is enabled
	output o_nReady,  							  // If the motor is at speed - ~500ms
	output oRequestData,						     // Set to '1' when the virtual drive wants data to be pumped in ~450ms
		
	output floppyBit,             			  // Pretends to be the floppy drive read head, so no flags are needed
	
	input fifo_empty,								  // If the core FIFO is empty
	input fifo_reset,								  // set to 1 if the fifo was reset
	input  [15:0] fluxDataIn,                // Flux data received from the core FIFO
	output fluxDataRead,							  // Set when the flux data was read
	output _Index									  // Index pulse	
);


reg[21:0] motorTimer[3:0];

reg[3:0] mtrReady;
reg[3:0] nDriveLatched;
reg[3:0] delayDrivesSelect;
reg[7:0] ticksUntilNextFlux;    // Ticks until next flux
reg[7:0] nextFluxByte;
reg usingNextByte;
reg triggerFlux;

reg _o_nReady;
assign o_nReady = _o_nReady;
reg oRequestDataReg;
assign oRequestData = oRequestDataReg;

// This should remain LOW for around 400ns +/- 20% (320-480ns, 2.24-3.36 ticks, we'll go with 3). It's detected on falling edge, this is just for completeness
reg[1:0] floppyDriveBitCounter;
assign floppyBit = floppyDriveBitCounter == 2'h3;

reg fluxDataReadOut = 0;
assign fluxDataRead = fluxDataReadOut;

// the index pulse can last around 1-8ms! Theres 7 clock ticks below in 1us, so, 1000us is 7000 clock ticks
reg[12:0] indexCounter;
assign _Index = indexCounter == 13'h1FFF;

/*
// Used for simulating weak-no flux areas
reg [7:0] randomNumber;
wire [5:0] s_raw  = randomNumber[5:0];                    // 0..63
wire [5:0] s_clip = (s_raw <= 6'd50) ? s_raw : {1'b0, s_raw[4:0]};

reg[6:0] lastNoise;
reg triggerWeakBits;
wire weakBitGeneratorReady = lastNoise[6];   // ~9.14 µs
*/


// This is for double density only. 
always@(posedge clk) begin
	if (clk7_en) begin  // 14 clocks is 2uS
/*	
		// Advance the random number generator used by the random flux noise thing
		randomNumber <= {randomNumber[6:0], randomNumber[7] ^ randomNumber[5] ^ randomNumber[4] ^ randomNumber[3]};
*/
		delayDrivesSelect <= drivesSelect;

		if (reset) begin	
			integer id;
			for (id = 0; id<4; id=id+1) begin	
				mtrReady[id] <= 0;
				nDriveLatched[id] <= 0;
				motorTimer[id] <= 22'b0;
			end
			fluxDataReadOut <= 0;
			_o_nReady <= 1;			
			ticksUntilNextFlux <= 0;
			nextFluxByte <= 8'h1C;   // kind of a 4us pulse
			indexCounter <= 13'h1FFF;
			floppyDriveBitCounter <= 2'h3;
			usingNextByte <= 1;
			triggerFlux <= 1;
/*			
			randomNumber <= 8'h1;
			triggerWeakBits <= 0;
*/			
		end else begin							
			integer id;
			// This isn't quite right, as driveId is faster than 7mhz, but it works for what we need.
			for (id = 0; id<4; id=id+1) begin
				if (drivesSelect[id] && ~delayDrivesSelect[id]) nDriveLatched[id] <= nMotorEnabled;	
			
				// Ready is a little more complex as we have to simulate it.  It's HIGH until ready
				if (~nDriveLatched[id]) begin
					if (motorTimer[id] != 3_500_000) begin    // that's 500ms, standard spinup time
						motorTimer[id] <= motorTimer[id] + 22'h1;
						mtrReady[id] <= 1'b1;
						//triggerWeakBits <= 0;
						//lastNoise <= 7'h0;
					end else begin 
						mtrReady[id] <= ~drivesSelect[id]; 
					end
				end else
				begin 
					mtrReady[id] <= ~drivesSelect[id];  
					motorTimer[id] <= 2'd0; 
				end
			end	
			
			// Handle a fifo reset
			if (fifo_reset) begin
				fluxDataReadOut <= 0;
				ticksUntilNextFlux <= 0;
				nextFluxByte <= 8'h1C;   // kind of a 4us pulse
				indexCounter <= 13'h1FFF;
				floppyDriveBitCounter <= 2'h3;
				usingNextByte <= 1;
/*				
				triggerWeakBits <= 0;
*/				
			end
						
			// which drive is selected?
			_o_nReady <= (enabled & drivesSelect[driveSelected]) ? mtrReady[driveSelected] : 1'b1;
			// Special flag if this is ready to start receiving data (~50ms before READY is set)
			oRequestDataReg <= enabled & drivesSelect[driveSelected] & ~nDriveLatched[driveSelected] & (motorTimer[driveSelected][21:20]==2'b11);
					
			// Index counter!
			if (!_Index) indexCounter <= indexCounter + 13'h1;
			fluxDataReadOut <= 1'b0;
			
			if (~floppyBit) floppyDriveBitCounter <= floppyDriveBitCounter + 2'h1;	
			
			
			// This isn't quite right, as data should always be ticking, but typically the "READ" data is HIGH until the drive is ready
			if (~_o_nReady & ~nDriveLatched[driveSelected]) begin				
				// Track how long it's been since a flux transition occured
				//if (~weakBitGeneratorReady) lastNoise <= lastNoise + 7'h1;

				if (ticksUntilNextFlux == 0) begin
					triggerFlux <= 1;									// Future transitions should trigger flux events unless overridden	
					if (usingNextByte) begin					
						if (~fifo_empty) begin			
							ticksUntilNextFlux <= fluxDataIn[7:0];
							nextFluxByte <= fluxDataIn[15:8];
							fluxDataReadOut <= 1'b1;
							usingNextByte <= 0;							
							case (fluxDataIn[7:0])
								8'd0: indexCounter <= 0;  			// trigger index marker
								8'd1: ticksUntilNextFlux <= 0; 	// Delay by 1 clock
								8'd2: begin
											triggerFlux <= 0;   			// Next timing, DON'T trigger a flux transition, its just a delay
											ticksUntilNextFlux <= 0;
									end
/*									
								8'd3: begin
											triggerWeakBits <= 1;      // Next timing is how long to trigger weak bits for
											ticksUntilNextFlux <= 0;
											lastNoise <= {2'b010, randomNumber[5:1]};
										end	
*/										
								default: begin
												if (triggerFlux/* & ~triggerWeakBits*/) floppyDriveBitCounter <= 2'h0;
												//triggerWeakBits <= 0;
											end
							endcase
						end else
						begin
							ticksUntilNextFlux <= 8'hFF;  // shouldn't happen
						end
					end else
					begin
						ticksUntilNextFlux <= nextFluxByte;						
						case (nextFluxByte)
								8'd0: indexCounter <= 0;  			// trigger index marker
								8'd1: ticksUntilNextFlux <= 0; 	// Delay by 1 clock
								8'd2: begin
											triggerFlux <= 0;   			// Next timing, DON'T trigger a flux transition, its just a delay
											ticksUntilNextFlux <= 0;
									end
/*									
								8'd3: begin
											triggerWeakBits <= 1;      // Next timing is how long to trigger weak bits for
											ticksUntilNextFlux <= 0;
											lastNoise <= {2'b010, randomNumber[5:1]};											
									end	
*/									
								default: begin
												if (triggerFlux/* & ~triggerWeakBits*/) floppyDriveBitCounter <= 2'h0;													
												//triggerWeakBits <= 0;
											end
						endcase
						usingNextByte <= 1;				
					end
				end else
				begin
					ticksUntilNextFlux <= ticksUntilNextFlux - 8'h01;
/*					
					// No flux for ~150us. Start to produce noise/weak data 
					if (triggerWeakBits & weakBitGeneratorReady) begin
						if (randomNumber[0]) begin
							// Simulate fake pulse
							floppyDriveBitCounter <= 2'h0;
							lastNoise <= {1'b0, s_clip};
						end						
					end
*/					
				end
			end
		end
	end
end



endmodule