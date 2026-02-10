//////////////////////////////////////////////////////////////////////////////
// MiSTer Floppy                                                            //
// Copyright (C) RobSmithDev 2022-2026                                      //
// https://mister.robsmithdev.co.uk                                         //
//////////////////////////////////////////////////////////////////////////////
//
// This module provides an IBM compatiable interface to the MiSTer Floppy board
// This module will automatically convert internally the signals
// depending on what type of drive is actually connected.
// THIS MODULE IS UNTESTED 
/*

module MiSTerFloppyIBM(
	input i_core_cpu_clk,
	
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,	
	
	input i_nWriteData,
	input i_nWriteGate,
	input i_nHeadSelect,
	output o_nReadData,
	output o_nIndex,
	
	output o_nTrk00,
	output o_nWriteProtected,
	output o_nDiskChange,
	
	input i_nDriveSelectA,
	input i_nDriveSelectB,
	input i_nMotorEnableA,
	input i_nMotorEnableB,
	
	input i_nDir,
	input i_nStep,
	
	input i_reset,
	
	output o_error,
	output o_detected,
	output o_PinIBMDrive
);



parameter CLK_Freq = 50_000_000;	//	50 MHz

	wire o_nPin34;
	wire o_nPin2;
	reg i_nPin12;
	reg i_nPin14;
	reg i_nPin16;
	reg i_nPin10;
	reg i_nPin6;
	reg i_nMTR123;
	
	reg _o_nDiskChange;
	assign o_nDiskChange = _o_nDiskChange;
	
	MiSTerFloppyRAWIO #(CLK_Freq) dbRaw(	
		.i_core_cpu_clk(i_core_cpu_clk),
		
		.USER_IN(USER_IN),
		.USER_OUT(USER_OUT),
		
		.i_nWriteData(i_nWriteData),
		.i_nWriteGate(i_nWriteGate),
		.i_nHeadSelect(i_nHeadSelect),
		.o_nReadData(o_nReadData),
		.o_nIndex(o_nIndex),
	
		.o_nTrk00(o_nTrk00),
		.o_nWriteProtected(o_nWriteProtected),
		.o_nPin34(o_nPin34),
		.o_nPin2(o_nPin2),

		.i_nPin12(i_nPin12),
		.i_nPin14(i_nPin14),
		.i_nPin16(i_nPin16),		
		.i_nDir(i_nDir),
		.i_nStep(i_nStep),
		.i_nPin10(i_nPin10),
		.i_nPin6(i_nPin6),

		.i_nPin4InUse(i_nMotorEnableA),		
		.i_nMTR123(i_nMotorEnableB),
		.i_reset(i_reset),
		.o_error(o_error),

		.o_PinIBMDrive(o_PinIBMDrive),
		.o_detected(o_detected),		
		);
		
			
	always@(posedge i_core_cpu_clk)begin
		if (i_reset) begin
			i_nPin12 <= 1;
			i_nPin14 <= 1;
			i_nPin16 <= 1;
			i_nPin10 <= 1;
			i_nPin6 <= 1;
			i_nMTR123 <= 1;
		end
		
		if (o_PinIBMDrive) begin
			// Connected drive is actually an IBM drive
			i_nPin14 <= i_nDriveSelectA;
			i_nPin12 <= i_nDriveSelectB;
			i_nPin10 <= i_nMotorEnableA;
			i_nPin16 <= i_nMotorEnableB;	
			_o_nDiskChange <= o_nPin34;
			i_nMTR123 <= i_nMotorEnableB;
		end else begin
		   // Connected drive is actually an SHUGART drive
			i_nPin10 <= i_nDriveSelectA;
			i_nPin12 <= i_nDriveSelectB;
			_o_nDiskChange <= o_nPin2;
			i_nPin14 <= 1;    // device select 2
			i_nPin6 <= 1;     // device select 3
			i_nPin16 <= i_nMotorEnableA;  // mtron
			i_nMTR123 <= i_nMotorEnableB;
		end
	end
		
endmodule

*/