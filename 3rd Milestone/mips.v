`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump;
  wire [2:0]  alucontrol;
	wire			brane;
	wire			jrcontrol, jalcontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
		.funct      (instr[5:0]), 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.alucontrol (alucontrol),
			.jrcontrol	(jrcontrol),
			.jalcontrol	(jalcontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata),
		.jrcontrol	(jrcontrol),
		.jalcontrol	(jalcontrol));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output [2:0] alucontrol,
							output	jrcontrol,
							output	jalcontrol);

  wire [1:0] aluop;
  wire       branch, brane;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop),
		.brane	(brane),
		.jalcontrol	(jalcontrol));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol),
		.jrcontrol	(jrcontrol));

  assign pcsrc = (brane ^ zero) & branch;

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output [1:0] aluop,
						output		brane,
						output		jalcontrol);

  reg [12:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop, brane, jalcontrol} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0011000001100; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100000; // LW
      6'b101011: controls <= #`mydelay 13'b1000101000000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000100; // BEQ
			6'b000101: controls <= #`mydelay 13'b1000010000110;	//	BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0010100001000; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000010000; // J
			6'b000011: controls <= #`mydelay 13'b0010000010001; //	JAL
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output 	 [2:0] alucontrol,
					output	jrcontrol);
	
	reg [3:0] controls;
  
  assign {alucontrol, jrcontrol} = controls;
	
  always @(*)
    case(aluop)
      2'b00: controls <= #`mydelay 4'b0100;  // add
      2'b01: controls <= #`mydelay 4'b1100;  // sub
      2'b10: controls <= #`mydelay 4'b0010;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: controls <= #`mydelay 4'b0100; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: controls <= #`mydelay 4'b1100; // SUB, SUBU: only difference is exception
          6'b100100: controls <= #`mydelay 4'b0000; // AND
          6'b100101: controls <= #`mydelay 4'b0010; // OR
			 6'b101010: controls <= #`mydelay 4'b1110; // SLT
				6'b101011: controls <= #`mydelay 4'b1110;		//sltu
				6'b001000: controls <= #`mydelay 4'b0001;		//	jr
          default:   controls <= #`mydelay 4'bxxx0; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
                input  [2:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata,
						input	jrcontrol,
						input	jalcontrol);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
	wire [31:0]	temp;
	wire [4:0]	tempwritereg;
	wire [31:0]	pcnextj;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s    (jump),
    .y    (pcnextj));

	 mux2 #(32)	jrmux(
	 .d0	(pcnextj),
	 .d1	(srca),
	 .s	(jrcontrol),
	 .y	(pcnext));
	 
  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (regdst),
    .y   (tempwritereg));

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
    .y  (temp));
	 
	 mux2 #(5) rawrmux(
	 .d0	(tempwritereg),
	 .d1	(5'b11111),
	 .s	(jalcontrol),
	 .y	(writereg));
	 
	 mux2 #(32) raresmux(
	 .d0	(temp),
	 .d1	(pcplus4),
	 .s	(jalcontrol),
	 .y	(result));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
endmodule
