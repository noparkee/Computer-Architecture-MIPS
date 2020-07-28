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

  wire        signext, shiftl16, memtoreg, memwrite_in, branch, brane, branch_2, brane_2, memread;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump;
  wire [2:0]  alucontrol;
	wire			jrcontrol, jalcontrol;
	wire [1:0]		aluop, aluop_1;
	wire [31:0]		instr_1, signimm_1;
	
  // Instantiate Controller
  controller c(
    .op         (instr_1[31:26]), 
		.funct      (signimm_1[5:0]), 
			.zero_1       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite_in),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.alucontrol (alucontrol),
			.jrcontrol	(jrcontrol),
			.jalcontrol	(jalcontrol),
			.memread	(memread),
			.branch_2	(branch_2),
			.brane_2	(brane_2),
			.branch		(branch),
			.brane		(brane),
			.aluop_1	(aluop_1),
			.aluop		(aluop));

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
		.zero_1       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout_1     (memaddr), 
    .writedata_3  (memwritedata),
    .readdata   (memreaddata),
		.jrcontrol	(jrcontrol),
		.jalcontrol	(jalcontrol),
		.memwrite	(memwrite_in),
		.memread	(memread),
		.aluop		(aluop),
		.aluop_1	(aluop_1),
		.branch		(branch),
		.brane		(brane),
		.branch_2	(branch_2),
		.brane_2	(brane_2),
		.instr_1		(instr_1),
		.signimm_1		(signimm_1),
		.memwrite_2		(memwrite));

endmodule

module controller(input  [5:0] op, funct,
					input        zero_1,
							input [1:0] aluop_1,
							input	branch_2, brane_2,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output [2:0] alucontrol,
							output	jrcontrol,
							output	jalcontrol,
							output [1:0]	aluop,
							output	branch, brane,
							output	memread);

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
		.jalcontrol	(jalcontrol),
		.memread	(memread));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop_1), 
    .alucontrol (alucontrol),
		.jrcontrol	(jrcontrol));

  assign pcsrc = (brane_2 ^ zero_1) & branch_2;

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
						output		jalcontrol,
						output		memread);

  reg [13:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop, brane, jalcontrol, memread} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 14'b00110000011000; // Rtype
      6'b100011: controls <= #`mydelay 14'b10101001000001; // LW
      6'b101011: controls <= #`mydelay 14'b10001010000000; // SW
      6'b000100: controls <= #`mydelay 14'b10000100001000; // BEQ
			6'b000101: controls <= #`mydelay 14'b10000100001100;	//	BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 14'b10101000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 14'b00101000010000; // ORI
      6'b001111: controls <= #`mydelay 14'b01101000000000; // LUI
      6'b000010: controls <= #`mydelay 14'b00000000100000; // J
			6'b000011: controls <= #`mydelay 14'b00100000100010; //	JAL
      default:   controls <= #`mydelay 14'bxx0xxxx00x00xx; // ???
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
						output        zero_1,
                output [31:0] pc,
                input  [31:0] instr,
					output [31:0] aluout_1, writedata_3,
                input  [31:0] readdata,
						input	jrcontrol,
						input	jalcontrol,
						input	memwrite, memread,
						input [1:0] aluop,
						output [1:0] aluop_1,
						input	branch, brane,
						output	branch_2, brane_2,
						output [31:0] instr_1,
						output memwrite_2,
						output [31:0] signimm_1);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch, pcbranch_1;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
	wire [31:0]	temp;
	wire [4:0]	tempwritereg;
	wire [31:0]	pcnextj;
  wire        shift;
  	
	
	wire [31:0]	srca_1, srca_2, srca_3;
	wire [31:0]	writedata, writedata_1, writedata_2, writedata_4;
	wire [31:0]	pcplus4_1, pcplus4_2;
	wire [31:0] aluout, aluout_2;
	wire [31:0]	readdata_1;
	wire [4:0]	rs_1, rt_1, rd_1;
	wire [4:0]	writereg_1, writereg_2;
	wire [10:0]	signal;
	
	wire	regwrite_1, regwrite_2, regwrite_3, memtoreg_1, memtoreg_2, memtoreg_3;
	wire	branch_1, brane_1, memread_1, memread_2, memwrite_1;
	wire	regdst_1, alusrc_1, shiftl16_1;
	wire	ifid_write, pcwrite, nopcontrol;
	wire	zero;
	wire [1:0]	forward_1, forward_2;
	wire	forward_3, forward_4;

// ##### Nokyung Park: Start #####
	
ifidm ifid(
	.clk	(clk),
	.reset	(reset),
	.ifid_write	(ifid_write),
	.instr_in	(instr),
	.pcplus4_in		(pcplus4),
	.instr_out		(instr_1),
	.pcplus4_out	(pcplus4_1));
		
idexm	idex(		
	.clk	(clk),
	.reset	(reset),
	.shiftl16_in	(signal[10]),
	.regwrite_in	(signal[9]),
	.memtoreg_in	(signal[8]),
	.branch_in	(signal[7]),
	.brane_in	(signal[6]),
	.memread_in	(signal[5]),
	.memwrite_in	(signal[4]),
	.regdst_in	(signal[3]),
	.aluop_in	(signal[2:1]),
	.alusrc_in	(signal[0]),
	.rd1_in		(srca_1),
	.rd2_in		(writedata_1),
	.pcplus4_in		(pcplus4_1),
	.signimm_in		(signimm),
	.rs_in		(instr_1[25:21]),
	.rt_in		(instr_1[20:16]),
	.rd_in		(instr_1[15:11]),
	.shiftl16_out	(shiftl16_1),
	.regdst_out		(regdst_1),
	.alusrc_out		(alusrc_1),
	.aluop_out		(aluop_1),
	.branch_out		(branch_1),
	.brane_out		(brane_1),
	.memread_out	(memread_1),
	.memwrite_out	(memwrite_1),
	.regwrite_out	(regwrite_1),
	.memtoreg_out	(memtoreg_1),
	.rd1_out	(srca_2),
	.rd2_out	(writedata_2),
	.pcplus4_out	(pcplus4_2),
	.signimm_out	(signimm_1),
	.rs_out			(rs_1),
	.rt_out			(rt_1),
	.rd_out			(rd_1));
		
exmemm	exmem(
	.clk	(clk),
	.reset	(reset),
	.regwrite_in	(regwrite_1),
	.memtoreg_in	(memtoreg_1),
	.branch_in		(branch_1),
	.brane_in		(brane_1),
	.memread_in		(memread_1),
	.memwrite_in	(memwrite_1),
	.zero_in		(zero),
	.pcbranch_in	(pcbranch),
	.aluout_in		(aluout),
	.writedata_in	(writedata_4),
	.writereg_in	(writereg),
	.regwrite_out	(regwrite_2),
	.memtoreg_out	(memtoreg_2),
	.branch_out		(branch_2),
	.brane_out		(brane_2),
	.memread_out	(memread_2),
	.memwrite_out	(memwrite_2),
	.zero_out		(zero_1),
	.pcbranch_out	(pcbranch_1),
	.aluout_out		(aluout_1),
	.writedata_out	(writedata_3),
	.writereg_out	(writereg_1));
	
memwbm	memwb(
	.clk	(clk),
	.reset	(reset),
	.regwrite_in	(regwrite_2),
	.memtoreg_in	(memtoreg_2),
	.aluout_in		(aluout_1),
	.readdata_in	(readdata),
	.writereg_in	(writereg_1),
	.regwrite_out	(regwrite_3),
	.memtoreg_out	(memtoreg_3),
	.aluout_out		(aluout_2),
	.readdata_out	(readdata_1),
	.writereg_out	(writereg_2));

forwarding_unit	forwarding (
	.rs_ex	(rs_1),
	.rt_ex	(rt_1),
	.rd_mem	(writereg_1),
	.rd_wb	(writereg_2),
	.rs_ifid	(instr_1[25:21]),
	.rt_ifid	(instr_1[20:16]),
	.regwrite_wb	(regwrite_3),
	.regwrite_mem	(regwrite_2),
	.forward_1		(forward_1),
	.forward_2		(forward_2),
	.forward_3		(forward_3),
	.forward_4		(forward_4));
	
hazzard_detection_unit hazzard_detection(
	.memread_ex		(memread_1),
	.rs_ifid	(instr_1[25:21]),
	.rt_ifid	(instr_1[20:16]),
	.rt_idex	(rt_1),
	.ifid_write		(ifid_write),
	.pcwrite		(pcwrite),
	.nopcontrol		(nopcontrol));
	
mux3 #(32) forward_mux_1 (
	.d0		(srca_2),
	.d1		(result),
	.d2		(aluout_1),
	.s		(forward_1),
	.y		(srca_3));
	
mux3 #(32) forward_mux_2 (
	.d0		(writedata_2),
	.d1		(result),
	.d2		(aluout_1),
	.s		(forward_2),
	.y		(writedata_4));
	
mux2 #(32) forward_mux_3 (
	.d0		(srca),
	.d1		(result),
	.s 		(forward_3),
	.y		(srca_1));
	
mux2 #(32) forward_mux_4 (
	.d0		(writedata),
	.d1		(result),
	.s		(forward_4),
	.y		(writedata_1));
	
mux2 #(11) nopmux(
	.d0		({shiftl16, regwrite, memtoreg, branch, brane, memread, memwrite, regdst, aluop, alusrc}),
	.d1		(0),
	.s		(nopcontrol),
	.y		(signal));
	
		
		
// ##### Nokyung Park: End #####


  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
		.en		(pcwrite),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm_1),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (pcplus4_2),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch_1),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr_1[25:0], 2'b00}),
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
    .we      (regwrite_3),
    .ra1     (instr_1[25:21]),
    .ra2     (instr_1[20:16]),
    .wa      (writereg_2),
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (rt_1),
    .d1  (rd_1),
    .s   (regdst_1),
    .y   (tempwritereg));

  mux2 #(32) resmux(
    .d0 (aluout_2),
    .d1 (readdata_1),
    .s  (memtoreg_3),
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
    .a       (instr_1[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm_1[31:0]),
    .shiftl16  (shiftl16_1),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata_4),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc_1),
    .y  (srcb));

  alu alu(
    .a       (srca_3),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
endmodule



// ##### Nokyung Park: Start #####


module ifidm (input		clk, reset, ifid_write,
			input [31:0] instr_in,
			input [31:0] pcplus4_in,
			output [31:0] instr_out,
			output [31:0] pcplus4_out);
			
 flopenr #(32) instrfl(
	.clk	(clk),
	.reset	(reset),
	.en		(ifid_write),
	.d		(instr_in),
	.q		(instr_out));
	
 flopenr #(32) pcplus4fl(
	.clk	(clk),
	.reset	(reset),
	.en		(ifid_write),
	.d		(pcplus4_in),
	.q		(pcplus4_out));
	
endmodule
				
		

module idexm (input		clk, reset,
			input		regdst_in, alusrc_in, shiftl16_in,
			input [1:0] aluop_in,	
			input		branch_in, brane_in, memread_in, memwrite_in,
			input		regwrite_in, memtoreg_in,
			input [31:0]	rd1_in, rd2_in,
			input [31:0]	pcplus4_in,
			input [31:0]	signimm_in,
			input [4:0] 	rs_in, rt_in, rd_in,
			
			output		regdst_out, alusrc_out, shiftl16_out,
			output [1:0] aluop_out,
			output		branch_out, brane_out, memread_out, memwrite_out,
			output		regwrite_out, memtoreg_out,
			output [31:0]	rd1_out, rd2_out,
			output [31:0]	pcplus4_out,
			output [31:0]	signimm_out,
			output [4:0]	rs_out, rt_out, rd_out);
		
wire [1:0] wb;
wire [3:0] m;
wire [4:0] ex;

assign	{regwrite_out, memtoreg_out} = wb;
assign	{branch_out, brane_out, memread_out, memwrite_out} = m;
assign	{regdst_out, aluop_out, alusrc_out, shiftl16_out} = ex;

 flopr #(2) wbfl(
	.clk	(clk),
	.reset	(reset),
	.d		({regwrite_in, memtoreg_in}),
	.q		(wb));

 flopr #(4) mfl(
	.clk	(clk),
	.reset	(reset),
	.d		({branch_in, brane_in, memread_in, memwrite_in}),
	.q		(m));
	
 flopr #(5) exfl(
	.clk	(clk),
	.reset	(reset),
	.d		({regdst_in, aluop_in, alusrc_in, shiftl16_in}),
	.q		(ex));

 flopr #(32) pcplus4fl(
	.clk	(clk),
	.reset	(reset),
	.d		(pcplus4_in),
	.q		(pcplus4_out));
	
 flopr #(32) signresultpl(
	.clk	(clk),
	.reset	(reset),
	.d		(signimm_in),
	.q		(signimm_out));
	
 flopr #(5) rsfl(
	.clk	(clk),
	.reset	(reset),
	.d		(rs_in),
	.q		(rs_out));
 
 flopr #(5) rtfl(
	.clk	(clk),
	.reset	(reset),
	.d		(rt_in),
	.q		(rt_out));
	
 flopr #(5) rdfl(
	.clk	(clk),
	.reset	(reset),
	.d		(rd_in),
	.q		(rd_out));

 flopr #(32) rd1fl(
	.clk	(clk),
	.reset	(reset),
	.d		(rd1_in),
	.q		(rd1_out));
	
 flopr #(32) rd2fl(
	.clk	(clk),
	.reset	(reset),
	.d		(rd2_in),
	.q		(rd2_out));
	
endmodule


module exmemm (input	clk,
				input	reset,
				input	regwrite_in, memtoreg_in,
				input	branch_in, brane_in, memread_in, memwrite_in,
				input	zero_in,
				input [31:0]	pcbranch_in,
				input [31:0]	aluout_in,
				input [31:0]	writedata_in,
				input [4:0] 	writereg_in,
				output	regwrite_out, memtoreg_out,
				output	branch_out, brane_out, memread_out, memwrite_out,
				output	zero_out,
				output [31:0]	pcbranch_out,
				output [31:0] 	aluout_out,
				output [31:0]	writedata_out,
				output [4:0]	writereg_out);
				
wire [1:0] wb;
wire [3:0] m;

assign	{regwrite_out, memtoreg_out} = wb;
assign	{branch_out, brane_out, memread_out, memwrite_out} = m;

flopr #(2) wbfl(
	.clk	(clk),
	.reset	(reset),
	.d		({regwrite_in, memtoreg_in}),
	.q		(wb));
	
flopr #(4) mfl(
	.clk	(clk),
	.reset	(reset),
	.d		({branch_in, brane_in, memread_in, memwrite_in}),
	.q		(m));
	
flopr #(1) zerofl(
	.clk	(clk),
	.reset	(reset),
	.d		(zero_in),
	.q		(zero_out));
	
flopr #(32) pcbranchfl(
	.clk	(clk),
	.reset	(reset),
	.d		(pcbranch_in),
	.q		(pcbranch_out));

flopr #(32) aluoutfl(
	.clk	(clk),
	.reset	(reset),
	.d		(aluout_in),
	.q		(aluout_out));

flopr #(32)	writedatafl(
	.clk	(clk),
	.reset	(reset),
	.d		(writedata_in),
	.q		(writedata_out));
	
flopr #(5) writeregfl(
	.clk	(clk),
	.reset	(reset),
	.d		(writereg_in),
	.q		(writereg_out));
	
endmodule


module memwbm (input	clk,
				input	reset,
				input	regwrite_in, memtoreg_in,
				input [31:0]	aluout_in,
				input [31:0]	readdata_in,
				input [4:0] 	writereg_in,
				output	regwrite_out, memtoreg_out,
				output [31:0]	aluout_out,
				output [31:0]	readdata_out,
				output [4:0]	writereg_out);
				
wire [1:0] wb;

assign	{regwrite_out, memtoreg_out} = wb;
	
 flopr #(2) wbfl(
	.clk	(clk),
	.reset	(reset),
	.d		({regwrite_in, memtoreg_in}),
	.q		(wb));

 flopr #(32) aluoutfl(
	.clk	(clk),
	.reset	(reset),
	.d 		(aluout_in),
	.q		(aluout_out));
	
 flopr #(32) readdatafl(
	.clk	(clk),
	.reset	(reset),
	.d		(readdata_in),
	.q		(readdata_out));

 flopr #(5)	writeregfl(
	.clk	(clk),
	.reset	(reset),
	.d 		(writereg_in),
	.q		(writereg_out));

endmodule

module	hazzard_detection_unit (input		memread_ex,
								input [4:0]	rs_ifid, rt_ifid, rt_idex,
								output reg ifid_write, pcwrite, nopcontrol);
								
always @(*) begin
	if(((rt_idex == rs_ifid) || (rt_idex == rt_ifid)) && memread_ex)
		begin
			ifid_write <= 0;
			pcwrite <= 0;
			nopcontrol <= 1;
		end
		else begin
				ifid_write <= 1;
				pcwrite <= 1;
				nopcontrol <= 0;
			end
		end
endmodule

module forwarding_unit	(input [4:0]	rs_ex, rt_ex,
						input [4:0]		rd_mem, rd_wb,
						input [4:0]		rs_ifid, rt_ifid,
						input	regwrite_mem, regwrite_wb,
						output reg [1:0] 	forward_1, forward_2,
						output reg forward_3, forward_4);
always @(*) begin
	if(regwrite_mem && (rd_mem != 0) && (rd_mem == rs_ex)) forward_1 <=2'b10;
	else if(regwrite_wb && (rd_wb != 0) && (rd_wb == rs_ex) &&  (~(regwrite_mem && (rd_mem != 0) && (rd_mem == rs_ex)))) forward_1 <= 2'b01;
	else forward_1 <= 2'b00;
	
	if(regwrite_mem && (rd_mem != 0) && (rd_mem == rt_ex)) forward_2 <= 2'b10;
	else if(regwrite_wb && (rd_wb != 0) && (rd_wb == rt_ex) && (~(regwrite_mem && (rd_mem != 0) && (rd_mem == rt_ex)))) forward_2 <= 2'b01;
	else forward_2 <= 2'b00;
	
	if(regwrite_wb && (rd_wb != 0) && (rd_wb == rs_ifid)) forward_3 <= 1'b1;
	else forward_3 <= 1'b0;
	
	if(regwrite_wb && (rd_wb != 0) && (rd_wb == rt_ifid)) forward_4 <= 1'b1;
	else forward_4 <= 1'b0;
	
	end
endmodule

// ##### Nokyung Park: End #####