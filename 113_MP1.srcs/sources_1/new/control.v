//====================================================================================
//  Machine Problem 1: 
//  Single Cycle RISC V Processor
//
//  Control
//
//  John Rufino I. Macasaet
//
//====================================================================================
//      Control block is asynchronous/combinational.

`timescale 1ns / 1ps

module Control 
#(  
    parameter   ALUOP_WIDTH     =  5,
    parameter   OPCODE_WIDTH    =  7,
    parameter   FUNCT3_WIDTH    =  3,
    parameter   FUNCT7_WIDTH    =  7,
    parameter   WMASK_WIDTH     =  8,
    parameter   PCSRC_WIDTH     =  2,
    parameter   REGWRSRC_WIDTH  =  2,
    //==============================
    //  OPCODES
    //==============================
    parameter   opcode_ld       = 7'b0000011,
    parameter   opcode_addi     = 7'b0010011,
    parameter   opcode_jalr     = 7'b1100111,
    parameter   opcode_sd       = 7'b0100011,
    parameter   opcode_rtype    = 7'b0110011,
    parameter   opcode_sbtype   = 7'b1100011,
    parameter   opcode_jal      = 7'b1101111,
    //==============================
    //  FUNCT3
    //==============================
    parameter   funct3_ld       = 3'b011,
    parameter   funct3_sd       = 3'b011,
    parameter   funct3_addsub   = 3'b000,
    parameter   funct3_and      = 3'b111,
    parameter   funct3_or       = 3'b110,
    parameter   funct3_xor      = 3'b100,
    parameter   funct3_slt      = 3'b010,
    parameter   funct3_beq      = 3'b000,
    parameter   funct3_bne      = 3'b001,
    //==============================
    //  FUNCT7
    //==============================
    parameter   funct7_add      = 7'd0,
    parameter   funct7_sub      = 7'b0100000,
    //===============================
    //  ALU Opcodes
    //===============================
    parameter   OP_AND        =   0,
    parameter   OP_OR         =   1,
    parameter   OP_ADD        =   2,
    parameter   OP_XOR        =   3,
    parameter   OP_SUB        =   6,
    parameter   OP_SLT        =   7,
    parameter   OP_ISEQ       =   10
)
(
    //==============================
    //  Control Inputs
    //==============================
    input   [OPCODE_WIDTH-1:0]      opcode,
    input   [FUNCT3_WIDTH-1:0]      funct3,
    input   [FUNCT7_WIDTH-1:0]      funct7,
    //==============================
    //  Control Outputs
    //==============================
    output reg                     Branch,
    output reg                     MemRead,
    output reg [PCSRC_WIDTH-1:0]   PCSrc,
    output reg [REGWRSRC_WIDTH-1:0]RegWrSrc,
    output reg [ALUOP_WIDTH-1:0]   ALUOp,
    output reg                     ALUSrc,
    output reg                     RegWrite,
    //==============================
    //  Processor Outputs
    //==============================
    output reg                     wr_en,
    output reg [WMASK_WIDTH-1:0]   wmask
);

always@(*) begin
    Branch      = 0;
    MemRead     = 0;
    PCSrc       = 0;
    RegWrSrc    = 0;
    ALUOp       = 64;
    ALUSrc      = 0;
    RegWrite    = 0;
    wr_en       = 0;
    wmask       = 8'hFF;
    
    case(opcode)
        //===============================================================================
        // I-TYPE
        //===============================================================================
        opcode_ld: begin
            if(funct3==funct3_ld) begin
                RegWrite    = 1'b1;     // Write to Register File
                RegWrSrc    = 2'd1;     // Choose rdata input to write to Register File
                ALUOp       = OP_ADD;   // Add op
                ALUSrc      = 1'b1;     // Choose i-type immediate as ALU inB
            end
        end
        
        opcode_addi: begin
            if(funct3==0) begin
                RegWrite    = 1'b1;     // Write to Register File
                RegWrSrc    = 2'd0;     // Choose ALURes to write to Register File
                ALUOp       = OP_ADD;   // Add ALU OP
                ALUSrc      = 1'b1;     // Choose Immediate as ALU inB
            end
        end
        
        opcode_jalr: begin
            if(funct3==0) begin
                RegWrite    = 1'b1;     // Write to Register File
                RegWrSrc    = 2'd2;     // Choose PC+4 to write to Register File
                ALUOp       = OP_ADD;   // Add ALU OP
                ALUSrc      = 1'b1;     // Choose Immediate as ALU inB
                PCSrc       = 2'd2;     // Choose ALURes to write to PC
            end
        end
        
        //===============================================================================
        // S-TYPE
        //===============================================================================
        
        opcode_sd: begin
            if(funct3==funct3_sd) begin
                ALUSrc      = 1'b1;     // Choose Immediate as ALU inB
                ALUOp       = OP_ADD;   // Add ALU OP
                wr_en       = 1'b1;     // Enable write to Data Memory
            end
        end
        
        //===============================================================================
        // R-TYPE
        //===============================================================================
        opcode_rtype: begin  
            RegWrite    = 1'b1;         // Write to Register File         
            case(funct3) 
                funct3_addsub: begin 
                    if(funct7==funct7_add)    
                        ALUOp = OP_ADD;
                    else if(funct7==funct7_sub)
                        ALUOp = OP_SUB;
                end
                
                funct3_and: begin
                        ALUOp = OP_AND; 
                end
                
                funct3_or: begin 
                        ALUOp = OP_OR; 
                end
                
                funct3_xor: begin 
                        ALUOp = OP_XOR; 
                end
                
                funct3_slt: begin 
                        ALUOp = OP_SLT; 
                end
            endcase
        end
        
        //===============================================================================
        // SB-TYPE
        //===============================================================================
        opcode_sbtype: begin
            Branch = 1'b1;
            if(funct3==funct3_beq) begin
                ALUOp = OP_XOR;
            end
            else if(funct3==funct3_bne) begin
                ALUOp = OP_ISEQ;
            end
        end
        
        //===============================================================================
        // J-TYPE
        //===============================================================================
        opcode_jal: begin
            RegWrSrc = 2'd2;
            RegWrite = 1'b1;
            PCSrc    = 1'b1;
        end
        
        default: begin
            Branch      = 0;
            MemRead     = 0;
            PCSrc       = 0;
            RegWrSrc    = 0;
            ALUOp       = 64;
            ALUSrc      = 0;
            RegWrite    = 0;
            wr_en       = 0;
            wmask       = 0;
        end
    endcase
end

endmodule