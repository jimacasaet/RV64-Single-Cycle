///////////////////////////////////////////////////////////////////////////////////////
//  Machine Problem 1: 
//  Single Cycle RISC V Processor
//
//  John Rufino I. Macasaet
///////////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

//====================================================================================
//
//  Top module
//
//====================================================================================

/*
--       ---------------------------        -----
P|       |        PROCESSOR        |        |
R|       |                         |        | D
O|       |                         |        | A
G| <--/--| pc               addr   | --/--> | T
R|   32  |                         |  32    | A
A|       |                         |        |
M| --/-->| inst            wr_en   | -----> | M
 |   32  |                         |        | E
M|       |                         |        | M
E|       |                 wdata   | --/--> | O
M|       |                         |  64    | R
O|       |                         |        | Y
R|       |                 wmask   | --/--> |
Y|       |                         |   8    |
--       |                         |        |
         |                 rdata   | <--/-- |
         |                         |   64   |
         |                         |        |
         ---------------------------        ------
         
        ^ LAYOUT OF THIS IMPLEMENTATION OF THE SINGLE CYCLE RISCV PROCESSOR
        
        --  pc: - provides address to the instruction memory
                - outputs the full 32 bit addresses 
                - truncate lower 2 bits when connecting to program memory
                  e.g. connect only pc[31:2] to address port of program memory 
                  so that addresses are word aligned
        
        --  addr: - provides address to the data memory
                  - outputs the full 32 bit addresses 
                  - truncate lower 3 bits when connecting to data memory
                    e.g. connect only pc[31:3] to address port of data memory 
                    so that addresses are doubleword aligned
*/

module processor #(
    //==============================
    //  Processor/Interface
    //==============================
    parameter PC_WIDTH    = 32,
    parameter INST_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter WDATA_WIDTH = 64,
    parameter WMASK_WIDTH = 8,
    parameter RDATA_WIDTH = 64,
    parameter DATA_WIDTH  = 64,
    //==============================
    //  Control
    //==============================
    parameter ALUOP_WIDTH = 5,
    parameter FUNCT3_WIDTH= 3,
    parameter FUNCT7_WIDTH= 7,
    parameter OPCODE_WIDTH= 7
)(
    //==============================
    //  Clocks and resets
    //==============================
    input                       clk,
    input                       nrst,
    //==============================
    //  Program Memory
    //==============================
    output  [PC_WIDTH-1:0]      pc,
    input   [INST_WIDTH-1:0]    inst,
    //==============================
    //  Data Memory
    //==============================
    output reg [ADDR_WIDTH-1:0] addr,
    output                      wr_en,
    output  [WDATA_WIDTH-1:0]   wdata,
    output  [WMASK_WIDTH-1:0]   wmask,
    input   [RDATA_WIDTH-1:0]   rdata
);
    //==============================
    // Wire declarations
    //==============================
    
    //  RegFile to ALU/MUX Wires
    wire    [RDATA_WIDTH-1:0]   rd1_to_inA;     // read data 1 to ALU IN A
    wire    [RDATA_WIDTH-1:0]   rd2_out;        // read data 2 to ALU MUX OR DataMemory
    wire    [RDATA_WIDTH-1:0]   mux_to_inB;     // ALU MUX out to ALU IN B 
    wire    [RDATA_WIDTH-1:0]   imm;            // Immediate
      
    // MUX (ALURes/Rdata/PC+4) to RegFile
    wire    [RDATA_WIDTH-1:0]   mux3_to_RegFile; // ALU/DataMem MUX to RegFile
    
    // ALU
    wire    [DATA_WIDTH-1:0]    ALURes;
    wire                        zero;
    
    // Control Input Wires
    wire    [OPCODE_WIDTH-1:0]  opcode;
    wire    [FUNCT3_WIDTH-1:0]  funct3;
    wire    [FUNCT7_WIDTH-1:0]  funct7;
    
    // Control Output Wires
    wire                        Branch, MemRead, MemToReg;
    wire    [ALUOP_WIDTH-1:0]   ALUOp;
    wire                        ALUSrc, RegWrite;
    wire    [1:0]               RegWrSrc, PCSrc;
    
    // PC Adder/MUX Wires
    wire    [PC_WIDTH-1:0]      adder0_res;     // Adder 0 Result (PC + 4)
    wire    [PC_WIDTH-1:0]      adder1_res;     // Adder 1 Result (PC + Branch Immediate)
    wire    [PC_WIDTH-1:0]      adder2_res;     // Adder 2 Result (PC + JAL Immediate)
    wire    [PC_WIDTH-1:0]      branch_res;     // PC+4 or Branch MUX Result
    wire    [PC_WIDTH-1:0]      branch_imm;     // Sign-extended immediate for branch addr compute
    wire    [PC_WIDTH-1:0]      mux_to_PC;      // MUX output to PC
    wire    [PC_WIDTH-1:0]      const_four;     // Constant 4
    wire    [PC_WIDTH-1:0]      JAL_imm;        // Immediate for JAL to PCSrc
    wire    [PC_WIDTH-1:0]      pc_out;
    
    wire                        branch_sel;     // Select wire for MUX (branch or PC+4)

    
    //==============================
    //  Register File
    //==============================
    RegisterFile I_RegisterFile(
        .clk(clk), .nrst(nrst),
        .reg_wr_en(RegWrite), .reg_wr_data(mux3_to_RegFile),
        .reg_read_addr1(inst[19:15]), .reg_read_addr2(inst[24:20]),
        .reg_read_data1(rd1_to_inA), .reg_read_data2(rd2_out),
        .reg_wr_dest(inst[11:7])
    );
    
    //==============================
    // Immediate Generator
    //==============================
    ImmGen I_IMMGEN(
        .inst(inst),
        .imm(imm)
    );
    
    //==============================
    // MUX (RegFile/Immediate to ALU)
    //==============================
    MUX I_MUX_REG_TO_ALU(
        .in0(rd2_out), 
        .in1(imm), 
        .sel(ALUSrc), 
        .out(mux_to_inB)
    );
    
    //==============================
    //  ALU
    //==============================
    ALU I_ALU(
        .ALUop(ALUOp), 
        .inA(rd1_to_inA), .inB(mux_to_inB),
        .zero(zero), .result(ALURes)
    );
    
    //==============================
    //  MUX3 (ALURes/Rdata/PC+4 To RegFile)
    //==============================
    MUX3 I_MUX3_REGWR(
        .in0(ALURes),
        .in1(rdata),
        .in2({ {PC_WIDTH{adder0_res[PC_WIDTH-1]}}, adder0_res}), // sign extend to 64 bits
        .sel(RegWrSrc),
        .out(mux3_to_RegFile)
    );
    
    //==============================
    // ADDER 0
    //==============================
    
    // Set constant to 4
    assign const_four = 32'd4;
    
    ADD I_ADD_ADDER0(
        .A(pc_out),
        .B(const_four),
        .Sum(adder0_res)
    );
    
    //==============================
    // ADDER 1
    //==============================
    
    // Sign-extended immediate for branch address computation
    // (Must be 32 bits)
    assign branch_imm = { {19{inst[31]}} ,inst[31],inst[7],inst[30:25],inst[11:8],1'b0};
    
    ADD I_ADD_ADDER1(
        .A(pc_out),
        .B(branch_imm),
        .Sum(adder1_res)
    );
    
    //==============================
    // MUX (PC+4 or Branch)
    //==============================
    assign branch_sel = zero & Branch;
    
    MUX #(PC_WIDTH) I_MUX_PC4_BRANCH(
        .in0(adder0_res), 
        .in1(adder1_res), 
        .sel(branch_sel), 
        .out(branch_res)
    );
      
    //==============================
    // ADDER 2
    //==============================
    
    // JAL Immediate field
    // sign-extended, 32 bits
    assign JAL_imm = { {11{inst[31]}} ,inst[31],inst[19:12],inst[20],inst[30:21],1'b0} ;
    
    ADD I_ADD_ADDER2(
        .A(pc_out),
        .B(JAL_imm),
        .Sum(adder2_res)
    );     
    
    //==============================
    // MUX3 (To PC)
    //==============================
    MUX3 #(PC_WIDTH) I_MUX3_TO_PC (
        .in0(branch_res),
        .in1(adder2_res),
        .in2(ALURes[PC_WIDTH-1:0]),
        .sel(PCSrc),
        .out(mux_to_PC)
    );
    //==============================
    //  ProgramCounter
    //==============================
    PC I_PC(
        .clk(clk), .nrst(nrst),
        .next(mux_to_PC), 
        .current(pc_out)
    );
    
    //==============================
    //  Control Input Wires
    //==============================
    assign  opcode = inst[6:0];
    assign  funct3 = inst[14:12];
    assign  funct7 = inst[31:25];
    
    //==============================
    //  Control
    //==============================
    Control I_Control(
        // control inputs
        .opcode(opcode), .funct3(funct3), .funct7(funct7),
        
        // control outputs
        .Branch(Branch), .MemRead(MemRead),
        .PCSrc(PCSrc), .RegWrSrc(RegWrSrc),
        .ALUOp(ALUOp), .ALUSrc(ALUSrc), .RegWrite(RegWrite),
        .wr_en(wr_en), .wmask(wmask)
    );
    
    //==============================
    // Processor Outputs
    //==============================
    
    // To Instruction Memory
    assign pc = pc_out[PC_WIDTH-1:0];
    
    // To Data Memory
    always@(*) begin
        if(!nrst) 
            addr = 0;
        else
            addr = ALURes[ADDR_WIDTH-1:0];
    end
    //wr_en connected to control output
    assign wdata = rd2_out;
    //wmask connected to control output
    
    
endmodule


//====================================================================================
//
//      RegFile
//
//====================================================================================
module RegisterFile #(
    parameter   DATA_WIDTH    = 64,         // Data width of each register
    parameter   NUM_REG       = 32,         // Number of registers
    parameter   ADDRESS_WIDTH = 5           // log_2(Num_reg)
)(
    //==============================
    //  Clocks and resets
    //==============================
    input                       clk,
    input                       nrst,
    //==============================
    //  Inputs
    //==============================  
    input                       reg_wr_en,      // Register write enable
    // Register write
    input   [DATA_WIDTH-1:0]    reg_wr_data,    // Data to write on register
    input   [ADDRESS_WIDTH-1:0] reg_wr_dest,    // Address to write on register
    // Register Read addresses
    input   [ADDRESS_WIDTH-1:0] reg_read_addr1, // 1st Address to read from
    input   [ADDRESS_WIDTH-1:0] reg_read_addr2, // 2nd Address to read from
    
    //==============================
    //  Outputs
    //==============================
    output  [DATA_WIDTH-1:0]    reg_read_data1, 
    output  [DATA_WIDTH-1:0]    reg_read_data2
);
    integer i;
    reg [DATA_WIDTH-1:0] register_file [0:NUM_REG-1];
    
    // Writing into the registers
    always@(posedge clk) begin
        if(!nrst)
            for (i=0; i < NUM_REG; i = i+1)
                register_file[i] <= 0;
        else
            if(reg_wr_en)
                if(reg_wr_dest > 0)
                    register_file[reg_wr_dest] <= reg_wr_data;
    end
    
    // Output assignment for register read
    assign    reg_read_data1 = register_file[reg_read_addr1];
    assign    reg_read_data2 = register_file[reg_read_addr2];

endmodule

//====================================================================================
//
//      ALU
//
//====================================================================================

module ALU #(
    parameter   ALUop_WIDTH   =   5,
    parameter   DATA_WIDTH    =   64,
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
    input       [ALUop_WIDTH-1:0]   ALUop,
    input       [DATA_WIDTH-1:0]    inA,
    input       [DATA_WIDTH-1:0]    inB,
    output                          zero,
    output  reg [DATA_WIDTH-1:0]    result
);

    always@(*) begin
        case(ALUop)
            OP_AND:     result = inA & inB;
            OP_OR:      result = inA | inB;
            OP_ADD:     result = inA + inB;
            OP_XOR:     result = inA ^ inB;
            OP_SUB:     result = $signed(inA) - $signed(inB);
            OP_SLT:     result = ($signed(inA) < $signed(inB)) ? 1'b1 : 1'b0;
            OP_ISEQ:    result = ($signed(inA) == $signed(inB)) ? 1'b1 : 1'b0;
            default:    result = 0;
        endcase
    end
    
    assign zero = (result==0) ? 1 : 0;  // zero=1 when result=0

endmodule

//====================================================================================
//
//      Program Counter
//
//====================================================================================
module PC #(
    parameter   PC_WIDTH = 32
)(
    //==============================
    //  Clocks and resets
    //==============================
    input                       clk,
    input                       nrst,
    //==============================
    //  Register Input and Output
    //==============================
    input      [PC_WIDTH-1:0]   next,
    output reg [PC_WIDTH-1:0]   current
);

    always@(posedge clk) begin
        if(!nrst)
            current <= 0;
        else
            current <= next;
    end

endmodule

//====================================================================================
//
//      Immediate Generator
//
//====================================================================================
module ImmGen #(
    parameter   INST_WIDTH  = 32,
    parameter   DATA_WIDTH  = 64,
    parameter   OPCODE_WID  = 7,
    //==============================
    //  OPCODES
    //==============================
    parameter   opcode_ld       = 7'b0000011,
    parameter   opcode_addi     = 7'b0010011,
    parameter   opcode_jalr     = 7'b1100111,
    parameter   opcode_sd       = 7'b0100011,
    parameter   opcode_rtype    = 7'b0110011,
    parameter   opcode_sbtype   = 7'b1100011,
    parameter   opcode_jal      = 7'b1101111
)(
    input       [INST_WIDTH-1:0]    inst,
    output reg  [DATA_WIDTH-1:0]    imm
);

wire [OPCODE_WID-1:0] opcode;
assign opcode = inst[6:0];

always@(*) begin
    imm = 0;
    case(opcode)
        // I-TYPE INSTRUCTIONS
        opcode_ld:      imm = { {52{inst[31]}} , inst[31:20]};
        opcode_addi:    imm = { {52{inst[31]}} , inst[31:20]};
        opcode_jalr:    imm = { {52{inst[31]}} , inst[31:20]};
        // S-TYPE INSTRUCTIONS
        opcode_sd:      imm = { {52{inst[31]}} , inst[31:25], inst[11:7]};
        default:        imm = 0;
    endcase
end

endmodule