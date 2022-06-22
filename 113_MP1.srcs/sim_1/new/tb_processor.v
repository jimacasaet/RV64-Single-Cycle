`timescale 1ns / 1ps
`include "defines.h"

//========================
// PROCESSOR TESTBENCH
//========================

module tb_processor();

    parameter PC_WIDTH = 32;
    parameter DATA_WID = 64;
    parameter ADDR_WID = 32;
    parameter DM_ADD_W = 29;    // Data Memory Addr Width
    parameter PM_ADD_W = 30;    // Program Memory Addr Width

    reg                 clk, nrst;
    wire [PC_WIDTH-1:0] inst;
    wire [ADDR_WID-1:0] pc;
    wire [DATA_WID-1:0] wdata, rdata;
    wire [ADDR_WID-1:0] addr;
    wire                wr_en;
    wire [7:0]          wmask;

    processor I_PROCESSOR(
        // Clocks and resets
        .clk(clk), .nrst(nrst),
        // Program Memory
        .pc(pc), .inst(inst),
        // Data Memory
        .addr(addr), .wr_en(wr_en), .wdata(wdata),
        .wmask(wmask), .rdata(rdata)
    );
    
    mem_model I_MEM_MODEL(
        // Clocks and resets
        .clk(clk),
        // Inputs
        .addr(addr[ADDR_WID-1:ADDR_WID-DM_ADD_W]), .wr_en(wr_en),
        .wdata(wdata), .wmask(wmask),
        // Outputs
        .rdata(rdata)
    );
    
    mem_prog I_MEM_PROG(
        .addr(pc[ADDR_WID-1:ADDR_WID-PM_ADD_W]), .rdata(inst)
    );
    
    always begin #10; clk = !clk; end
    
    initial begin
        clk     <= 1'b0;
        nrst    <= 1'b0;
        #15
        nrst    <= 1'b1;
        #5
        `ifdef LDTEST
            #280
        `elsif ATEST
            #400
        `elsif BTEST
            #280
        `elsif LTEST
            #1800
        `elsif JTEST
            #240
        `else
            #400
        `endif

        $finish;
    end

endmodule
