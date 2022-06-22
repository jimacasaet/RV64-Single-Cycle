/* Program memory model
 * - Asynchronous read
 * - Replace $readmemh() argument with your own memory initialization if needed
 * -- Argument points to a text file containing hex values of a word per line
 * -- Default file: progmem.mem
 * -- For Vivado, use Add Sources -> Simulation Sources -> Add File to include the memory initialization
 */

`timescale 1ns / 1ps
`include "defines.h"

module mem_prog
    #(  parameter DATA_DEP = 20, // Depth of memory (in words)
        parameter ADDR_WID = 30   // Word address width (32-2)
    ) 
    (   input   [ADDR_WID-1:0]  addr, // doubleword address
        output  [31:0]          rdata
    );
    
    reg [31:0] memdata [0:DATA_DEP-1];
    
    /* Read path */
    assign rdata = memdata[addr];
    
    /* Initialization */
    initial begin
        `ifdef LDTEST
        $readmemh("ldtest_prog.mem",memdata);
        `elsif ATEST
        $readmemh("arithtest_prog.mem",memdata);
        `elsif BTEST
        $readmemh("brtest_prog.mem",memdata);
        `elsif LTEST
        $readmemh("looptest_prog.mem",memdata);
        `elsif JTEST
        $readmemh("jtest_prog.mem",memdata);
        `elsif PLDTEST
        $readmemh("pldtest_prog.mem",memdata);
        `elsif PATEST
        $readmemh("parithtest_prog.mem",memdata);
        `elsif PBTEST
        $readmemh("pbrtest_prog.mem",memdata);
        `else
        $readmemh("progmem.mem",memdata);
        `endif
    end
        
endmodule
