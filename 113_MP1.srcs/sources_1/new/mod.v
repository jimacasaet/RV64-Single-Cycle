///////////////////////////////////////////////////////////////////////////////////////
//  Machine Problem 1: 
//  Single Cycle RISC V Processor
//
//  Basic Modules
//  (MUX, Adders, etc)
//
//  John Rufino I. Macasaet
//
///////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps


//====================================================================================
//
//      MUXes
//
//====================================================================================
module MUX #(
    parameter   WIDTH   = 64
)(
    input   [WIDTH-1:0]   in0,
    input   [WIDTH-1:0]   in1,
    input                 sel,
    output  [WIDTH-1:0]   out
);
    
    assign out = (sel) ? in1 : in0;
endmodule

module MUX3 #(
    parameter   WIDTH   = 64
)(
    input       [WIDTH-1:0]   in0,
    input       [WIDTH-1:0]   in1,
    input       [WIDTH-1:0]   in2,
    input       [1:0]         sel,
    output reg  [WIDTH-1:0]   out
);
    
    always@(*) begin
        case(sel)
            0:  out = in0;
            1:  out = in1;
            2:  out = in2;
            default: out = 0;
        endcase
    end
endmodule

//====================================================================================
//
//      Adder
//
//====================================================================================
module ADD #(
    parameter   WIDTH   = 32
)(
    input   [WIDTH-1:0] A,
    input   [WIDTH-1:0] B,
    output  [WIDTH-1:0] Sum,
    output              Cout
);
    assign {Cout,Sum} = A + B;
    
endmodule
