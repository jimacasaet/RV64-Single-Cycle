
x
Command: %s
53*	vivadotcl2G
3synth_design -top processor -part xc7a35ticsg324-1L2default:defaultZ4-113h px? 
:
Starting synth_design
149*	vivadotclZ4-321h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7a35ti2default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7a35ti2default:defaultZ17-349h px? 
?
%s*synth2?
sStarting Synthesize : Time (s): cpu = 00:00:03 ; elapsed = 00:00:03 . Memory (MB): peak = 444.781 ; gain = 153.668
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2
	processor2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
542default:default8@Z8-6157h px? 
^
%s
*synth2F
2	Parameter PC_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter INST_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter ADDR_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter WDATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter WMASK_WIDTH bound to: 8 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter RDATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter ALUOP_WIDTH bound to: 5 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter FUNCT3_WIDTH bound to: 3 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter FUNCT7_WIDTH bound to: 7 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter OPCODE_WIDTH bound to: 7 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2 
RegisterFile2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
3032default:default8@Z8-6157h px? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter NUM_REG bound to: 32 - type: integer 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter ADDRESS_WIDTH bound to: 5 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2 
RegisterFile2default:default2
 2default:default2
12default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
3032default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
ImmGen2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
4292default:default8@Z8-6157h px? 
`
%s
*synth2H
4	Parameter INST_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
_
%s
*synth2G
3	Parameter OPCODE_WID bound to: 7 - type: integer 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter opcode_ld bound to: 7'b0000011 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter opcode_addi bound to: 7'b0010011 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter opcode_jalr bound to: 7'b1100111 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter opcode_sd bound to: 7'b0100011 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter opcode_rtype bound to: 7'b0110011 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter opcode_sbtype bound to: 7'b1100011 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter opcode_jal bound to: 7'b1101111 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ImmGen2default:default2
 2default:default2
22default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
4292default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
MUX2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
202default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
MUX2default:default2
 2default:default2
32default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
202default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
ALU2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
3562default:default8@Z8-6157h px? 
`
%s
*synth2H
4	Parameter ALUop_WIDTH bound to: 5 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_AND bound to: 0 - type: integer 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter OP_OR bound to: 1 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_ADD bound to: 2 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_XOR bound to: 3 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_SUB bound to: 6 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_SLT bound to: 7 - type: integer 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter OP_ISEQ bound to: 10 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ALU2default:default2
 2default:default2
42default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
3562default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
MUX32default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
322default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter WIDTH bound to: 64 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
MUX32default:default2
 2default:default2
52default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
322default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
ADD2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
572default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ADD2default:default2
 2default:default2
62default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
572default:default8@Z8-6155h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2 
I_ADD_ADDER02default:default2
ADD2default:default2
42default:default2
32default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
1902default:default8@Z8-7023h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2 
I_ADD_ADDER12default:default2
ADD2default:default2
42default:default2
32default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
2042default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2'
MUX__parameterized02default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
202default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2'
MUX__parameterized02default:default2
 2default:default2
62default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
202default:default8@Z8-6155h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2 
I_ADD_ADDER22default:default2
ADD2default:default2
42default:default2
32default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
2302default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2(
MUX3__parameterized02default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
322default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2(
MUX3__parameterized02default:default2
 2default:default2
62default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mod.v2default:default2
322default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
PC2default:default2
 2default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
4002default:default8@Z8-6157h px? 
^
%s
*synth2F
2	Parameter PC_WIDTH bound to: 32 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
PC2default:default2
 2default:default2
72default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
4002default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
Control2default:default2
 2default:default2b
LC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/control.v2default:default2
142default:default8@Z8-6157h px? 
`
%s
*synth2H
4	Parameter ALUOP_WIDTH bound to: 5 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter OPCODE_WIDTH bound to: 7 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter FUNCT3_WIDTH bound to: 3 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter FUNCT7_WIDTH bound to: 7 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter WMASK_WIDTH bound to: 8 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter PCSRC_WIDTH bound to: 2 - type: integer 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter REGWRSRC_WIDTH bound to: 2 - type: integer 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter opcode_ld bound to: 7'b0000011 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter opcode_addi bound to: 7'b0010011 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter opcode_jalr bound to: 7'b1100111 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter opcode_sd bound to: 7'b0100011 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter opcode_rtype bound to: 7'b0110011 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter opcode_sbtype bound to: 7'b1100011 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter opcode_jal bound to: 7'b1101111 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter funct3_ld bound to: 3'b011 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter funct3_sd bound to: 3'b011 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter funct3_addsub bound to: 3'b000 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter funct3_and bound to: 3'b111 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter funct3_or bound to: 3'b110 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter funct3_xor bound to: 3'b100 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter funct3_slt bound to: 3'b010 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter funct3_beq bound to: 3'b000 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter funct3_bne bound to: 3'b001 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter funct7_add bound to: 7'b0000000 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter funct7_sub bound to: 7'b0100000 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_AND bound to: 0 - type: integer 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter OP_OR bound to: 1 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_ADD bound to: 2 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_XOR bound to: 3 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_SUB bound to: 6 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter OP_SLT bound to: 7 - type: integer 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter OP_ISEQ bound to: 10 - type: integer 
2default:defaulth p
x
? 
?
-case statement is not full and has no default155*oasys2b
LC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/control.v2default:default2
1452default:default8@Z8-155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Control2default:default2
 2default:default2
82default:default2
12default:default2b
LC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/control.v2default:default2
142default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	processor2default:default2
 2default:default2
92default:default2
12default:default2^
HC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.srcs/sources_1/new/mp1.v2default:default2
542default:default8@Z8-6155h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[19]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[18]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[17]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[16]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[15]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[14]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[13]2default:defaultZ8-3331h px? 
}
!design %s has unconnected port %s3331*oasys2
ImmGen2default:default2
inst[12]2default:defaultZ8-3331h px? 
?
%s*synth2?
sFinished Synthesize : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 506.988 ; gain = 215.875
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
~Finished Constraint Validation : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 506.988 ; gain = 215.875
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Loading part: xc7a35ticsg324-1L
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 506.988 ; gain = 215.875
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
X
Loading part %s157*device2%
xc7a35ticsg324-1L2default:defaultZ21-403h px? 
u
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2
wmask2default:defaultZ8-5546h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 506.988 ; gain = 215.875
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   3 Input     64 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     33 Bit       Adders := 3     
2default:defaulth p
x
? 
8
%s
*synth2 
+---XORs : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit         XORs := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               64 Bit    Registers := 32    
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               32 Bit    Registers := 1     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   3 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   4 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     32 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   4 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      8 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   5 Input      4 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      3 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   5 Input      2 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      2 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      2 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 34    
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      1 Bit        Muxes := 3     
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Y
%s
*synth2A
-Start RTL Hierarchical Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Hierarchical RTL Component report 
2default:defaulth p
x
? 
>
%s
*synth2&
Module processor 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
? 
A
%s
*synth2)
Module RegisterFile 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               64 Bit    Registers := 32    
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 32    
2default:defaulth p
x
? 
;
%s
*synth2#
Module ImmGen 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   3 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   5 Input      2 Bit        Muxes := 1     
2default:defaulth p
x
? 
8
%s
*synth2 
Module MUX 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
8
%s
*synth2 
Module ALU 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   3 Input     64 Bit       Adders := 1     
2default:defaulth p
x
? 
8
%s
*synth2 
+---XORs : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit         XORs := 1     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
9
%s
*synth2!
Module MUX3 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   4 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
8
%s
*synth2 
Module ADD 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     33 Bit       Adders := 1     
2default:defaulth p
x
? 
H
%s
*synth20
Module MUX__parameterized0 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
? 
I
%s
*synth21
Module MUX3__parameterized0 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   4 Input     32 Bit        Muxes := 1     
2default:defaulth p
x
? 
7
%s
*synth2
Module PC 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               32 Bit    Registers := 1     
2default:defaulth p
x
? 
<
%s
*synth2$
Module Control 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      8 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   5 Input      4 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      3 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      2 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      2 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   8 Input      1 Bit        Muxes := 3     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 2     
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
[
%s
*synth2C
/Finished RTL Hierarchical Component Statistics
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2j
VPart Resources:
DSPs: 90 (col length:60)
BRAMs: 100 (col length: RAMB18 60 RAMB36 30)
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
]
%s
*synth2E
1Warning: Parallel synthesis criteria is not met 
2default:defaulth p
x
? 

8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2#
I_Control/wmask2default:defaultZ8-5546h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:01:02 ; elapsed = 00:01:02 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
|Finished Timing Optimization : Time (s): cpu = 00:01:04 ; elapsed = 00:01:04 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][0] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][1] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][2] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][3] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][4] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][5] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][6] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][7] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][8] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2<
(\I_RegisterFile/register_file_reg[0][9] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][10] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][11] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][12] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][13] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][14] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][15] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][16] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][17] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][18] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][19] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][20] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][21] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][22] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][23] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][24] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][25] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][26] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][27] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][28] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][29] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][30] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][31] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][32] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][33] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][34] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][35] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][36] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][37] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][38] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][39] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][40] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][41] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][42] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][43] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][44] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][45] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][46] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][47] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][48] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][49] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][50] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][51] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][52] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][53] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][54] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][55] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][56] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][57] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][58] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][59] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][60] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][61] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][62] 2default:defaultZ8-3333h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2=
)\I_RegisterFile/register_file_reg[0][63] 2default:defaultZ8-3333h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
{Finished Technology Mapping : Time (s): cpu = 00:01:06 ; elapsed = 00:01:06 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
uFinished IO Insertion : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
D
%s
*synth2,

Report Check Netlist: 
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|      |Item              |Errors |Warnings |Status |Description       |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|1     |multi_driven_nets |      0|        0|Passed |Multi driven nets |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Instances : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Ports : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Nets : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
D
%s*synth2,
+------+-------+------+
2default:defaulth px? 
D
%s*synth2,
|      |Cell   |Count |
2default:defaulth px? 
D
%s*synth2,
+------+-------+------+
2default:defaulth px? 
D
%s*synth2,
|1     |BUFG   |     1|
2default:defaulth px? 
D
%s*synth2,
|2     |CARRY4 |    70|
2default:defaulth px? 
D
%s*synth2,
|3     |LUT1   |     4|
2default:defaulth px? 
D
%s*synth2,
|4     |LUT2   |   262|
2default:defaulth px? 
D
%s*synth2,
|5     |LUT3   |    60|
2default:defaulth px? 
D
%s*synth2,
|6     |LUT4   |    91|
2default:defaulth px? 
D
%s*synth2,
|7     |LUT5   |   271|
2default:defaulth px? 
D
%s*synth2,
|8     |LUT6   |  1264|
2default:defaulth px? 
D
%s*synth2,
|9     |MUXF7  |   513|
2default:defaulth px? 
D
%s*synth2,
|10    |MUXF8  |    88|
2default:defaulth px? 
D
%s*synth2,
|11    |FDRE   |  2016|
2default:defaulth px? 
D
%s*synth2,
|12    |IBUF   |    98|
2default:defaulth px? 
D
%s*synth2,
|13    |OBUF   |   137|
2default:defaulth px? 
D
%s*synth2,
+------+-------+------+
2default:defaulth px? 
E
%s
*synth2-

Report Instance Areas: 
2default:defaulth p
x
? 
\
%s
*synth2D
0+------+-----------------+-------------+------+
2default:defaulth p
x
? 
\
%s
*synth2D
0|      |Instance         |Module       |Cells |
2default:defaulth p
x
? 
\
%s
*synth2D
0+------+-----------------+-------------+------+
2default:defaulth p
x
? 
\
%s
*synth2D
0|1     |top              |             |  4875|
2default:defaulth p
x
? 
\
%s
*synth2D
0|2     |  I_ADD_ADDER0   |ADD          |     8|
2default:defaulth p
x
? 
\
%s
*synth2D
0|3     |  I_ADD_ADDER1   |ADD_0        |    48|
2default:defaulth p
x
? 
\
%s
*synth2D
0|4     |  I_ADD_ADDER2   |ADD_1        |    14|
2default:defaulth p
x
? 
\
%s
*synth2D
0|5     |  I_ALU          |ALU          |    98|
2default:defaulth p
x
? 
\
%s
*synth2D
0|6     |  I_PC           |PC           |   183|
2default:defaulth p
x
? 
\
%s
*synth2D
0|7     |  I_RegisterFile |RegisterFile |  4286|
2default:defaulth p
x
? 
\
%s
*synth2D
0+------+-----------------+-------------+------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
s
%s
*synth2[
GSynthesis finished with 0 errors, 0 critical warnings and 11 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
~Synthesis Optimization Runtime : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth p
x
? 
?
%s
*synth2?
Synthesis Optimization Complete : Time (s): cpu = 00:01:08 ; elapsed = 00:01:08 . Memory (MB): peak = 784.105 ; gain = 492.992
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
g
-Analyzing %s Unisim elements for replacement
17*netlist2
6712default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
?
?Netlist '%s' is not ideal for floorplanning, since the cellview '%s' contains a large number of primitives.  Please consider enabling hierarchy in synthesis if you want to do floorplanning.
310*netlist2
	processor2default:default2 
RegisterFile2default:defaultZ29-101h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0022default:default2
784.1052default:default2
0.0002default:defaultZ17-268h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
982default:default2
122default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
synth_design: 2default:default2
00:01:122default:default2
00:01:142default:default2
784.1052default:default2
492.9922default:defaultZ17-268h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0022default:default2
784.1052default:default2
0.0002default:defaultZ17-268h px? 
K
"No constraints selected for write.1103*constraintsZ18-5210h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2^
JC:/Users/macas/Documents/Xilinx/113_MP1/113_MP1.runs/synth_1/processor.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2|
hExecuting : report_utilization -file processor_utilization_synth.rpt -pb processor_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Wed May 18 01:23:52 20222default:defaultZ17-206h px? 


End Record