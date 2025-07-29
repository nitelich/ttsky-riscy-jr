@echo off
REM Basic command structure:
REM
REM Compile the Verilog using Icarus Verilog
REM    iverilog -s (module to run) -o (output object file) (input file 1) (input file 2) (...)

REM Get starting time
echo Starting time: %time%

REM Compilation test for the RISCY Jr. CPU
echo Running RISCY Jr. CPU core compilation test...
C:\iverilog\bin\iverilog.exe -Wall -s tt_um_nitelich_riscyjr -o project.o project.v peripheral_library.v alu_library.v
echo:

REM Clean up all unnecessary files
del .\*.o 2>nul
del .\*.vcd 2>nul

REM All tests done!
echo End time: %time%
echo All tests complete
pause