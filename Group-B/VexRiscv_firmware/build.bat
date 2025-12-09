@echo off
setlocal ENABLEDELAYEDEXPANSION

rem -------------------------------
rem Tools
rem -------------------------------
set CC=riscv-none-elf-gcc
set OBJCOPY=riscv-none-elf-objcopy

rem -------------------------------
rem Flags (RV32IMC + CSRs)
rem -------------------------------
rem IMPORTANT: Only use rv32imc if the Murax core was built with:
rem   - MulPlugin / DivPlugin   (M extension)
rem   - compressedGen = true   (C extension)
rem Otherwise use rv32i instead.
set CFLAGS=-std=c99 -O3 -march=rv32imc_zicsr -mabi=ilp32 -mcmodel=medany -mstrict-align -ffreestanding -fno-builtin -ffunction-sections -fdata-sections
set LDFLAGS=-nostdlib -nostartfiles -Wl,--gc-sections -Wl,-Map=build\prog.map -T linker.ld

rem -------------------------------
rem Prepare build dir
rem -------------------------------
if not exist build mkdir build

echo [1/4] Compile crt0.S
"%CC%" %CFLAGS% -c crt0.S -o build\crt0.o || goto :err

echo [2/4] Compile main.c
"%CC%" %CFLAGS% -c main.c -o build\main.o || goto :err

echo [3/4] Link -> build\prog.elf
"%CC%" %CFLAGS% build\crt0.o build\main.o %LDFLAGS% -lc -lgcc -o build\prog.elf || goto :err

echo [4/4] ELF -> Intel HEX (prog.hex)
"%OBJCOPY%" -O ihex build\prog.elf prog.hex || goto :err

echo.
echo Copying prog.hex to Murax hex location...
copy /Y prog.hex ..\..\..\ressource\hex\muraxDemo.hex >nul || goto :err

echo.
echo SUCCESS: prog.hex (RV32IMC, Intel HEX) ready and copied to src\main\ressource\hex\muraxDemo.hex
echo Next steps:
echo   1. From project root:  sbt "runMain vexriscv.demo.MuraxWithRamInit"
echo   2. Re-run synth/impl/bitstream in Vivado and program the FPGA.
exit /b 0

:err
echo.
echo BUILD FAILED
exit /b 1
