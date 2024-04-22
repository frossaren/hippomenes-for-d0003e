@echo off

rem Check if the argument is provided
if "%1"=="" (
    echo Error: Please provide the filename for the code to build as an argument.
    exit /b 1
)

rem Step 1: Remove existing binary.mem file
if exist binary.mem (
    del binary.mem
)

rem Step 2: Build Cargo examples with the provided file argument
cargo build --examples %1 --release

rem Step 3: Move the resulting binary file to binary.mem
move .\text.mem binary.mem

rem Step 4: Run program_arty.cmd
cd ..
cd fpga
program_arty.cmd
