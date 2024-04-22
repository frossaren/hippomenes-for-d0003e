## How to run for d0003e students
On windows:
Locate the vivado.bat file within the vivado install folder.
Cd to hippomenes/fpga
Note: If you are unable to run vivado through the console on windows it means vivado isnt added in the path variable. To do this go to advanced system settings, enviroment variables and add the path to the folder where vivado.bat is located.
run vivado -mode tcl -source arty.tcl
This will create the Vivado project file arty/arty.xpr

- Run Synthesis
- Run Implementation
- Generate Bitstream
- Open Hardware Manager (the ARTY needs to be connected over USB)
- Program Device (the `fpga\arty\arty.runs\impl_1\fgpa_arty.bit` under Windows or `fpga/arty/arty.runs/impl_1/fgpa_arty.bit` under Linux).

Source file examples are found under `rust_examples`, to build `asm_loop`:
On windows, navigate to "rust_examples" then run ./run.bat "file", where "file" is the name of the rust program in rust_examples. This will build the program and flash it to the fpga
or on linux run the run.sh from the same folder