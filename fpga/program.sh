#!/bin/bash
updatemem --meminfo hippomenes/hippomenes.runs/impl_2/fpga_arty.mmi --data ../rust_examples/binary.mem --proc hippo/imem/xpm_memory_spram_inst/xpm_memory_base_inst --bit hippomenes/hippomenes.runs/impl_2/fpga_arty.bit -out hippomenes/hippomenes.runs/impl_2/fpga_arty.bit -force
vivado -mode tcl -source program.tcl
