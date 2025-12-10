onbreak {quit -f}
onerror {quit -f}

vsim  -lib xil_defaultlib petalinux_base_opt

set NumericStdNoWarnings 1
set StdArithNoWarnings 1

do {wave.do}

view wave
view structure
view signals

do {petalinux_base.udo}

run 1000ns

quit -force
