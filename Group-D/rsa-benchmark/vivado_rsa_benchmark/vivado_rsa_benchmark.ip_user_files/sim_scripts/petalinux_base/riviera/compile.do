transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib riviera/xilinx_vip
vlib riviera/xpm
vlib riviera/axi_infrastructure_v1_1_0
vlib riviera/axi_vip_v1_1_22
vlib riviera/processing_system7_vip_v1_0_24
vlib riviera/xil_defaultlib
vlib riviera/axi_lite_ipif_v3_0_4
vlib riviera/interrupt_control_v3_1_5
vlib riviera/axi_gpio_v2_0_37
vlib riviera/proc_sys_reset_v5_0_17
vlib riviera/smartconnect_v1_0
vlib riviera/axi_register_slice_v2_1_36

vmap xilinx_vip riviera/xilinx_vip
vmap xpm riviera/xpm
vmap axi_infrastructure_v1_1_0 riviera/axi_infrastructure_v1_1_0
vmap axi_vip_v1_1_22 riviera/axi_vip_v1_1_22
vmap processing_system7_vip_v1_0_24 riviera/processing_system7_vip_v1_0_24
vmap xil_defaultlib riviera/xil_defaultlib
vmap axi_lite_ipif_v3_0_4 riviera/axi_lite_ipif_v3_0_4
vmap interrupt_control_v3_1_5 riviera/interrupt_control_v3_1_5
vmap axi_gpio_v2_0_37 riviera/axi_gpio_v2_0_37
vmap proc_sys_reset_v5_0_17 riviera/proc_sys_reset_v5_0_17
vmap smartconnect_v1_0 riviera/smartconnect_v1_0
vmap axi_register_slice_v2_1_36 riviera/axi_register_slice_v2_1_36

vlog -work xilinx_vip  -incr "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi4stream_vip_axi4streampc.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi_vip_axi4pc.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/xil_common_vip_pkg.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi4stream_vip_pkg.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi_vip_pkg.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi4stream_vip_if.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/axi_vip_if.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/clk_vip_if.sv" \
"/opt/Xilinx/2025.2/data/xilinx_vip/hdl/rst_vip_if.sv" \

vlog -work xpm  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"/opt/Xilinx/2025.2/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
"/opt/Xilinx/2025.2/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv" \
"/opt/Xilinx/2025.2/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -93  -incr \
"/opt/Xilinx/2025.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work axi_infrastructure_v1_1_0  -incr -v2k5 "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl/axi_infrastructure_v1_1_vl_rfs.v" \

vlog -work axi_vip_v1_1_22  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/b16a/hdl/axi_vip_v1_1_vl_rfs.sv" \

vlog -work processing_system7_vip_v1_0_24  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl/processing_system7_vip_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_processing_system7_0_0/sim/petalinux_base_processing_system7_0_0.v" \

vcom -work axi_lite_ipif_v3_0_4 -93  -incr \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/66ea/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd" \

vcom -work interrupt_control_v3_1_5 -93  -incr \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/d8cc/hdl/interrupt_control_v3_1_vh_rfs.vhd" \

vcom -work axi_gpio_v2_0_37 -93  -incr \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/0271/hdl/axi_gpio_v2_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -93  -incr \
"../../../bd/petalinux_base/ip/petalinux_base_axi_gpio_0_0/sim/petalinux_base_axi_gpio_0_0.vhd" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/sim/bd_c62f.v" \

vcom -work proc_sys_reset_v5_0_17 -93  -incr \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9438/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -93  -incr \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_1/sim/bd_c62f_psr_aclk_0.vhd" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/sc_util_v1_0_vl_rfs.sv" \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/3d9a/hdl/sc_mmu_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_2/sim/bd_c62f_s00mmu_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/7785/hdl/sc_transaction_regulator_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_3/sim/bd_c62f_s00tr_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/3051/hdl/sc_si_converter_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_4/sim/bd_c62f_s00sic_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/852f/hdl/sc_axi2sc_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_5/sim/bd_c62f_s00a2s_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/sc_node_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_6/sim/bd_c62f_sarn_0.sv" \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_7/sim/bd_c62f_srn_0.sv" \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_8/sim/bd_c62f_sawn_0.sv" \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_9/sim/bd_c62f_swn_0.sv" \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_10/sim/bd_c62f_sbn_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/fca9/hdl/sc_sc2axi_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_11/sim/bd_c62f_m00s2a_0.sv" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/e44a/hdl/sc_exit_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/bd_0/ip/ip_12/sim/bd_c62f_m00e_0.sv" \

vcom -work smartconnect_v1_0 -93  -incr \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/cb42/hdl/sc_ultralite_v1_0_rfs.vhd" \

vlog -work smartconnect_v1_0  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/cb42/hdl/sc_ultralite_v1_0_rfs.sv" \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/0848/hdl/sc_switchboard_v1_0_vl_rfs.sv" \

vlog -work axi_register_slice_v2_1_36  -incr -v2k5 "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/bc4b/hdl/axi_register_slice_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib  -incr "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/ip/petalinux_base_smartconnect_0_2/sim/petalinux_base_smartconnect_0_2.sv" \

vcom -work xil_defaultlib -93  -incr \
"../../../bd/petalinux_base/ip/petalinux_base_proc_sys_reset_0_2/sim/petalinux_base_proc_sys_reset_0_2.vhd" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/ec67/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/9a25/hdl" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/f0b6/hdl/verilog" "+incdir+../../../../vivado_rsa_benchmark.gen/sources_1/bd/petalinux_base/ipshared/00fe/hdl/verilog" "+incdir+../../../../../../../../../opt/Xilinx/2025.2/data/rsb/busdef" "+incdir+/opt/Xilinx/2025.2/data/xilinx_vip/include" -l xilinx_vip -l xpm -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_22 -l processing_system7_vip_v1_0_24 -l xil_defaultlib -l axi_lite_ipif_v3_0_4 -l interrupt_control_v3_1_5 -l axi_gpio_v2_0_37 -l proc_sys_reset_v5_0_17 -l smartconnect_v1_0 -l axi_register_slice_v2_1_36 \
"../../../bd/petalinux_base/sim/petalinux_base.v" \

vlog -work xil_defaultlib \
"glbl.v"

