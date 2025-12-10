## ====================================================
## On-board User LEDs (LD0-LD1) for led_gpio[1:0]
## ====================================================
set_property PACKAGE_PIN R14 [get_ports {led_gpio_tri_o[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_gpio_tri_o[0]}]

set_property PACKAGE_PIN P14 [get_ports {led_gpio_tri_o[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_gpio_tri_o[1]}]
