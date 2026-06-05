## -------------------------------------------------------------------------
## can_adc.sdc - timing constraints for the CAN_ADC project (DE0-Nano)
## -------------------------------------------------------------------------

## 50 MHz board oscillator (PIN_R8)
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

## CAN timing clock: CLOCK_50 divided by 2 (toggle FF "can_clk") = 25 MHz.
## If TimeQuest reports the target register cannot be found, adjust the node
## name to match your hierarchy (e.g. {top|can_clk}).
create_generated_clock -name can_clk -source [get_ports CLOCK_50] \
    -divide_by 2 [get_registers {can_clk}]

## Account for clock uncertainty (jitter, etc.)
derive_clock_uncertainty

## Asynchronous / low-speed I/O - cut from timing analysis.
## (Push-buttons, the ADC SPI pins, and the CAN line are not source-synchronous
##  to the FPGA clock at these low rates; constrain on the bench if needed.)
set_false_path -from [get_ports {KEY[*]}] -to *
set_false_path -from [get_ports {ADC_SDAT}] -to *
set_false_path -to   [get_ports {ADC_CS_N ADC_SCLK ADC_SADDR}]
set_false_path -from [get_ports {CAN_RX}] -to *
set_false_path -to   [get_ports {CAN_TX}]
set_false_path -to   [get_ports {LED[*]}]
