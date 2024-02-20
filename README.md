pps-gen-gpio
============

Linux kernel PPS generator using GPIO pins.

In kernel 5.10 there is no support for using a GPIO pin as a PPS generator, only a GPIO PPS client is available. This driver is derived from GPIO driver by 
Juan Solano which is derived from the current parallel port PPS generator and provides a PPS signal through a GPIO pin specified in the device tree. The PPS signal is synchronized to the tv_sec increment of the wall clock. 

This version of the driver generates long pulses (500ms) and provides activatin/deactivation via sysfs.

Tested on Radxa CM3 IO Board.

Usage
-----
Set PPS GPIO pin in your device tree:

		pps-gen {
         compatible = "pps-gen-gpio";
         pinctrl-names = "default";
         pps-gen-gpio = <&gpio3 RK_PD5 GPIO_ACTIVE_HIGH>;
         default-state = "off";
         status = "okay";
      }

After modifying the device tree, add the files into drivers/pps/generators and configure the driver to be built as a module. You need to enable PPS support in the kernel.

When loaded, driver sets configured PPS pin to high and waits for activation command. To activate PPS generation, write 1 to
      /sys/devices/pps-gen/state/active

To deactivate, write 0 to
      /sys/devices/pps-gen/state/active
