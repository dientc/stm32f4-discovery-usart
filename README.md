# stm32f4-discovery-usart

This is a sample code for USART3 on stm32f4 discovery board. It runs on Ubuntu 18.04 with:
+ gcc-arm-embedded toolchain https://developer.arm.com/open-source/gnu-toolchain/gnu-rm
+ make, cmake
+ st-link

How to run:

- cd ~
- git clone https://github.com/dientc/stm32f4-discovery-usart.git
- cd stm32f4-discovery-usart
- make flash #to make and flash firmware

Note: This sample code builds with c99 library, so you can use many math functions in your code.
