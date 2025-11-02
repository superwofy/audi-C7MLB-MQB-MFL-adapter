**Introduction**

This is a simple LIN to LIN main-in-the-middle adapter that allows MQB (and other?) steering wheel controls to work in a C7/C7.5 MLB car.  
Hardware is based on the readily available yellow board on Aliexpress and similar. With a genuine MCU and a few resistors this should cost no more than $30.  
It's all built with stm32duino for simplicity. No need for CubeIDE or understanding of the HAL.

**Features differing from original:**

* Re-mapping of any button
* Drive select control through horn input (for R8-style [buttons](extra%20buttons/README.md))
* Back button functionality
* Steering heater temperature adjustment
* Optional bypass of errors caused by missing components


**Arduino settings:**

* Board: "Generic STM32F1 series"
* Optimize: "Faster (-O3) with LTO"
* Board part number: "Generic F103CBTx"
* Upload method: *
* U(S)ART support: "Enabled (no generic 'Serial')"


**Hardware changes:**

See [adapter-readme](adapter/README.md)  
![board-pinout](adapter/pinout.jpg "board-pinout")


**Notes**

I was not able to connect to the processor with a genuine ST-LINK/V2. J-LINK works fine if BOOT0 is grounded.  
I have not yet tested if the program works on the CKS32, only the genuine STM32F103. **// TODO**  
I have tested it on my C7.5 with the original buttons and with a B9 style steering wheel and controls YMMV...