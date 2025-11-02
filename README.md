**Introduction**

This is a simple LIN to LIN main-in-the-middle adapter that allows MQB to work in a C7/C7.5 MLB car.
Hardware is based on the readily available yellow board on Aliexpress and similar.


**Features differing from original:**

* Drive select control through horn input
* Back button
* Re-mapping of any button
* Steering heater temperature adjustment


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
I have not yet tested if the program works on the CKS32, only the genuine STM32F103.