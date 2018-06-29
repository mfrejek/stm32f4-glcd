# STM32F4-GLCD

[[https://github.com/mfrejek/stm32f4-glcd/blob/master/screen.jpg|alt=Screenshot]]

This is a small graphics library and display driver for STM32 controllers. It is designed and tested on a STM32F407 MCU and a 320x480 color display with ILI9481 controller.
The graphics library `gfxlib.c` is mostly platform independent; only the display driver `ili9481_drv_*HAL*.c` must be adapted to the MCU, display controller and HAL used. 
Three major STM32 HALs are supported: The 'old' Standard Peripheral Driver, the 'new' STM32Cube HAL by ST Microelectronics, and of course libopencm3. Also, no HAL may be used, then the driver will directly access the hardware registers using the CMSIS register definitions.

## Features

Available graphics elements:

* Texts with 4 different monospaced fonts (8 px, 16 px, 24 px or 32 px height)
* Rectangles, optionally with border
* Circles, optionally with border
* Arcs (circle segments)
* Polygons with up to 8 points
* Bitmaps; monochrome and 16 bit color

For most of the elements, it can be selected if the background should be drawn in a background color or is transparent

Technical aspects:

* No frame buffer needed => Low RAM usage
* ILI9481 driver uses DMA and the FSMC for fast display update
* About 25-55 FPS in the demo; depending on how many elements are displayed
* Maximum number of display elements is configurable
* Elements overlay each other in the order they are drawn
* Portable to other display types and controllers, as long as they have 16 bpp (64K colors)

All rendering is done line by line. This eliminates the need for a complete frame buffer, which would be 320x480x2 = 307200 bytes in size; exceeding the RAM capacity of most MCUs.
When a graphics element is added, all required information for rendering the element is stored in the `graphItems` struct. During rendering, the display image is drawn line by line from this struct.
Still, the library supports background transparency for texts, rectangles and monochrome bitmaps; and has a defined and predictible Z-order: The item which was added last is drawn on the very top.
Alpha channels are however not supported; mostly because I never needed them and also for performance reasons. Adding them would be possible.

Please note that the library was mainly created for being able to test and develop a university related project at home.
So, I focused on the subset of functionality which I needed for the project and didn't do too much testing with other input parameters.
Everything works fine in the demo, but during development there were many issues with the arc and polygon functions.
While I resolved all issues I encountered, these functions still might not behave as expected for certain drawing parameters.

## MCU Ressource Requirements

* About 35 kB flash (of which 28 kB is for the font tables)
* 9 k RAM (with the maximum number of graphics elements NUM_GFX_ELEMENTS set to 128)

## How to use

### Hardware Side

The driver is designed for a ILI9481 LCD with 16 bit parallel interface.
It has the following conenctions to the STM32F407 MCU:

| ILI9481 | STM32F407 |
| ------------------- |
| DB[0-15] | FSMC_D[0-15] |
| RS | FSMC_A18 |
| RST | +3.3V, or RESET of the STM32F407 |
| CS | FSMC_NE1 |
| WR | FSMC_NWE |

Since some LCDs don't have a RD pin for reading from the ILI9481, the driver doesn't need and use this pin.

### Running the Demo
* Connect the display to the MCU
* When building with libopencm3: Download + build libopencm3 (if you don't have it yet), and set the 'LIBOPENCM3_DIR' variable in the makefile to your libopemcm3 directory.
* Make sure an ARM GCC toolchain is in your PATH
* Build the example using 'make'. With the 'HAL' parameter, the HAL to use can be specified )'libopencm3', 'stdperiph', 'stm32cube', or 'none'). 'none' is the default. For example 'make HAL=libopemcm3' will build the example using the libopencm3 HAL.
* Flash it via a ST-Link by executing 'make flash', assuming that 'st-flash' is within your PATH. Otherwise, flash the binary 'src/main.bin' with a programmer of your choice.
* Note that the STM32Cube startup code currently doesn't work, therefore don't flash the demo with the STM32Cube HAL. 
* Now the demo should run

### Software Side

In your main program, include `gfxlib.h`. At startup, call `P_9481_HardwareInit()` and `P_9481_DisplayInit()`.
`P_9481_HardwareInit()` initialized the GPIOs, FSMC and DMA channel for communication with the display.
`P_9481_DisplayInit()` configures and initializes the ILI9481 display controller. Afterwards, the display is ready for adding graphics elements.
For example
`GFXAddText(-1, "Hello World", 0, 0, 0xFFFF, MEDIUM, 0, 0x0000)`
adds a white text in the top left corner of the LCD to the graphics buffer.
Note that calling a 'GFXAdd...' function doesn't immediately draw the element on the screen. At first, it leaves the display contents unchanged.
For updating the display, `GFXUpdateDisplay()` must be called.
`GFXReset()` deletes all elements from the graphics buffer.
All 'GFXAdd...' functions allow overwriting/reusing an existing element by giving the element ID as the first parameter. If -1 is given instead, a new element is created. The ID of the used/created element is always returned.
There are two ways for using the library:
* For each frame, call `GFXReset()` first, then add all graphics elements (each with -1 as first parameter) and afterwards draw the entire screen using `GFXUpdateDisplay()`
* Create the items once by calling the `GFXAdd...` functions with -1 as a first parameter, store its returned item ID, and use it as a first parameter for subsequent updating of the element. Then, either update the lines affected by the item with GFXUpdateItem(), or the entire screen with GFXUpdateDisplay().


## To-Do
* Add functions to read bitmaps from external memory (likely by a function pointer, so that the user can define the function for reading the bitmap from SPI flash/SD cards/the cloud/whatever)
* Try adding vector fonts (may be difficult without a frame buffer, but they would look nice on such a relatively high-res display)
* Include a Doxygen documentation
* Fix STM32Cube startup code
* More testing
