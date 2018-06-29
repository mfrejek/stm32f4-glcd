##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##
V=1
HAL ?= libopencm3

ifeq ($(HAL),libopencm3)
	LIBOPENCM3_DIR=~/ARMProjects/libopencm3
	INC = -I./include -I./system/include -I./system/include/cmsis

	BINARY = src/main
	OBJS = src/gfxlib.o src/ili9481_drv_libopencm3.o

	LDSCRIPT = ldscripts/stm32f4-discovery.ld
	include libopencm3.include
endif

ifeq ($(HAL),stdperiph)
	INC = -I./include -I./system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/inc -I./system/include/cmsis

	BINARY = src/main
	OBJS = system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.o system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.o system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.o system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.o system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/system_stm32f4xx.o src/gfxlib.o src/ili9481_drv_stdperiph.o system/StdPeriph_Lib/Startup/startup_stm32f407xx.o 

	LDSCRIPT = ldscripts/stm32f407ve.ld

	include stdperiph.include
endif

ifeq ($(HAL),stm32cube)
	INC = -I./include -I./system/STM32F4xx_HAL_Driver/Inc/ -I./system/include/cmsis
	
	BINARY = src/main
	OBJS = system/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.o system/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.o system/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.o system/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.o  system/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.o system/src/cmsis/system_stm32f4xx.o system/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.o src/ili9481_drv_stm32cube.o system/STM32F4xx_HAL_Driver/Src/Startup/startup_stm32f407xx.o src/gfxlib.o

	LDSCRIPT = ldscripts/stm32f407ve.ld

	include stm32cube.include
endif

ifeq ($(HAL),none)
	INC = -I./include -I./system/include -I./system/include/cmsis -I./system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/inc

	BINARY = src/main
	OBJS = src/gfxlib.o src/ili9481_drv_nohal.o system/StdPeriph_Lib/STM32F4xx_StdPeriph_Driver/src/system_stm32f4xx.o system/StdPeriph_Lib/Startup/startup_stm32f407xx.o

	LDSCRIPT = ldscripts/stm32f407ve.ld
	include stdperiph.include
endif
