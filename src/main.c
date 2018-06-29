/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
//#include "diag/Trace.h"

#include "gfxlib.h"
#include "kitten.h"
#ifdef LIBOPENCM3           //check if complied with libopencm3 or StdPeriph/STM32Cube
	#include "libopencm3/stm32/timer.h"
    #include "libopencm3/stm32/rcc.h"
#else
	#include "stm32f4xx.h"
    #ifdef STM32CUBE
        #include "system_stm32f4xx.h"
        #include "stm32f4xx_hal.h"
        #include "stm32f4xx_hal_rcc.h"
        #include "stm32f4xx_hal_flash.h"
        #include "stm32f4xx_hal_pwr_ex.h"
    #endif
#endif
// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define DEMO1_END  140
#define TROLL_END  200

#ifdef STM32CUBE
/**
  * @brief  System Clock Configuration. Only needed for STM32Cube HAL
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config  (void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
#endif

void DelayTime(void) {
	volatile uint32_t i = 0;
	while(i < 5000000) {
		i++;
	}
}

int main(int argc, char* argv[])
{
  #ifdef STM32F4     //check if complied with libopencm3 or StdPeriph/STM32Cube
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  #elif defined(STM32CUBE)
    SystemClock_Config();
    SystemCoreClockUpdate();
  #endif
  // At this stage the system clock should have already been configured
  // at high speed.

	uint32_t loopctr = 0;
	  uint16_t trollcolor = 0;
	  char outstr[20];
	  strcpy(outstr, "CLK:   M; FPS: ");
		//SystemCoreClockUpdate();
		//LCD init
		P_9481_HardwareInit();
		P_9481_DisplayInit();

	  //TIM2 for performance measurement
	  #ifdef STM32F4            //check if complied with libopencm3 or StdPeriph/STM32Cube
		rcc_periph_clock_enable(RCC_TIM2);
		TIM2_CR2 = 0;
		TIM2_SMCR = 0;
		TIM2_DIER = 0;
		TIM2_CCMR1 = 0;
		TIM2_CCMR2 = 0;
		TIM2_CCER = 0;
		TIM2_EGR = 0;
		TIM2_ARR = 0xFFFFFFFF;
		TIM2_PSC = 0;  //Prescaler 1; run at full 84 MHz clock
		TIM2_CNT = 0;  //Reset count register
		TIM2_CR1 = 0;  //Disable counter (reset CEN bit)
	  #else
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		TIM2->CR2 = 0;
		TIM2->SMCR = 0;
		TIM2->DIER = 0;
		TIM2->CCMR1 = 0;
		TIM2->CCMR2 = 0;
		TIM2->CCER = 0;
		TIM2->EGR = 0;
		TIM2->ARR = 0xFFFFFFFF;
		TIM2->PSC = 0;  //Prescaler 1; run at full 84 MHz clock
		TIM2->CNT = 0;  //Reset count register
		TIM2->CR1 = 0;  //Disable counter (reset CEN bit)
	  #endif
	  // Set most registers to 0


		// Some demo graphics
		while(1) {
	    GFXReset(); // Clear rendering buffer. Won't affect display yet
	    if(loopctr > DEMO1_END)
	    {
	      GFXAddColorBitmap(-1, 0, 0, 320, 480, (uint16_t*)kitten_large, 0, 0);
	    }
	    if((loopctr < DEMO1_END) || (loopctr > TROLL_END)) {
	      GFXAddText(-1, "ILI9481 Demo on STM32F4", 0, 0, COLOR_ORANGE, MEDIUM, 1, 0x0000);
	      if(loopctr > 5) GFXAddText(-1, "Size: ~7k code, 28k font, 9k RAM", 0, 24, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 5) GFXAddText(-1, "Supports:", 0, 40, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 10) GFXAddText(-1, "- Rectangles (with border)", 0, 56, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 20) GFXAddText(-1, "- Texts (4 sizes + transparency)", 0, 72, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 40) GFXAddText(-1, "- Circles (with border)", 0, 88, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 60) GFXAddText(-1, "- Arcs", 0, 104, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 80) GFXAddText(-1, "- Polygons (max. 8 points)", 0, 120, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 100) GFXAddText(-1, "- RGB 565 Color Bitmaps", 0, 136, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 103) GFXAddText(-1, "- Monochrome Bitmaps", 0, 152, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 110) GFXAddText(-1, "No frame buffer needed", 0, 168, COLOR_WHITE, SMALL, 1, 0x0000);
	      if(loopctr > 112) GFXAddText(-1, "Needs 50 B RAM per GFX element;", 0, 184, COLOR_LIGHTGREEN, SMALL, 1, 0x0000);
	      if(loopctr > 114) GFXAddText(-1, "max element number configurable", 0, 200, COLOR_LIGHTGREEN, SMALL, 1, 0x0000);
	      if(loopctr > 120) GFXAddText(-1, "The Internet is made of cats,", 0, 216, COLOR_RED, SMALL, 1, 0x0000);
	      if(loopctr > 125) GFXAddText(-1, "the embedded world will be soon", 0, 232, COLOR_RED, SMALL, 1, 0x0000);
	    }
	    if(((loopctr > 10) && (loopctr <= 40)) || (loopctr > TROLL_END)) {
	      //Rect demo
	      GFXAddRect(-1, 50, 240, 220, 100, COLOR_MAGENTA, 4, COLOR_GREEN, 0);
	      GFXAddRect(-1, 50, 400, 100, 30, COLOR_GREEN, 0, 0, 0);
	      // Add text demo to show transparency
	      if(loopctr > 20) {
	        GFXAddText(-1, "Some large red text", 10, 235, COLOR_RED, LARGE, 1, 0x0000);
	        GFXAddText(-1, "Some medium blue text", 10, 275, COLOR_BLUE, MEDIUM, 1, 0x0000);
	        GFXAddText(-1, "Some small green text", 20, 300, COLOR_GREEN, SMALL, 1, 0x0000);
	        GFXAddText(-1, "Some very small white text", 20, 320, COLOR_WHITE, XSMALL, 0, 0x0000);
	      }
	    }
	    if(((loopctr > 40) && (loopctr <= 60)) || (loopctr > TROLL_END)) {
	      // Circle demo
	      GFXAddCircle(-1, 160, 360, 100, COLOR_YELLOW, 10, COLOR_GRAY, 0);
	      if(loopctr > 50) GFXAddCircle(-1, 231, 431, 15, 0, 3, COLOR_RED, 1);
	    }

	    if(((loopctr > 60) && (loopctr <= 100)) || (loopctr > TROLL_END)) {
	      GFXAddArc(-1, 160, 360, 100, 80, 2730, 16380, COLOR_LIGHTGREEN);
	      if(loopctr > 65) GFXAddArc(-1, 160, 360, 100, 80, 0, 2730, COLOR_RED);
	      if(loopctr > 80) {
	        int16_t px[8], py[8];
	        // Arrow
	        px[0] = 0;
	        py[0] = 0;
	        px[1] = -10;
	        py[1] = 25;
	        px[2] = 0;
	        py[2] = 15;
	        px[3] = 10;
	        py[3] = 25;
	        GFXAddPoly(-1, 160, 280, px, py, 4, COLOR_ORANGE);
	        // Stop sign; each edge 32 long
	        px[0] = -16;
	        py[0] = 0;
	        px[1] = 16;
	        py[1] = 0;
	        px[2] = 38;
	        py[2] = 22;
	        px[3] = 38;
	        py[3] = 54;
	        px[4] = 16;
	        py[4] = 77;
	        px[5] = -16;
	        py[5] = 77;
	        px[6] = -38;
	        py[6] = 54;
	        px[7] = -38;
	        py[7] = 22;
	        GFXAddPoly(-1, 160, 360, px, py, 8, COLOR_RED);
	        GFXAddText(-1, "TEST", 128, 384, COLOR_WHITE, LARGE, 1, 0x0000);
	      }
	    }
	    //GFXAddColorBitmap(120, 24, 200, 352, (uint16_t*)kitten_large, 0, 0);
	    if(loopctr > TROLL_END) {
	      if(trollcolor < 32) trollcolor++; //Blue
	      else if(trollcolor < 2047) trollcolor+=32;//Green
	      else if(trollcolor < 63488) trollcolor+=2048; //Red
	      else trollcolor=0;
	      GFXAddMonochromeBitmap(-1, 0, 80, 320, 320, (uint16_t*)trollface, trollcolor, 0, 1);
	    }
	    GFXAddText(-1, outstr, 0, 463, COLOR_GREEN, SMALL, 1, 0x0000);
		#ifdef STM32F4     //check if complied with libopencm3 or StdPeriph/STM32Cube
			TIM2_CNT = 0;  //Reset count register
			TIM2_CR1 = 1;  //Enable counter (set CEN bit)
		#else
			TIM2->CNT = 0;  //Reset count register
			TIM2->CR1 = 1;  //Enable counter (set CEN bit)
		#endif
	    GFXUpdateDisplay();
		#ifdef STM32F4     //check if complied with libopencm3 or StdPeriph/STM32Cube
			TIM2_CR1 = 0;  //Disable counter (reset CEN bit)
			print_fixed_str((rcc_ahb_frequency / 1000000), 3, 0, &outstr[4], 1, 0);
		    print_fixed_str(((rcc_ahb_frequency * 5) / TIM2_CNT), 6, 1, &outstr[14], 1, 1);
		#else
			TIM2->CR1 = 0;  //Disable counter (reset CEN bit)
			print_fixed_str((SystemCoreClock / 1000000), 3, 0, &outstr[4], 1, 0);
		    print_fixed_str(((SystemCoreClock * 5) / TIM2->CNT), 6, 1, &outstr[14], 1, 1);
		#endif
	    loopctr++;
        DelayTime();
		}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
