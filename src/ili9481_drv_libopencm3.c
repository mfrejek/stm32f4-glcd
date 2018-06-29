#include "gfxlib.h"
#include "ili9481_cmd.h"
//#include "stm32f4xx.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/fsmc.h>


/**
* @brief Send a command to the LCD. Selecting between commands and data is done via the address to which is written; since the FSMC is mapped into the CPU's address range
* @param cmd: Command to be sent
* @retval none
*/
static inline void P_9481_WriteCmd(uint16_t cmd)
{
  LCD_REG = cmd; 
}

/**
* @brief Send data to the LCD. Selecting between commands and data is done via the address to which is written; since the FSMC is mapped into the CPU's address range
* @param cmd: Command to be sent
* @retval none
*/
static inline void P_9481_WriteData(uint16_t data)
{
  LCD_RAM = data; 
}

/**
* @brief Initialize GPIO, FSMC and DMA for writing to the display. Must be called before calling any other display-related functions
* @getval none
*/
void P_9481_HardwareInit(void)
{ 
  rcc_periph_clock_enable(RCC_FSMC);
  FSMC_BTR(0) = (10 << 8) | (1 << 4) | 6;
  FSMC_BCR(0) = FSMC_BCR_WREN | FSMC_BCR_MWID | FSMC_BCR_MBKEN;  //FSMC in 16 bit mode, write enable + Memory bank Enable;

  //FSMC_Bank1->BTCR[1] = (10 << 8) | (1 << 4) | 6;
  //FSMC_Bank1->BTCR[0] = FSMC_BCR1_WREN | FSMC_BCR1_MWID_0 | FSMC_BCR1_MBKEN;  //FSMC in 16 bit mode, write enable + Memory bank Enable
  

  //-----------------------------------------
  // Clock Enable for Port-B, Port-D und Port-E
  //-----------------------------------------
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_GPIOE);
  
  //-----------------------------------------
  // Init all nedded Pins of Port-D
  //-----------------------------------------
  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOD, GPIO_AF12, GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO13 | GPIO14 | GPIO15);

  //GPIOD->AFR[0] |= (0x0CU << 28U) | (0x0C << 20) | (0x0C << 16) | (0x0C << 4) | (0x0C); //PD0,1,4,5,7
  //GPIOD->AFR[1] |= (0x0CU << 28U) | (0x0C << 24) | (0x0C << 20) | (0x0C << 8) | (0x0C << 4) | (0x0C); //PD8,9,10,13,14,15
  // All used GPIOs to maximum speed
  //GPIOD->OSPEEDR |= (3U << 30U) | (3U << 28U) | (3U << 26U) | (3U << 20U) | (3U << 18U) | (3U << 16U) | (3U << 14U) | (3U << 10U) | (3U << 8U) | (3U << 2U) | 3U;
  // Ensure outputs are open drain
	//GPIOD->OTYPER &= ~((1<<15) | (1<<14) | (1<<13) | (1<<10) | (1<<9) | (1<<8) | (1<<7) | (1<<5) | (1<<4) | (1<<1) | 1);
  //Rerset mode
  //GPIOD->MODER &= ~((3U << 30U) | (3U << 28U) | (3U << 26U) | (3U << 20U) | (3U << 18U) | (3U << 16U) | (3U << 14U) | (3U << 10U) | (3U << 8U) | (3U << 2U) | 3U);
  // Set to alternate mode
  //GPIOD->MODER |= (2U << 30U) | (2U << 28U) | (2U << 26U) | (2U << 20U) | (2U << 18U) | (2U << 16U) | (2U << 14U) | (2U << 10U) | (2U << 8U) | (2U << 2U) | 2U;
    
  
  //-----------------------------------------
  // Init all nedded Pins of Port-E
  //-----------------------------------------
  
  gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
  gpio_set_af(GPIOE, GPIO_AF12, GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

/*  GPIOE->AFR[0] |= (0x0CU << 28U); //PE7
  GPIOE->AFR[1] |= (0x0CU << 28U) | (0x0C << 24) | (0x0C << 20) | (0x0C << 16) | (0x0C << 12) | (0x0C << 8) | (0x0C << 4) | (0x0C); //PE8,9,10,11,12,13,14,15
  // All used GPIOs to maximum speed
  GPIOE->OSPEEDR |= (3U << 30U) | (3U << 28U) | (3U << 26U) | (3U << 24U) | (3U << 22U) |(3U << 20U) | (3U << 18U) | (3U << 16U) | (3U << 14U);
  // Ensure outputs are open drain
	GPIOE->OTYPER &= ~((1<<15) | (1<<14) | (1<<13) | (1<<12) | (1<<11) | (1<<10) | (1<<9) | (1<<8) | (1<<7));
  //Rerset mode
  GPIOE->MODER &= ~((3U << 30U) | (3U << 28U) | (3U << 26U) | (3U << 24U) | (3U << 22U) |(3U << 20U) | (3U << 18U) | (3U << 16U) | (3U << 14U));
  // Set to alternate mode
  GPIOE->MODER |= (2U << 30U) | (2U << 28U) | (2U << 26U) | (2U << 24U) | (2U << 22U) |(2U << 20U) | (2U << 18U) | (2U << 16U) | (2U << 14U);
	*/
	#ifdef DMA_ENABLE
		//Configure DMA if enabled
    rcc_periph_clock_enable(RCC_DMA2);
    dma_stream_reset(DMA2, DMA_STREAM7);
    dma_set_priority(DMA2, DMA_STREAM7, DMA_SxCR_PL_LOW);
    dma_set_memory_size(DMA2, DMA_STREAM7, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM7, DMA_SxCR_PSIZE_16BIT);
    dma_enable_peripheral_increment_mode(DMA2, DMA_STREAM7);
    dma_set_transfer_mode(DMA2, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_MEM);
    dma_set_memory_address(DMA2, DMA_STREAM7, (uint32_t) LCD_RAM_ADR); // Target address
    dma_set_number_of_data(DMA2, DMA_STREAM7, SCREEN_WIDTH); //Number of elements
    dma_channel_select(DMA2, DMA_STREAM7, DMA_SxCR_CHSEL_0);
    /*
    DMA2_Stream7->CR = 0; //Disable DMA channel to be able to reconfigure it
    DMA2_Stream7->FCR = 0; //Disable FIFO
    DMA2_Stream7->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_PINC | DMA_SxCR_DIR_1; //16 bit data size, MemToMem, Memory increment
    DMA2_Stream7->NDTR = SCREEN_WIDTH;
    DMA2_Stream7->M0AR = LCD_RAM_ADR;
	*/
	#endif
}

/**
* @brief Set the gamma curve for the display. This function is platform + HAL dependent
* @param Refer to ILI9481 datasheet
* @retval none
*/

void P_9481_SetGammaCurve(uint8_t KP[6], uint8_t RP[2], uint8_t VRP[2], uint8_t KN[6], uint8_t RN[2], uint8_t VRN[2])
{
  P_9481_WriteCmd(CMD_SET_GAMMA);
  P_9481_WriteData(KP[0] | (KP[1] << 4));
  P_9481_WriteData(KP[2] | (KP[3] << 4));
  P_9481_WriteData(KP[4] | (KP[5] << 4));
  P_9481_WriteData(RP[0] | (RP[1] << 4));
  P_9481_WriteData(VRP[0]);
  P_9481_WriteData(VRP[1]);
  
  P_9481_WriteData(KN[0] | (KN[1] << 4));
  P_9481_WriteData(KN[2] | (KN[3] << 4));
  P_9481_WriteData(KN[4] | (KN[5] << 4));
  P_9481_WriteData(RN[0] | (RN[1] << 4));
  P_9481_WriteData(VRN[0]);
  P_9481_WriteData(VRN[1]);
}

void P_9481_DisplayInit(void) 
{
	bgColor32 = 0;
	P_9481_WriteCmd(CMD_EXIT_SLEEP);		//Exit Sleep
	P_9481_Delay(1000000);
	P_9481_WriteCmd(CMD_ENTER_NORMAL_MODE);		//Enter Normal Mode
	//Power settings. Read datasheet before adjusting those; incorrect settings may damage the display!
	P_9481_WriteCmd(CMD_POWER_SETTING);
  P_9481_WriteData(VCI1_100); //Vci1 = 1.00 * Vci = 2.50V if INT_VCI_2V5_REF_EN is set
	P_9481_WriteData(STUP_PON | STUP_VGH5_VGL5); // Vgh = 5*Vci (+12.5V), Vgl = -5*Vci (-12.5V), Enable voltage converter
	P_9481_WriteData(INT_VCI_2V5_REF_EN | VREGOUT1_180); // Enable internal reference (Vci = 2.5V); set VREGOUT1 to 1.8*Vci(4.5V)
	// VCom (more power settings)
	P_9481_WriteCmd(CMD_VCOM_CTRL);
  P_9481_WriteData(0);  //Use VCM from this register, as configured in the following 2 bytes
  // Adjust contrast with these values; already pretty good at minimum voltages
	P_9481_WriteData(CALC_VCOMH_MUL(685));   //VCOMH = 0.685 * VREGOUT1 (Range: 0.685...1.000)
	P_9481_WriteData(CALC_VCOM_AC_MUL(700)); //VCOM AC amplitude = 0.700 * VREGOUT1 (Range: 0.700...1.320)
	// power setting for normal mode
	P_9481_WriteCmd(CMD_NORM_PWR_SETTING);
  P_9481_WriteData(OPV_GAMMA100_SRC100);            //Gamma and source OpAmps at 100% current => Maximum LCD switching speed. May be reduced if power consumption is an issue
	P_9481_WriteData(DCDC1_CKDIV_4 | DCDC2_CKDIV_16); //DC/DC converter clock dividers; selected somewhat arbitrarily
	// Panel driver setting
	P_9481_WriteCmd(CMD_PANELDRV_CFG);
  P_9481_WriteData(0);                        // Not two halves and not top-down
	P_9481_WriteData(VRAM_CALC_LINES(SCREEN_HEIGHT));     // 480 px height
	P_9481_WriteData(0);                        // Start at first line
	P_9481_WriteData(0);                        // PTV bit not set
  P_9481_WriteData(PTS_FULLSPD_NORM);         // Step up clock not divided
	P_9481_WriteData(ISC_SCANCYCLE_3);          //3 frames ISC scan cycle
	// Normal mode timing
  P_9481_WriteCmd(CMD_SET_NORMAL_TIMING);
	P_9481_WriteData(TIMING_CKDIV_1);
	P_9481_WriteData(0x10);                     //16 clocks line period
	P_9481_WriteData(0x22);                     //Front and back porch each 2 cycles
	//Idle timing
	P_9481_WriteCmd(CMD_SET_IDLE_TIMING);
	P_9481_WriteData(TIMING_CKDIV_1);
	P_9481_WriteData(0x10);             //16 clocks line period
	P_9481_WriteData(0x22);             //Front and back porch each 2 cycles
	
	//85 Hz frame rate
	P_9481_WriteCmd(CMD_SET_FRAMERATE);
  P_9481_WriteData(FRAMERATE_85HZ);
	// Interface setup; all pins active high
	P_9481_WriteCmd(CMD_CFG_INTERFACE);
  P_9481_WriteData(IFACE_CFG_VSPL | IFACE_CFG_HSPL | IFACE_CFG_EPL | IFACE_CFG_DPL);
	// RGB and frame memory access
	P_9481_WriteCmd(CMD_CONFIG_RGB);
  P_9481_WriteData(0);  //Ignore additional bytes
	P_9481_WriteData(0);  //TE signal not needed
	P_9481_WriteData(0);  //1 frame GRAM write cycle
	P_9481_WriteData(RGB565_MSB_AS_LSB | DFM_DBI_B);
	// Color depth/Orientation
	P_9481_WriteCmd(CMD_SET_PIXELFMT);
  P_9481_WriteData(PIXELFMT_16BPP | (PIXELFMT_16BPP << 4));	//64K colors
	P_9481_WriteCmd(CMD_SET_ADDRMODE);
  P_9481_WriteData(ADDRMODE_HFLIP); // Horizontal flip
	//Inverting mode needed fore some reason
	P_9481_WriteCmd(CMD_COLOR_INVERT);
  
  // Relocate this
  uint8_t KP[6], RP[2], VRP[2], KN[6], RN[2], VRN[2];
  memset(KP, 0, 6);
  memset(RP, 0, 2);
  memset(VRP, 0, 2);
  memset(KN, 0, 6);
  memset(RN, 0, 2);
  memset(VRN, 0, 2);
  VRP[0] = 8;
  VRP[1] = 16;
  RP[0] = RP[1] = RN[0] = RN[1] = 4;
  VRN[0] = 8;
  VRN[1] = 16;
  P_9481_SetGammaCurve(KP, RP, VRP, KN, RN, VRN);
	//Display on!
	P_9481_Delay(1000000);
	P_9481_WriteCmd(CMD_DISPLAY_ON);
	P_9481_Delay(1000000);
}

/**
* @brief Set the range of lines to be updated. This function is platform + HAL dependent
* @param startline: First line to update [0...SCREEN_HEIGHT-2]
* @param endline: Last line to update [1...SCREEN_HEIGHT-1]
* @retval none
*/
void P_9481_SetUpdateLineRegion(uint16_t startline, uint16_t endline)
{
  P_9481_WriteCmd(CMD_SET_PAGEADDR);    //"Set_page_address" command
  P_9481_WriteData(startline >> 8);     //Start line
  P_9481_WriteData(startline & 0xFF);
  P_9481_WriteData(endline >> 8);       //End line
  P_9481_WriteData(endline & 0xFF);
  P_9481_WriteCmd(CMD_START_MEM_WRITE);
}
/**
* @brief Output a completely rendered line (stored in the global currline array) to the display. This function is platform + HAL dependent
* @param linenum: Number of line [0...SCREEN_HEIGHT-1]
* @retval none
*/
void P_9481_OutputScreenLine(uint16_t linenum)
{
  #ifdef DMA_ENABLE
    if(linenum > 0) {
      while(!(dma_get_interrupt_flag(DMA2, DMA_STREAM7, DMA_TCIF))) {};
      dma_clear_interrupt_flags(DMA2, DMA_STREAM7, DMA_TCIF);
    }
    dma_set_peripheral_address(DMA2, DMA_STREAM7, (uint32_t) &currline[bufindex]);
    dma_enable_stream(DMA2, DMA_STREAM7);
    /*
    DMA2_Stream7->PAR = (uint32_t)&currline[bufindex];  //Set DMA address to current buffer
    DMA2_Stream7->CR |= DMA_SxCR_EN;  //Start DMA
    */
    if(bufindex == 0) {
      bufindex = SCREEN_WIDTH;
    } else {
      bufindex = 0;
    }
  #else
    for(int i=0;i<SCREEN_WIDTH;i++) P_9481_WriteData(currline[i]);
  #endif
}
