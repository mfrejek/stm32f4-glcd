#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_ll_fsmc.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"

#include "gfxlib.h"
#include "ili9481_cmd.h"

#ifdef DMA_ENABLE
  DMA_HandleTypeDef myDMA;
#endif

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
  FSMC_NORSRAM_InitTypeDef        FSMC_NORSRAMInitStructure;
  FSMC_NORSRAM_TimingTypeDef      FSMC_NORSRAMTimingInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_FSMC_CLK_ENABLE();  //Enable FSMC clock
	
  // FSMC config
  //-----------------------------------------
  // Structure for timing
  //-----------------------------------------  
  FSMC_NORSRAMTimingInitStructure.AddressSetupTime = 6;
  FSMC_NORSRAMTimingInitStructure.AddressHoldTime = 1;
  FSMC_NORSRAMTimingInitStructure.DataSetupTime = 10;
  FSMC_NORSRAMTimingInitStructure.BusTurnAroundDuration = 0;
  FSMC_NORSRAMTimingInitStructure.CLKDivision = 0;
  FSMC_NORSRAMTimingInitStructure.DataLatency = 0;
  FSMC_NORSRAMTimingInitStructure.AccessMode = FSMC_ACCESS_MODE_B;

  //-----------------------------------------
  // Structure for Bank-1 / PSRAM-1
  //-----------------------------------------  
  FSMC_NORSRAMInitStructure.NSBank = FSMC_NORSRAM_BANK1;
  FSMC_NORSRAMInitStructure.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  FSMC_NORSRAMInitStructure.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  FSMC_NORSRAMInitStructure.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  FSMC_NORSRAMInitStructure.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  FSMC_NORSRAMInitStructure.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;   
  FSMC_NORSRAMInitStructure.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;  
  FSMC_NORSRAMInitStructure.WrapMode = FSMC_WRAP_MODE_DISABLE;  
  FSMC_NORSRAMInitStructure.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;  
  FSMC_NORSRAMInitStructure.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;  
  FSMC_NORSRAMInitStructure.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;   
  FSMC_NORSRAMInitStructure.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;  
  FSMC_NORSRAMInitStructure.WriteBurst = FSMC_WRITE_BURST_DISABLE;  
  
  // FSMC Config
  FSMC_NORSRAM_Init(FSMC_Bank1, &FSMC_NORSRAMInitStructure); 
	FSMC_NORSRAM_Timing_Init(FSMC_Bank1, &FSMC_NORSRAMTimingInitStructure,0);

  /* Enable FSMC_NORSRAM_BANK1 */
	FSMC_NORSRAM_WriteOperation_Enable(FSMC_Bank1, 0);
	FSMC_Bank1->BTCR[0] |= 1;
  

  //-----------------------------------------
  // Clock Enable for Port-B, Port-D und Port-E
  //-----------------------------------------
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
  
  
  //-----------------------------------------
  // Init all nedded Pins of Port-D
  //-----------------------------------------
	
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
  
  //-----------------------------------------
  // Init all nedded Pins of Port-E
  //-----------------------------------------
	GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	
	#ifdef DMA_ENABLE
		//Configure DMA if enabled
    __HAL_RCC_DMA2_CLK_ENABLE();
     //memset(&myDMA.Init,0,sizeof(DMA_InitStructure));
     myDMA.Init.Channel = DMA_CHANNEL_0;
     myDMA.Init.Direction = DMA_MEMORY_TO_MEMORY;
     myDMA.Init.MemInc = DMA_MINC_DISABLE;										//Disable FSMC address increment
     myDMA.Init.PeriphInc = DMA_PINC_ENABLE;										
     myDMA.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
     myDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
     myDMA.Init.Priority = DMA_PRIORITY_HIGH;
     myDMA.Instance = DMA2_Stream7;
     HAL_DMA_Init(&myDMA);
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
    HAL_DMA_PollForTransfer(&myDMA,HAL_DMA_FULL_TRANSFER, 100);
    HAL_DMA_Start(&myDMA,(uint32_t)&currline[bufindex], (uint32_t)LCD_RAM_ADR, SCREEN_WIDTH);
    if(bufindex == 0) {
      bufindex = SCREEN_WIDTH;
    } else {
      bufindex = 0;
    }
  #else
    for(int i=0;i<SCREEN_WIDTH;i++) P_9481_WriteData(currline[i]);
  #endif
}

/**
* @brief Wait until DMA operation has completed
* @retval none
*/
void P_9481_WaitForDMACompletion(void)
{
	#ifdef DMA_ENABLE
    HAL_DMA_PollForTransfer(&myDMA, HAL_DMA_FULL_TRANSFER, 100);
	#endif
}
