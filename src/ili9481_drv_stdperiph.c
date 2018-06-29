#include "gfxlib.h"
#include "ili9481_cmd.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"

#ifdef DMA_ENABLE
  DMA_InitTypeDef myDMA;
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
  FSMC_NORSRAMInitTypeDef        FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);  
	
  // FSMC config
  //-----------------------------------------
  // Structure for timing
  //-----------------------------------------  
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 6;
  FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 1;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime = 10;
  FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0;
  FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;

  //-----------------------------------------
  // Structure for Bank-1 / PSRAM-1
  //-----------------------------------------  
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;  
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;  
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;  
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;  
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;  
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure; 
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure; 
  
  // FSMC Config
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /* Enable FSMC_NORSRAM_BANK1 */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
  

  //-----------------------------------------
  // Clock Enable for Port-B, Port-D und Port-E
  //-----------------------------------------
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  
  //-----------------------------------------
  // Init all nedded Pins of Port-D
  //-----------------------------------------
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0,  GPIO_AF_FSMC); // PD0=FSMC_D2   -> DB2
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1,  GPIO_AF_FSMC); // PD1=FSMC_D3   -> DB3
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4,  GPIO_AF_FSMC); // PD4=FSMC_NOE  -> RD
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5,  GPIO_AF_FSMC); // PD5=FSMC_NWE  -> WR
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource7,  GPIO_AF_FSMC); // PD7=FSMC_NE1  -> CS
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8,  GPIO_AF_FSMC); // PD8=FSMC_D13  -> DB15
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,  GPIO_AF_FSMC); // PD9=FSMC_D14  -> DB16
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC); // PD10=FSMC_D15 -> DB17 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC); // PD13=FSMC_A18
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC); // PD14=FSMC_D0  -> DB0
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC); // PD15=FSMC_D1  -> DB1
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  
  GPIO_Init(GPIOD, &GPIO_InitStructure);   
  
  //-----------------------------------------
  // Init all nedded Pins of Port-E
  //-----------------------------------------
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7,  GPIO_AF_FSMC); // PE7=FSMC_D4   -> DB4
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8,  GPIO_AF_FSMC); // PE8=FSMC_D5   -> DB5  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_FSMC); // PE9=FSMC_D6   -> DB6
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC); // PE10=FSMC_D7  -> DB7
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC); // PE11=FSMC_D8  -> DB10
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC); // PE12=FSMC_D9  -> DB11
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC); // PE13=FSMC_D10 -> DB12
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC); // PE14=FSMC_D11 -> DB13
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC); // PE15=FSMC_D12 -> DB14

  // Structure for Port-E
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 |
                                GPIO_Pin_14 | GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  // Port E config
  GPIO_Init(GPIOE, &GPIO_InitStructure);

	
	#ifdef DMA_ENABLE
		//Configure DMA if enabled
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_StructInit(&myDMA);
    myDMA.DMA_DIR = DMA_DIR_MemoryToMemory;
    myDMA.DMA_Mode = DMA_Mode_Normal;
    myDMA.DMA_Channel = DMA_Channel_0;
    myDMA.DMA_MemoryInc = DMA_MemoryInc_Disable;
    myDMA.DMA_PeripheralInc = DMA_PeripheralInc_Enable;										//Must be enabled for some reason
    myDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    myDMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    myDMA.DMA_Priority = DMA_Priority_High;
    myDMA.DMA_BufferSize = SCREEN_WIDTH;
    myDMA.DMA_Memory0BaseAddr = LCD_RAM_ADR;
    DMA_Init(DMA2_Stream7, &myDMA);
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
      while(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == RESET) {};
      DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
    }
    myDMA.DMA_PeripheralBaseAddr = (uint32_t)&currline[bufindex];
    DMA_Init(DMA2_Stream7, &myDMA);
    DMA_Cmd(DMA2_Stream7,ENABLE);
    if(bufindex == 0) {
      bufindex = SCREEN_WIDTH;
    } else {
      bufindex = 0;
    }
  #else
    for(int i=0;i<SCREEN_WIDTH;i++) P_9481_WriteData(currline[i]);
  #endif
}
