/**
* ****************************************************************************************************************
*  @file    ili9481_cmd.h
*  @author  Markus Frejek
*  @date    22.01.2018
*  @brief   ILI9481 register names and commands from datasheet. Not entirely complete.
*/

#define CMD_ENTER_SLEEP         0x10		//!< Exit sleep mode
#define CMD_EXIT_SLEEP		      0x11		//!< Exit sleep mode
#define CMD_ENTER_NORMAL_MODE		0x13		//!< Enter normal mode
#define CMD_COLOR_NONINVERT     0x20		//!< Display colors not inverted
#define CMD_COLOR_INVERT        0x21		//!< Display colors inverted
#define CMD_DISPLAY_OFF         0x28		//!< Display off (black, but controller stays active)
#define CMD_DISPLAY_ON          0x29		//!< Display on

#define CMD_SET_COLADDR         0x2A		//!< Set column address for partial update
#define CMD_SET_PAGEADDR        0x2B		//!< Set page address (line numbers) for partial update
#define CMD_START_MEM_WRITE     0x2C		//!< Start frame buffer write at selected address

#define CMD_SET_ADDRMODE        0x36
  #define ADDRMODE_VFLIP          0x01  //!< Flip display vertically
  #define ADDRMODE_HFLIP          0x02  //!< Flip display horizontally
  #define ADDRMODE_BGR            0x08  //!< BGR instead of RGB color mode
  #define ADDR_INVERT_LINE        0x10  //!< Reverse line address order
  #define ADDR_INVERT_PAGEORDER   0x20  //!< Reverse page/column order
  #define ADDR_INVERT_COLUMN      0x40  //!< Reverse column address order
  #define ADDR_INVERT_PAGE        0x80  //!< Reverse page address order

#define CMD_SET_PIXELFMT        0x3A    //!< Set pixel format (bit/pixel)
  // Pixel format parameters. This libary only works with PIXELFMT_16BPP!
  #define PIXELFMT_3BPP         0x01    //!< 3 bpp = 8 colors
  #define PIXELFMT_16BPP        0x05    //!< 16 bpp = 65536 colors
  #define PIXELFMT_18BPP        0x07    //!< 18 bpp = 262144 colors

#define CMD_CONFIG_RGB          0xB3    //!< Config RGB mode (565 or 18 bit) and frame buffer access
//CMD_CONFIG_RGB parameters
  #define DFM_DBI_B               0x01    //!< DFM bit for CMD_CONFIG_RGB
  /* RGB 565 config
  The LCD internally has 6 bits per color => 18 bits in total, but should be controlled with 16 bits.
  Hence, only 5 bits are used for R and B. For generating the "missing" bit (LSB), 3 possibilites exist:
  */
  #define RGB565_MSB_AS_LSB       0x00   //!< Use MSB as LSB
  #define RGB565_LSB_ZERO         0x01   //!< LSB always 0
  #define RGB565_LSB_ONE          0x02   //!< LSB always 1
  
#define CMD_PANELDRV_CFG        0xC0    //!< Panel driver config
  //First byte
  #define TFT_2HALVES         0x08        //!< SM bit => TFT split in 2 halves
  #define TFT_TOPDOWN         0x10        //!< GS bit => Reverse line order for LCD driver
  //Second byte: # of LCD lines; set in driver
  #define VRAM_CALC_LINES(X)  ((X / 8) - 1)
  //Third byte: Scan start position: Normally 0, set in driver
  // Fourth byte: PTV bit
  #define HALT_VCOM_NONDISP    0x01       //!< Halt VCOM output in non-displayed area
  // Fifth byte
  #define DRV_NONDISP_NDL      0x10       //!< Drive positive (normally white?) in non-display area
  #define PTS_FULLSPD_NORM     0x00       //!< Normal PTS setting with full step-up clock
  #define PTS_FULLSPD_GND      0x02       //!< TFT sources at GND with full step-up clock
  #define PTS_FULLSPD_HIZ      0x03       //!< TFT sources open/high impedance with full step-up clock
  #define PTS_HALFSPD_NORM     0x04       //!< Normal PTS setting with half step-up clock
  #define PTS_HALFSPD_GND      0x06       //!< TFT sources at GND with half step-up clock
  #define PTS_HALFSPD_HIZ      0x07       //!< TFT sources open/high impedance with half step-up clock
  // Sixth byte
  #define DRV_NONDISP_PTG      0x10       //!< Interval scan in non-dispaly area
  //ISC scan cycles
  #define ISC_SCANCYCLE_3       0x01       //!< 3 frames scan cycle
  #define ISC_SCANCYCLE_5       0x02       //!< 5 frames scan cycle
  #define ISC_SCANCYCLE_7       0x03       //!< 7 frames scan cycle
  #define ISC_SCANCYCLE_9       0x04       //!< 9 frames scan cycle
  #define ISC_SCANCYCLE_11      0x05       //!< 11 frames scan cycle
  #define ISC_SCANCYCLE_13      0x06       //!< 13 frames scan cycle
  #define ISC_SCANCYCLE_15      0x07       //!< 15 frames scan cycle
  #define ISC_SCANCYCLE_17      0x08       //!< 17 frames scan cycle
  #define ISC_SCANCYCLE_19      0x09       //!< 19 frames scan cycle
  #define ISC_SCANCYCLE_21      0x0A       //!< 21 frames scan cycle
  #define ISC_SCANCYCLE_23      0x0B       //!< 23 frames scan cycle
  #define ISC_SCANCYCLE_25      0x0C       //!< 25 frames scan cycle
  #define ISC_SCANCYCLE_27      0x0D       //!< 27 frames scan cycle
  #define ISC_SCANCYCLE_29      0x0E       //!< 29 frames scan cycle
  #define ISC_SCANCYCLE_31      0x0F       //!< 31 frames scan cycle
  
#define CMD_SET_NORMAL_TIMING  0xC1   //!< Display timing in normal mode
#define CMD_SET_PARTIAL_TIMING 0xC2   //!< Display timing in partial mode
#define CMD_SET_IDLE_TIMING    0xC3   //!< Display timing in idle mode
  //First byte for timings
  #define TIMING_CKDIV_1        0x00       //!< Divide internal clock by 1
  #define TIMING_CKDIV_2        0x01       //!< Divide internal clock by 2
  #define TIMING_CKDIV_4        0x02       //!< Divide internal clock by 4
  #define TIMING_CKDIV_8        0x03       //!< Divide internal clock by 8
  
  #define TIMING_BC0            0x10       //!< BC0 bit; 1 = line inversion waveform, 0 = frame inversion
  // Second byte: Clocks per line. Must be between 0x10 and 0x1F
  // Third byte: Front and back porch periods (2-15). Each 4 bit; front porch timing is high and back porch timing is low nibble

#define CMD_SET_FRAMERATE      0xC5   //!< Set frame rate
  //First byte
  #define FRAMERATE_125HZ       0x00  //!< 125 Hz
  #define FRAMERATE_100HZ       0x01  //!< 100 Hz
  #define FRAMERATE_85HZ        0x02  //!<  85 Hz
  #define FRAMERATE_72HZ        0x03  //!<  72 Hz
  #define FRAMERATE_56HZ        0x04  //!<  56 Hz
  #define FRAMERATE_50HZ        0x05  //!<  50 Hz
  #define FRAMERATE_45HZ        0x06  //!<  45 Hz
  #define FRAMERATE_42HZ        0x07  //!<  72 Hz
  
#define CMD_CFG_INTERFACE     0xC6   //!< Interface config
  // First byte
  #define IFACE_CFG_DPL         0x01  //!< 0 = data valid on rising edge, 1 = valid on falling edge
  #define IFACE_CFG_EPL         0x02  //!< 0 = ENABLE active low, 1 = active high
  #define IFACE_CFG_HSPL        0x08  //!< 0 = HSYNC active low, 1 = active high
  #define IFACE_CFG_VSPL        0x10  //!< 0 = HSYNC active low, 1 = active high
  #define IFACE_CFG_SDA_EN      0x80  //!< 1 = Enable SDA instaed of DOUT
  
#define CMD_SET_GAMMA         0xC8   //!< Set gamma
// Needs 12 bytes for parameters. Refer to datasheet

#define CMD_POWER_SETTING     0xD0   //!< Power supply configuration
/* WARNING: Depending on the supply voltage, the maximum voltages stated in the datasheet can be exceeded if the multipliers are set too high. This may damage the display module.
READ THE DATASHEET before changing the configuration */
  // First byte: Vci1 level. Must not exceed 3.0V
  #define VCI1_OFF              0x00  //!< Vci1 regulator off
  #define VCI1_070              0x01  //!< Vci1 = 0.70*Vci. Vci is externally supplied, or taken from internal 2.5V reference if INT_VCI_2V5_REF_EN is set
  #define VCI1_075              0x02  //!< Vci1 = 0.75*Vci
  #define VCI1_080              0x03  //!< Vci1 = 0.80*Vci
  #define VCI1_085              0x04  //!< Vci1 = 0.85*Vci
  #define VCI1_090              0x05  //!< Vci1 = 0.90*Vci
  #define VCI1_095              0x06  //!< Vci1 = 0.95*Vci
  #define VCI1_100              0x07  //!< Vci1 = 1.00*Vci
  //Second byte: TFT Voltage multipliers. Vgh max. 18V; Vgl max. -12.5V
  #define STUP_VGH6_VGL5        0x00  //!< Vgh = 6 * Vci1; Vgl = -5 * Vci1
  #define STUP_VGH6_VGL4        0x01  //!< Vgh = 6 * Vci1; Vgl = -4 * Vci1
  #define STUP_VGH6_VGL3        0x02  //!< Vgh = 6 * Vci1; Vgl = -3 * Vci1
  #define STUP_VGH5_VGL5        0x03  //!< Vgh = 5 * Vci1; Vgl = -5 * Vci1
  #define STUP_VGH5_VGL4        0x04  //!< Vgh = 5 * Vci1; Vgl = -4 * Vci1
  #define STUP_VGH5_VGL3        0x05  //!< Vgh = 5 * Vci1; Vgl = -3 * Vci1
  #define STUP_VGH4_VGL4        0x06  //!< Vgh = 4 * Vci1; Vgl = -4 * Vci1
  #define STUP_VGH4_VGL3        0x07  //!< Vgh = 4 * Vci1; Vgl = -3 * Vci1
  
  #define STUP_PON              0x40  //!< Enable step-up
  // Third byte
  #define INT_VCI_2V5_REF_EN    0x10  //!< VCIRE bit; 1 = enable internal 2.5V ref; 0 = use external ref
  // VREG1OUT should be between 4.0V and (DDVDH - 0.5V); with DDVDH = 2 * Vci1 => VREG1OUT usually between 4.0V and 4.5V
  #define VREGOUT1_OFF          0x00 //!< VREGOUT1 regulator off
  #define VREGOUT1_160          0x01 //!< VREGOUT1 = 1.60 * Vci
  #define VREGOUT1_165          0x02 //!< VREGOUT1 = 1.65 * Vci
  #define VREGOUT1_170          0x03 //!< VREGOUT1 = 1.70 * Vci
  #define VREGOUT1_175          0x04 //!< VREGOUT1 = 1.75 * Vci
  #define VREGOUT1_180          0x05 //!< VREGOUT1 = 1.80 * Vci
  #define VREGOUT1_185          0x06 //!< VREGOUT1 = 1.85 * Vci
  #define VREGOUT1_190          0x07 //!< VREGOUT1 = 1.90 * Vci
  #define VREGOUT1_195          0x08 //!< VREGOUT1 = 1.95 * Vci
  #define VREGOUT1_200          0x09 //!< VREGOUT1 = 2.00 * Vci
  #define VREGOUT1_205          0x0A //!< VREGOUT1 = 2.05 * Vci
  #define VREGOUT1_210          0x0B //!< VREGOUT1 = 2.10 * Vci
  #define VREGOUT1_220          0x0C //!< VREGOUT1 = 2.20 * Vci
  #define VREGOUT1_230          0x0D //!< VREGOUT1 = 2.30 * Vci
  #define VREGOUT1_240          0x0E //!< VREGOUT1 = 2.40 * Vci

#define CMD_VCOM_CTRL     0xD1   //!< Power supply configuration
  // First byte:
  #define VCOM_SELVCM         0x01 //!< 1 = Use VCM from NVM, 0 = Use VCM from this register
  // Second byte: VCOMH can be selected from 0.685*VREGOUT1 to 1.000*VREGOUT1 in 64 steps. Use the following macro with any number from 685 (0.685) to 1000 (=1.000) 
  #define CALC_VCOMH_MUL(X) ((X - 685) / 5)
  //Third byte: VCOM alternating amplitude can be selected from 0.70*VREGOUT1 to 1.32*VREGOUT1 in 32 steps. Use the following macro with any number from 700 (0.70) to 1320 (=1.320)
  #define CALC_VCOM_AC_MUL(X) ((X - 700) / 20)
  
#define CMD_NORM_PWR_SETTING  0xD2   //!< Power settings for normal mode
#define CMD_PART_PWR_SETTING  0xD3   //!< Power settings for partial mode
#define CMD_IDLE_PWR_SETTING  0xD4   //!< Power settings for idle mode
  //First byte (for all the power settings)
  // OpAmp currents higher = higher LCD switching speed, but also higher current consumption
  #define OPV_DISABLE           0x00  //!< OpAmps + step-up disabled, to save current when display is off
  #define OPV_GAMMA100_SRC100   0x01  //!< Gamma driver 100%, Source driver 100%
  #define OPV_GAMMA100_SRC075   0x02  //!< Gamma driver 100%, Source driver 75%
  #define OPV_GAMMA100_SRC050   0x03  //!< Gamma driver 100%, Source driver 50%
  #define OPV_GAMMA075_SRC100   0x04  //!< Gamma driver 75%, Source driver 100%
  #define OPV_GAMMA075_SRC075   0x05  //!< Gamma driver 75%, Source driver 75%
  #define OPV_GAMMA075_SRC050   0x06  //!< Gamma driver 75%, Source driver 50%
  #define OPV_GAMMA050_SRC050   0x07  //!< Gamma driver 50%, Source driver 50%
  // Second byte: DC/DC converter switching frequencies
  #define DCDC1_CKDIV_1         0x00  //!< Divide clock by 1 for first DC/DC converter 
  #define DCDC1_CKDIV_2         0x01  //!< Divide clock by 2 for first DC/DC converter
  #define DCDC1_CKDIV_4         0x02  //!< Divide clock by 4 for first DC/DC converter 
  #define DCDC1_CKDIV_8         0x03  //!< Divide clock by 8 for first DC/DC converter 
  #define DCDC1_CKDIV_16        0x04  //!< Divide clock by 16 for first DC/DC converter 
  #define DCDC1_CKDIV_32        0x05  //!< Divide clock by 32 for first DC/DC converter 
  #define DCDC1_CKDIV_64        0x06  //!< Divide clock by 64 for first DC/DC converter 
  #define DCDC1_HALT            0x07  //!< Halt first DC/DC converter 
  
  #define DCDC2_CKDIV_16        0x00  //!< Divide clock by 16 for second DC/DC converter 
  #define DCDC2_CKDIV_32        0x10  //!< Divide clock by 32 for second DC/DC converter 
  #define DCDC2_CKDIV_64        0x20  //!< Divide clock by 64 for second DC/DC converter 
  #define DCDC2_CKDIV_128       0x30  //!< Divide clock by 128 for second DC/DC converter 
  #define DCDC2_CKDIV_256       0x40  //!< Divide clock by 256 for second DC/DC converter
  #define DCDC2_CKDIV_512       0x50  //!< Divide clock by 512 for second DC/DC converter
  #define DCDC2_HALT            0x70  //!< Halt second DC/DC converter
