#ifndef INC_MAG_H_
#define INC_MAG_H_

//                 			Register Address, Auto-Increment Address (Fast Read)
#define DR_STATUS		0x00 	//R 0x00 0x01 0000 0000 Data ready status per axis
#define OUT_X_MSB 	0x01	//R 0x01 0x02 (0x03) data Bits [15:8] of X measurement
#define OUT_X_LSB 	0x02	//R 0x02 0x03 data Bits [7:0] of X measurement
#define OUT_Y_MSB 	0x03	//R 0x03 0x04 (0x05) data Bits [15:8] of Y measurement
#define OUT_Y_LSB 	0x04	//R 0x04 0x05 data Bits [7:0] of Y measurement
#define OUT_Z_MSB 	0x05	//R 0x05 0x06 (0x07) data Bits [15:8] of Z measurement
#define OUT_Z_LSB 	0x06	//R 0x06 0x07 data Bits [7:0] of Z measurement
#define WHO_AM_I  	0x07	//R 0x07 0x08 0xC4 Device ID Number
#define SYSMOD    	0x08	//R 0x08 0x09 data Current System Mode
#define OFF_X_MSB 	0x09	//R/W 0X09 0x0A 0000 0000 Bits [14:7] of user X offset
#define OFF_X_LSB 	0x0A 	//R/W 0X0A 0X0B 0000 0000 Bits [6:0] of user X offset
#define OFF_Y_MSB 	0x0B	//R/W 0X0B 0X0C 0000 0000 Bits [14:7] of user Y offset
#define OFF_Y_LSB 	0x0C	//R/W 0X0C 0X0D 0000 0000 Bits [6:0] of user Y offset
#define OFF_Z_MSB 	0x0D	//R/W 0X0D 0X0E 0000 0000 Bits [14:7] of user Z offset
#define OFF_Z_LSB 	0x0E	//R/W 0X0E 0X0F 0000 0000 Bits [6:0] of user Z offset
#define DIE_TEMP 		0x0F	//R 0X0F 0X10 data Temperature, signed 8 bits in °C
#define CTRL_REG1 	0x10	//R/W 0X10 0X11 0000 0000 Operation modes
#define CTRL_REG2 	0x11	//R/W 0X11 0x12 0000 0000 Operation modes

#define MAG3110_DEV_ADDR_W	0x1C
#define MAG3110_DEV_ADDR_R	0x1D

#define MAG3110_AUTO_MRST_EN 	0x80
#define MAG3110_ACTIVE_MODE		0x01
#define MAG3110_STANDBY_MODE	0x00
#define MAG3110_RAW_MODE			0x20
#define MAG3110_NORMAL_MODE		0x00


uint8_t const MAG3110_DR_OS_80_16 =    0x00;
uint8_t const MAG3110_DR_OS_40_32 =		0x08;
uint8_t const MAG3110_DR_OS_20_64 =		0x10;
uint8_t const MAG3110_DR_OS_10_128 =	  0x18;
uint8_t const MAG3110_DR_OS_40_16 =		0x20;
uint8_t const MAG3110_DR_OS_20_32 =		0x28;
uint8_t const MAG3110_DR_OS_10_64 =		0x30;
uint8_t const MAG3110_DR_OS_5_128 =		0x38;
uint8_t const MAG3110_DR_OS_20_16 =		0x40;
uint8_t const MAG3110_DR_OS_10_32 =		0x48;
uint8_t const MAG3110_DR_OS_5_64 =		  0x50;
uint8_t const MAG3110_DR_OS_2_5_128 =	0x58;
uint8_t const MAG3110_DR_OS_10_16 =		0x60;
uint8_t const MAG3110_DR_OS_5_32 =		  0x68;
uint8_t const MAG3110_DR_OS_2_5_64 =   0x70;
uint8_t const MAG3110_DR_OS_1_25_128 = 0x78;
uint8_t const MAG3110_DR_OS_5_16 =     0x80;
uint8_t const MAG3110_DR_OS_2_5_32 =	  0x88;
uint8_t const MAG3110_DR_OS_1_25_64	= 0x90;
uint8_t const MAG3110_DR_OS_0_63_128 = 0x98;
uint8_t const MAG3110_DR_OS_2_5_16 =   0xA0;
uint8_t const MAG3110_DR_OS_1_25_32 =  0xA8;
uint8_t const MAG3110_DR_OS_0_63_64 =  0xB0;
uint8_t const MAG3110_DR_OS_0_31_128 = 0xB8;
uint8_t const MAG3110_DR_OS_1_25_16 =  0xC0;
uint8_t const MAG3110_DR_OS_0_63_32 =  0xC8;
uint8_t const MAG3110_DR_OS_0_31_64 =  0xD0;
uint8_t const MAG3110_DR_OS_0_16_128 = 0xD8;
uint8_t const MAG3110_DR_OS_0_63_16 =  0xE0;
uint8_t const MAG3110_DR_OS_0_31_32 =  0xE8;
uint8_t const MAG3110_DR_OS_0_16_64 =  0xF0;
uint8_t const MAG3110_DR_OS_0_08_128 = 0xF8;


#endif /* INC_MAG_H_ */
