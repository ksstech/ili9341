/*
 * ili9341.h
 */

#pragma	once

#include "hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ########################################## Macros ###############################################

#define	ili9341GPIO_LIGHT			GPIO_NUM_5
#define	ili9341GPIO_RESET			GPIO_NUM_18
#define	ili9341GPIO_SCLK			GPIO_NUM_19
#define	ili9341GPIO_D_C_X			GPIO_NUM_21
#define ili9341GPIO_CS				GPIO_NUM_22
#define	ili9341GPIO_MOSI			GPIO_NUM_23
#define	ili9341GPIO_MISO			GPIO_NUM_25

#define	ili9341LCD_OVERCLOCK		1

#define ili9341LINES_PARALLEL 		16

#define	ili9341WR_DSP_BRIGHTNESS	0x51

// ######################################### Enumerations ##########################################

enum {
	SET_COL_AD	= 0x2A,
	SET_PAG_AD	= 0x2B,
	WR_MEM		= 0x2C,
} ;

// ##################################### GLOBAL variables ##########################################


// ##################################### Functions prototypes ######################################

int	ili9341Init(void) ;
int	ili9341Config(int DevType) ;
int	ili9341DeInit(void) ;

void ili9341SendCombo(const u8_t cmd, const u8_t * data, int len);
void ili9341BacklightInit(void);
void ili9341BacklightLevel(u8_t Percent);

void ili9341TestInit(void) ;
void ili9341TestUpdate(void) ;

#ifdef __cplusplus
}
#endif
