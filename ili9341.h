// ili9341.h

#pragma	once

#include "definitions.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ########################################## Macros ###############################################

#define	ili9341WRDISBV				0x51
#define	ili9341WRCTRLD				0x53
#define	ili9341WRCTRLD_BCTRL		0x20
#define	ili9341WRCTRLD_DD			0x08
#define	ili9341WRCTRLD_BL			0x04

// ######################################### Enumerations ##########################################
// ##################################### GLOBAL variables ##########################################
// ##################################### Functions prototypes ######################################

int	ili9341InitSPI(void);
int	ili9341Config(int DevType);
int	ili9341DeInitSPI(void);

void ili9341SendCommand(const u8_t cmd);
void ili9341SendData(const u8_t * data, int len);
void ili9341SendCombo(const u8_t cmd, const u8_t * data, int len);

void ili9341BacklightInit(void);
u8_t ili9341BacklightLevel(u8_t Percent);
void ili9341BackLightStatus(bool Status);

int ili9341PutChar(int cChr);
void ili9341PutString(const char *pString);

#ifdef __cplusplus
}
#endif
