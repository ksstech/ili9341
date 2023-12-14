/*
 * ili9341.h
 */

#pragma	once

#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

// ########################################## Macros ###############################################


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
void ili9341BacklightLevel(u8_t Percent);
void ili9341BackLightStatus(bool Status);

int ili9341PutChar(int cChr);
void ili9341PutString(const char *pString);

#ifdef __cplusplus
}
#endif
