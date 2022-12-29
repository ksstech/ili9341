/*
 * ili9341.h
 */

#pragma	once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ########################################## Macros ###############################################

#define ENABLE_EPID					0

// ######################################### Enumerations ##########################################


// ##################################### GLOBAL variables ##########################################


// ##################################### Functions prototypes ######################################

int	ili9341InitSPI(void) ;
int	ili9341Config(int DevType) ;
int	ili9341DeInitSPI(void) ;

void ili9341SendCommand(const uint8_t cmd);
void ili9341SendData(const uint8_t * data, int len);
void ili9341SendCombo(const uint8_t cmd, const uint8_t * data, int len);

void ili9341BacklightInit(void);
void ili9341BacklightLevel(uint8_t Percent);
void ili9341BackLightStatus(bool Status);

void ili9341TestInit(void) ;
void ili9341TestUpdate(void) ;

#ifdef __cplusplus
}
#endif
